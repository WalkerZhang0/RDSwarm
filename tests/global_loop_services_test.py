#!/usr/bin/python3
"""Real model/recorded frames integration, isolated master and automatic BF/PnP."""
import argparse
import io
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import xmlrpc.client


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('logs/global_loop_services'))
    parser.add_argument('--port', type=int, default=11628)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    if (args.output / 'summary.json').exists():
        parser.error('use_a_new_output_directory')
    env = os.environ.copy()
    for name in ['ROS_HOSTNAME', 'ROS_NAMESPACE', 'ROS_LOG_FILENAME']:
        env.pop(name, None); os.environ.pop(name, None)
    env.update(ROS_MASTER_URI='http://127.0.0.1:{}'.format(args.port), ROS_IP='127.0.0.1',
               ROS_LOG_DIR=str(args.output))
    os.environ.update({k: env[k] for k in ['ROS_MASTER_URI','ROS_IP','ROS_LOG_DIR']})
    processes, handles = [], []
    report = dict(passed=False, scope='recorded real keyframes, real ONNX CPU, same-master automation; no GT pose claim')

    def start(name, command):
        handle = (args.output / (name+'.log')).open('w'); handles.append(handle)
        process = subprocess.Popen(command, cwd=root, env=env, stdout=handle, stderr=subprocess.STDOUT)
        processes.append(process)
        return process

    try:
        with socket.socket() as sock: sock.bind(('127.0.0.1', args.port))
        master = start('master', ['roscore','-p',str(args.port)])
        deadline=time.monotonic()+15
        while time.monotonic()<deadline:
            assert master.poll() is None, 'master_exit'
            try:
                if xmlrpc.client.ServerProxy(env['ROS_MASTER_URI']).getPid('/test')[0]==1: break
            except (OSError, xmlrpc.client.Error): pass
            time.sleep(.1)
        else: raise RuntimeError('master_timeout')
        import rosbag
        import rospy
        from rdswarm_msgs.msg import Keyframe, KeyframeSummary, VioState, LoopCandidate
        from rdswarm_msgs.srv import DescribeKeyframe, GetLocalFeatures
        from sensor_msgs.msg import Image
        summaries='/globaltest/summaries'
        frames=[]
        with rosbag.Bag(str(root/'results/s3e_loop_validation/keyframes.bag')) as bag:
            for _, raw, _ in bag.read_messages():
                if getattr(raw,'_type','')=='rdswarm_msgs/Keyframe':
                    buffer=io.BytesIO();raw.serialize(buffer)
                    frames.append(Keyframe().deserialize(buffer.getvalue()))
                    if len(frames)>=16: break
        assert {f.key.agent_id for f in frames}=={0,1}
        for agent,calibration in [(0,'alpha_cam0.yaml'),(1,'carol_cam0.yaml')]:
            ns='/globaltest/agent{}'.format(agent)
            base=[str(root/'devel_cpu/lib/rdswarm_loop/rdswarm_feature_node'),
                  '__ns:='+ns,'__name:=features','keyframes_in:=vins/keyframes',
                  '_model_path:='+str(root/'models/superpoint_dense.onnx'),
                  '_camera_calib_file:='+str(root/'src/rdswarm_bringup/config/s3e'/calibration),
                  '_cache_directory:='+str(args.output/str(agent)/'features'), '_memory_keyframes:=1']
            start('features'+str(agent),base)
            start('loop'+str(agent),[str(root/'devel_cpu/lib/rdswarm_loop/rdswarm_loop_node'),
                  '__ns:='+ns,'__name:=loop_core','summaries:='+summaries,
                  '_cache_directory:='+str(args.output/str(agent)/'loop'),
                  '_memory_keyframes:=1','_remote_cache_limit:=1'])
            start('global'+str(agent),['/usr/bin/python3',str(root/'src/rdswarm_loop/scripts/global_descriptor_node.py'),
                  '__ns:='+ns,'__name:=global_descriptor',
                  '_model_path:='+str(root/'models/mobilenetvlad.onnx'),
                  '_cache_directory:='+str(args.output/str(agent)/'global'),
                  '_fallback_enabled:=true','_fallback_period_s:=3.0',
                  'keyframes_in:=vins/keyframes','summaries_out:='+summaries,'summaries_in:='+summaries,
                  'states_in:=vins/states','images_in:=images'])
        start('baseline',['/usr/bin/python3',str(root/'src/rdswarm_loop/scripts/loop_baseline_node.py'),
              '__ns:=/globaltest/agent0','__name:=baseline','_agent_id:=0','_top_k:=2',
              '_output_directory:='+str(args.output/'baseline'),
              "_agent_namespaces:={'0': '/globaltest/agent0', '1': '/globaltest/agent1'}",
              'summaries_in:='+summaries])
        rospy.init_node('global_loop_test',anonymous=True,disable_signals=True)
        observed=[];fallback=[]
        subs=[rospy.Subscriber(summaries,KeyframeSummary,observed.append,queue_size=100),
              rospy.Subscriber('/globaltest/agent0/fallback_candidates',LoopCandidate,fallback.append,queue_size=20)]
        pubs={a:rospy.Publisher('/globaltest/agent{}/vins/keyframes'.format(a),Keyframe,queue_size=20) for a in [0,1]}
        image_pub=rospy.Publisher('/globaltest/agent0/images',Image,queue_size=2)
        state_pub=rospy.Publisher('/globaltest/agent0/vins/states',VioState,queue_size=2)
        for a in [0,1]:
            rospy.wait_for_service('/globaltest/agent{}/describe_keyframe'.format(a),timeout=20)
        deadline=time.monotonic()+20
        while time.monotonic()<deadline and not all(p.get_num_connections()>=2 for p in pubs.values()):time.sleep(.1)
        assert all(p.get_num_connections()>=2 for p in pubs.values()),'global_or_feature_subscriber_missing'
        time.sleep(.5)
        for frame in frames:
            pubs[frame.key.agent_id].publish(frame)
            time.sleep(.06)
        deadline=time.monotonic()+60
        events=[]
        while time.monotonic()<deadline:
            assert all(p.poll() is None for p in processes),'child_exited'
            path=args.output/'baseline/loops.jsonl'
            events=[json.loads(l) for l in path.read_text().splitlines()] if path.exists() else []
            if len(observed)>=len(frames) and sum(e['stage']=='verification' for e in events)>=8:break
            time.sleep(.2)
        assert len(observed)==len(frames),(len(observed),len(frames))
        assert all(len(s.global_descriptor)==4096 for s in observed)
        assert sum(e['stage']=='verification' for e in events)>=8,'automatic_verification_missing'
        assert any(e['stage']=='verification' and e['success'] for e in events),'no_geometry_acceptance'
        first=frames[0]
        ns='/globaltest/agent{}'.format(first.key.agent_id)
        response=rospy.ServiceProxy(ns+'/describe_keyframe',DescribeKeyframe)(first.key,True)
        assert response.success,response.status
        time.sleep(.2)
        served=rospy.ServiceProxy(ns+'/get_local_features',GetLocalFeatures)(first.key,False)
        assert served.success and all(not f.position_valid for f in served.packet.features)
        assert 'image' not in served.packet.__slots__
        before=len(observed)
        latest=max((f for f in frames if f.key.agent_id==0),key=lambda f:f.header.stamp.to_nsec())
        search_time=latest.header.stamp+rospy.Duration(4)
        image=latest.image;image.header.stamp=search_time
        state=VioState();state.header=latest.header;state.header.stamp=search_time
        state.agent_id=0;state.session_id=latest.key.session_id;state.initialized=True
        state.body_pose_vio=latest.body_pose_vio;state.vio_frame_id=latest.vio_frame_id
        for _ in range(5):
            image_pub.publish(image);state_pub.publish(state);time.sleep(.1)
        deadline=time.monotonic()+10
        while time.monotonic()<deadline and not fallback:time.sleep(.1)
        assert fallback,'fallback_search_missing'
        assert len(observed)==before,'fallback_wrongly_indexed_as_keyframe'
        time.sleep(.4)
        log=args.output/'0/global/global.jsonl'
        fallback_calls=[e for e in map(json.loads,log.read_text().splitlines()) if e['stage']=='global_descriptor' and e['trigger']=='fallback']
        assert len(fallback_calls)==1,'fallback_not_rate_limited'
        report.update(passed=True,keyframes=len(frames),global_descriptors=len(observed),
            automatic_verifications=sum(e['stage']=='verification' for e in events),
            accepted=sum(e['stage']=='verification' and e['success'] for e in events),
            archive_reload=True,peer_image_transmitted=False,fallback_candidates=len(fallback),
            fallback_indexed=False,fallback_inferences=len(fallback_calls))
    except Exception as error:
        import traceback
        report.update(error=str(error),traceback=traceback.format_exc())
    finally:
        for process in reversed(processes):
            if process.poll() is None:process.send_signal(signal.SIGINT)
        for process in reversed(processes):
            try:process.wait(timeout=8)
            except subprocess.TimeoutExpired:process.kill();process.wait()
        for handle in handles:handle.close()
        (args.output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
