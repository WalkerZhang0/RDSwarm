#!/usr/bin/env python3
"""Optional CI regression only; normal operation is documented as separate nodes/services."""
import argparse,io,json,os,signal,socket,subprocess,sys,time,xmlrpc.client,importlib.util
from pathlib import Path
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument("--model",type=Path,required=True)
parser.add_argument("--devel-space",type=Path,default=Path("devel_cpu"))
parser.add_argument("--port",type=int,default=11541)
args=parser.parse_args()
root=Path(__file__).resolve().parents[1]
devel=args.devel_space if args.devel_space.is_absolute() else root/args.devel_space
logdir=root/"logs"/"phase2_integration"
logdir.mkdir(parents=True,exist_ok=True)
env=os.environ.copy()
for k in ['ROS_HOSTNAME','ROS_NAMESPACE','ROS_LOG_FILENAME']:env.pop(k,None)
env.update(ROS_MASTER_URI=f"http://127.0.0.1:{args.port}",ROS_IP='127.0.0.1',ROS_LOG_DIR=str(logdir))
os.environ.update({k:env[k] for k in ['ROS_MASTER_URI','ROS_IP','ROS_LOG_DIR']})
for k in ['ROS_HOSTNAME','ROS_NAMESPACE','ROS_LOG_FILENAME']:os.environ.pop(k,None)
processes=[];handles=[];report={'scope':'synthetic two-agent interface/inference test, not field accuracy or retrieval quality'}
def start(label,args):
 h=(logdir/(label+'.log')).open('w');handles.append(h)
 p=subprocess.Popen(args,cwd=root,env=env,stdout=h,stderr=subprocess.STDOUT);processes.append(p);return p
try:
 with socket.socket() as probe:probe.bind(("127.0.0.1",args.port))
 if not args.model.is_file():raise RuntimeError("model file does not exist")
 master=start('roscore',['roscore','-p',str(args.port)])
 deadline=time.monotonic()+15
 while time.monotonic()<deadline:
  if master.poll() is not None:raise RuntimeError("test master exited early")
  try:
   if xmlrpc.client.ServerProxy(env['ROS_MASTER_URI']).getPid('/test')[0]==1:break
  except Exception:time.sleep(.1)
 else:raise RuntimeError('master not ready')
 for i in [0,1]:
  ns=f'/phase2/agent{i}'
  start(f'features{i}',[str(devel/'lib/rdswarm_loop/rdswarm_feature_node'),f'__ns:={ns}','__name:=features',f'_model_path:={args.model.resolve()}','_auto_describe:=false','_append_image_features:=false','keyframes_in:=vins/keyframes'])
  start(f'loop{i}',[str(devel/'lib/rdswarm_loop/rdswarm_loop_node'),f'__ns:={ns}','__name:=loop_core'])
 import rospy
 from rdswarm_msgs.msg import Keyframe,FrameKey,KeyframeSummary,LoopConstraint
 from rdswarm_msgs.srv import DescribeKeyframe,GetLocalFeatures,FetchPeerFeatures,VerifyLoop,QueryLoopCandidates
 rospy.init_node('phase2_integration',anonymous=True,disable_signals=True)
 names={
  'd0':'/phase2/agent0/describe_keyframe','d1':'/phase2/agent1/describe_keyframe',
  'get1':'/phase2/agent1/get_local_features','fetch':'/phase2/agent0/fetch_peer_features',
  'verify':'/phase2/agent0/verify_loop','query':'/phase2/agent0/query_candidates'}
 for name in names.values():rospy.wait_for_service(name,15)
 spec=importlib.util.spec_from_file_location('fixture',root/'tests/publish_loop_fixture.py');fixture=importlib.util.module_from_spec(spec);spec.loader.exec_module(fixture)
 frames=[fixture.frame(i) for i in [0,1]]
 pubs=[rospy.Publisher(f'/phase2/agent{i}/vins/keyframes',Keyframe,queue_size=1) for i in [0,1]]
 observed=[[],[]]
 subs=[rospy.Subscriber(f'/phase2/agent{i}/keyframes',Keyframe,lambda x,i=i:observed[i].append(x)) for i in [0,1]]
 deadline=time.monotonic()+5
 while time.monotonic()<deadline and not all(x.get_num_connections() for x in pubs):time.sleep(.05)
 for p,f in zip(pubs,frames):p.publish(f)
 time.sleep(.3)
 assert observed==[[],[]], 'auto_describe unexpectedly ran'
 for i in [0,1]:
  call=rospy.ServiceProxy(names[f'd{i}'],DescribeKeyframe)
  deadline=time.monotonic()+5
  while True:
   result=call(frames[i].key,True)
   if result.success:break
   if time.monotonic()>deadline:raise RuntimeError(result.status)
   time.sleep(.05)
  assert result.valid_descriptor_count==len(frames[i].features), result
 deadline=time.monotonic()+5
 while time.monotonic()<deadline and not all(observed):time.sleep(.02)
 assert all(observed),'enriched frames not received'
 for i in [0,1]:
  actual=observed[i][-1]
  assert actual.header.stamp==frames[i].header.stamp
  assert [f.feature_id for f in actual.features]==[f.feature_id for f in frames[i].features]
  assert all(len(f.descriptor)==256 and f.descriptor_valid for f in actual.features)
 verify=rospy.ServiceProxy(names['verify'],VerifyLoop)
 deadline=time.monotonic()+5
 while True:
  before=verify(frames[0].key,frames[1].key,False)
  if before.status=='peer_features_missing_call_fetch_first':break
  if time.monotonic()>deadline:raise RuntimeError(before.status)
  time.sleep(.05)
 assert not before.success
 malformed=rospy.ServiceProxy(names['fetch'],FetchPeerFeatures)(frames[1].key,'/invalid service',False)
 assert not malformed.success and malformed.status=='invalid_peer_service_name'
 rospy.set_param('/phase2/agent0/loop_core/matching/ratio_threshold',1.2)

 packet=rospy.ServiceProxy(names['get1'],GetLocalFeatures)(frames[1].key,False)
 assert packet.success and all(not f.position_valid for f in packet.packet.features)
 assert 'image' not in packet.packet.__slots__
 buffer=io.BytesIO();packet.packet.serialize(buffer)
 fetch=rospy.ServiceProxy(names['fetch'],FetchPeerFeatures)(frames[1].key,names['get1'],False)
 assert fetch.success,fetch.status
 invalid_options=verify(frames[0].key,frames[1].key,False)
 assert not invalid_options.success and invalid_options.status=='input:invalid_options'
 rospy.set_param('/phase2/agent0/loop_core/matching/ratio_threshold',0.8)
 result=verify(frames[0].key,frames[1].key,False)
 assert result.success,result.status
 assert result.constraint.geometry_verified and not result.constraint.covariance_valid
 t=result.constraint.relative_pose.position
 assert (t.x*t.x+t.y*t.y+t.z*t.z)**.5<1e-3,(t,result.status)
 assert result.constraint.inlier_count>=8
 assert len(result.correspondences.matches)>=8
 summary_pub=rospy.Publisher('/phase2/agent0/summaries',KeyframeSummary,queue_size=10)
 deadline=time.monotonic()+5
 while time.monotonic()<deadline and not summary_pub.get_num_connections():time.sleep(.05)
 for i in [0,1]:
  s=KeyframeSummary();s.header=frames[i].header;s.key=frames[i].key
  s.global_descriptor_valid=True;s.global_descriptor_model='synthetic_index_test_only';s.global_descriptor=[1.,0.1*i,0.]
  summary_pub.publish(s)
 time.sleep(.2)
 query=rospy.ServiceProxy(names['query'],QueryLoopCandidates)(frames[0].key,2,True,5.)
 assert query.success and len(query.candidates)==1,query.status
 report.update(passed=True,descriptors_per_agent=[len(observed[i][-1].features) for i in [0,1]],descriptor_dimension=256,
  automatic_work_disabled=True,missing_peer_rejected=True,invalid_service_rejected=True,live_parameter_validation=True,remote_image_transmitted=False,remote_valid_3d_points=0,
  remote_packet_bytes=len(buffer.getvalue()),verified_inliers=result.constraint.inlier_count,
  normalized_reprojection_rms=result.constraint.reprojection_error,nominal_covariance_enabled=False,
  verified_matches_reserved=True,synthetic_global_index_candidates=len(query.candidates))
except Exception as e:
 report.update(passed=False,error=repr(e))
finally:
 for p in reversed(processes):
  if p.poll() is None:p.send_signal(signal.SIGINT)
 for p in reversed(processes):
  try:p.wait(timeout=7)
  except subprocess.TimeoutExpired:p.kill();p.wait(timeout=3)
 for h in handles:h.close()
 (logdir/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
sys.exit(0 if report['passed'] else 1)
