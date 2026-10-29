#!/usr/bin/env python3
"""Optional regression of preserved VIO + appended 2D features using recorded real keyframes.

Normal operation remains separate launch files and explicit service calls.
"""
import argparse
import copy
import io
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import traceback
import xmlrpc.client


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--keyframes", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--calib-a", type=Path, required=True)
    parser.add_argument("--calib-b", type=Path, required=True)
    parser.add_argument("--frame-a", type=int, required=True)
    parser.add_argument("--frame-b", type=int, required=True)
    parser.add_argument("--reference-json", type=Path)
    parser.add_argument("--port", type=int, default=11584)
    parser.add_argument("--output", type=Path, default=Path("logs/peer_2d_services"))
    args = parser.parse_args()
    import rosbag
    from rdswarm_msgs.msg import Keyframe, LoopConstraint
    from rdswarm_msgs.srv import DescribeKeyframe, GetLocalFeatures, FetchPeerFeatures, VerifyLoop
    import rospy
    root = Path(__file__).resolve().parents[1]
    args.output.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    for key in ["ROS_HOSTNAME", "ROS_NAMESPACE", "ROS_LOG_FILENAME"]:
        env.pop(key, None)
        os.environ.pop(key, None)
    env.update(ROS_MASTER_URI=f"http://127.0.0.1:{args.port}", ROS_IP="127.0.0.1", ROS_LOG_DIR=str(args.output.resolve()))
    os.environ.update({key: env[key] for key in ["ROS_MASTER_URI", "ROS_IP", "ROS_LOG_DIR"]})
    processes, handles = [], []
    report = {"scope": "recorded real keyframes; same-master services; no VIO accuracy or wireless claim", "passed": False}

    def start(name, command):
        handle = (args.output / (name + ".log")).open("w")
        handles.append(handle)
        process = subprocess.Popen(command, cwd=root, env=env, stdout=handle, stderr=subprocess.STDOUT)
        processes.append(process)
        return process

    try:
        frames = {}
        with rosbag.Bag(str(args.keyframes)) as bag:
            for _, frame, _ in bag.read_messages():
                if (frame.key.agent_id, frame.key.frame_id) in [(0, args.frame_a), (1, args.frame_b)]:
                    # rosbag 可生成动态消息类；统一为当前生成类后再比较，避免字段相同却因类不同而失败。 (Ref: 2026-10-25_Extend_Peer_2D_Features.md)
                    serialized = io.BytesIO()
                    frame.serialize(serialized)
                    canonical = Keyframe()
                    canonical.deserialize(serialized.getvalue())
                    frames[canonical.key.agent_id] = canonical
        assert len(frames) == 2, "requested raw frames not found"
        with socket.socket() as probe:
            probe.bind(("127.0.0.1", args.port))
        master = start("roscore", ["roscore", "-p", str(args.port)])
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            assert master.poll() is None, "master exited"
            try:
                if xmlrpc.client.ServerProxy(env["ROS_MASTER_URI"]).getPid("/test")[0] == 1:
                    break
            except (OSError, xmlrpc.client.Error):
                time.sleep(0.1)
        else:
            raise RuntimeError("master not ready")
        for agent, calibration in [(0, args.calib_a), (1, args.calib_b)]:
            ns = f"/peer2d/agent{agent}"
            start(f"features{agent}", [str(root / "devel_cpu/lib/rdswarm_loop/rdswarm_feature_node"),
                  f"__ns:={ns}", "__name:=features", f"_model_path:={args.model.resolve()}",
                  f"_camera_calib_file:={calibration.resolve()}", "_auto_describe:=false",
                  "_append_image_features:=true", "keyframes_in:=vins/keyframes"])
            start(f"loop{agent}", [str(root / "devel_cpu/lib/rdswarm_loop/rdswarm_loop_node"),
                  f"__ns:={ns}", "__name:=loop_core"])
        rospy.init_node("peer_2d_services_test", anonymous=True, disable_signals=True)
        observed = {0: [], 1: []}
        published_constraints = []
        subscriptions = [rospy.Subscriber(f"/peer2d/agent{i}/keyframes", Keyframe,
                         lambda message, i=i: observed[i].append(message), queue_size=10) for i in [0, 1]]
        subscriptions.append(rospy.Subscriber("/peer2d/agent0/loop_constraints", LoopConstraint, published_constraints.append))
        publishers = [rospy.Publisher(f"/peer2d/agent{i}/vins/keyframes", Keyframe, queue_size=1) for i in [0, 1]]
        for agent in [0, 1]:
            for service in ["describe_keyframe", "get_local_features", "fetch_peer_features", "verify_loop"]:
                rospy.wait_for_service(f"/peer2d/agent{agent}/{service}", timeout=15)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and not all(p.get_num_connections() for p in publishers):
            time.sleep(0.05)
        assert all(p.get_num_connections() for p in publishers)
        for agent in [0, 1]:
            publishers[agent].publish(frames[agent])
        time.sleep(0.3)
        assert not any(observed.values()), "automatic inference unexpectedly ran"
        counts = {}
        for agent in [0, 1]:
            describe = rospy.ServiceProxy(f"/peer2d/agent{agent}/describe_keyframe", DescribeKeyframe)
            response = describe(frames[agent].key, True)
            assert response.success, response.status
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline and not observed[agent]:
                time.sleep(0.02)
            assert observed[agent], "enriched frame not published"
            enriched = observed[agent][-1]
            original = frames[agent]
            assert len(enriched.features) > len(original.features), "no added image features"
            assert enriched.key == original.key and enriched.header.stamp == original.header.stamp, "frame identity or timestamp changed"
            for actual, expected in zip(enriched.features, original.features):
                original_fields = copy.deepcopy(actual)
                original_fields.descriptor = copy.deepcopy(expected.descriptor)
                original_fields.descriptor_valid = expected.descriptor_valid
                assert original_fields == expected, "original geometry or ID changed"
            for feature in enriched.features[len(original.features):]:
                assert feature.feature_id >= 1 << 63 and not feature.position_valid
                assert feature.position_camera.x == feature.position_camera.y == feature.position_camera.z == 0
            assert len({f.feature_id for f in enriched.features}) == len(enriched.features)
            repeated = describe(original.key, True)
            assert repeated.success and repeated.feature_count == response.feature_count
            counts[agent] = {"original": len(original.features), "total": len(enriched.features),
                             "valid_3d": sum(f.position_valid for f in enriched.features)}
        get_peer = rospy.ServiceProxy("/peer2d/agent1/get_local_features", GetLocalFeatures)
        deadline = time.monotonic() + 5
        while True:
            peer = get_peer(frames[1].key, False)
            if peer.success:
                break
            if time.monotonic() > deadline:
                raise RuntimeError(peer.status)
            time.sleep(0.02)
        assert len(peer.packet.features) == counts[1]["total"]
        assert all(not f.position_valid for f in peer.packet.features)
        assert "image" not in peer.packet.__slots__
        payload = io.BytesIO()
        peer.packet.serialize(payload)
        fetch = rospy.ServiceProxy("/peer2d/agent0/fetch_peer_features", FetchPeerFeatures)(
            frames[1].key, "/peer2d/agent1/get_local_features", False)
        assert fetch.success and fetch.feature_count == counts[1]["total"], fetch.status
        result = rospy.ServiceProxy("/peer2d/agent0/verify_loop", VerifyLoop)(frames[0].key, frames[1].key, False)
        if args.reference_json:
            reference = json.loads(args.reference_json.read_text())
            assert bool(result.success) == bool(reference["success"])
            if result.success:
                assert result.constraint.inlier_count == reference["inliers"]
        assert not published_constraints, "constraint published despite explicit false"
        report.update(passed=True, features=counts, remote_image_transmitted=False, remote_valid_3d=0,
                      peer_packet_bytes=len(payload.getvalue()), fetch_success=fetch.success,
                      pnp_success=result.success, pnp_status=result.status, inliers=result.constraint.inlier_count,
                      covariance_valid=result.constraint.covariance_valid)
    except Exception as error:
        report["error"] = repr(error)
        report["traceback"] = traceback.format_exc()
    finally:
        # 只结束本测试启动的进程，不干扰用户现有 master 与节点。 (Ref: 2026-10-25_Extend_Peer_2D_Features.md)
        for process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
        for process in reversed(processes):
            try:
                process.wait(timeout=7)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)
        for handle in handles:
            handle.close()
        (args.output / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
