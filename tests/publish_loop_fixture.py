#!/usr/bin/env python3
"""Publish two synthetic raw keyframes only; run describe/fetch/verify yourself."""
import argparse
import numpy as np
import rospy
from cv_bridge import CvBridge
from rdswarm_msgs.msg import Keyframe,TrackedFeature

def frame(agent):
    out=Keyframe()
    out.header.stamp=rospy.Time(1760000001+agent,123456789)
    out.header.frame_id=f"vio/agent{agent}/fixture"
    out.key.agent_id=agent
    out.key.session_id="fixture"
    out.key.frame_id=11 if agent==0 else 22
    out.vio_frame_id=out.header.frame_id
    out.body_pose_vio.orientation.w=1
    out.body_T_camera.rotation.w=1
    out.initialized=True
    out.capture_stamp_exact=True
    rng=np.random.RandomState(7)
    image=rng.randint(0,256,(120,160),dtype=np.uint8)
    for y in range(18,110,17):
        for x in range(18,150,19):
            feature=TrackedFeature()
            feature.feature_id=len(out.features)+(0 if agent==0 else 1000)
            feature.pixel.x=x;feature.pixel.y=y
            feature.normalized.x=(x-80)/120
            feature.normalized.y=(y-60)/120
            feature.normalized.z=1
            depth=4+0.07*len(out.features)
            feature.position_camera.x=feature.normalized.x*depth
            feature.position_camera.y=feature.normalized.y*depth
            feature.position_camera.z=depth
            feature.position_valid=True
            out.features.append(feature)
    out.image=CvBridge().cv2_to_imgmsg(image,"mono8")
    out.image.header=out.header
    out.image_included=True
    return out

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prefix",default="",help="Optional namespace prefix, e.g. /phase2")
    args=parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("publish_loop_fixture",anonymous=True)
    pubs=[rospy.Publisher(f"{args.prefix}/agent{i}/vins/keyframes",Keyframe,queue_size=1) for i in range(2)]
    deadline=rospy.get_time()+10
    while not all(p.get_num_connections() for p in pubs):
        if rospy.get_time()>deadline:raise RuntimeError("Start both feature nodes before publishing fixture")
        rospy.sleep(0.05)
    for i,p in enumerate(pubs):p.publish(frame(i))
    rospy.sleep(0.5)
    print("Published synthetic identity-pose loop; this is an interface fixture, not accuracy evidence.")
    print("agent0 key: {agent_id: 0, session_id: fixture, frame_id: 11}")
    print("agent1 key: {agent_id: 1, session_id: fixture, frame_id: 22}")
if __name__=="__main__":
    main()
