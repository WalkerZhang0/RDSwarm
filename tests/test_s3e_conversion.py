#!/usr/bin/env python3
"""Small ROS2->native ROS1 regression; no dataset download or ROS master required."""
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import cv2
import numpy as np
import rosbag
from genpy import Time
from std_msgs.msg import Header
from sensor_msgs.msg import CompressedImage,Imu
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores,get_typestore

class ConversionTest(unittest.TestCase):
    def test_headers_values_images_and_native_lz4(self):
        script=Path(__file__).resolve().parents[1]/"scripts/convert_s3e_ros1.py"
        ros2=get_typestore(Stores.ROS2_HUMBLE)
        image=np.arange(48*64,dtype=np.uint8).reshape(48,64)
        ok,jpeg=cv2.imencode(".jpg",image);self.assertTrue(ok)
        expected_gray=cv2.imdecode(jpeg,cv2.IMREAD_GRAYSCALE)
        with tempfile.TemporaryDirectory(prefix="rdswarm_s3e_test_") as tmp:
            root=Path(tmp);source=root/"source"
            expected={}
            with Writer(source) as writer:
                for agent in ["Alpha","Carol"]:
                    ci=writer.add_connection(f"/{agent}/left_camera/compressed",
                        "sensor_msgs/msg/CompressedImage",typestore=ros2)
                    cu=writer.add_connection(f"/{agent}/imu/data",
                        "sensor_msgs/msg/Imu",typestore=ros2)
                    for i in range(2):
                        stamp=Time(100+i,123456789)
                        record=stamp.to_nsec()+1234
                        camera=CompressedImage(header=Header(seq=42,stamp=stamp,frame_id="camera"),
                            format="jpeg",data=jpeg.tobytes())
                        imu=Imu();imu.header=Header(seq=33,stamp=stamp,frame_id="imu")
                        imu.orientation.w=1.0
                        imu.angular_velocity.x=0.123+i
                        imu.linear_acceleration.x=-0.75;imu.linear_acceleration.z=9.8
                        for conn,msg in [(ci,camera),(cu,imu)]:
                            out=io.BytesIO();msg.serialize(out)
                            writer.write(conn,record,ros2.ros1_to_cdr(out.getvalue(),conn.msgtype))
                        expected[(agent,record)]=stamp.to_nsec()
            for mode in ["mono8","compressed"]:
                destination=root/mode
                command=[sys.executable,str(script),"--source",str(source),"--output",str(destination),
                    "--agents","Alpha","Carol","--image-mode",mode,"--compression","lz4"]
                completed=subprocess.run(command,text=True,capture_output=True)
                self.assertEqual(completed.returncode,0,completed.stderr)
                report=json.loads((destination/"conversion.json").read_text())
                self.assertEqual(report["status"],"complete")
                for agent in ["Alpha","Carol"]:
                    with rosbag.Bag(report["output_bags"][agent]) as output:
                        self.assertEqual(output.get_message_count(),4)
                        for topic,msg,t in output.read_messages():
                            self.assertEqual(msg.header.stamp.to_nsec(),expected[(agent,t.to_nsec())])
                            self.assertEqual(msg.header.seq,0)
                            if topic.endswith("/imu/data"):
                                self.assertEqual(msg.linear_acceleration.z,9.8)
                                self.assertEqual(msg.angular_velocity.x,0.123+(msg.header.stamp.secs-100))
                            elif mode=="mono8":
                                self.assertEqual(msg.encoding,"mono8")
                                self.assertEqual(bytes(msg.data),expected_gray.tobytes())
                            else:
                                self.assertEqual(bytes(msg.data),jpeg.tobytes())
                repeated=subprocess.run(command,text=True,capture_output=True)
                self.assertNotEqual(repeated.returncode,0)
                self.assertIn("Refusing to overwrite",repeated.stderr)

if __name__=="__main__":
    unittest.main()
