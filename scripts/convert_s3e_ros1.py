#!/usr/bin/env python3
"""Convert selected S3E ROS2 camera/IMU topics into separate ROS1 bags. No ROS nodes are started."""
import argparse
from contextlib import ExitStack
import importlib.metadata
import json
from pathlib import Path
import time

import cv2
import numpy as np
from rosbags.highlevel import AnyReader
import rosbag
from genpy import Time as RosTime
from std_msgs.msg import Header as RosHeader
from sensor_msgs.msg import Image as RosImage,Imu as RosImu,CompressedImage as RosCompressedImage
from rosbags.typesys import Stores,get_typestore

def header_ns(header):
    return int(header.stamp.sec)*1_000_000_000+int(header.stamp.nanosec)

def new_stats(topic,output_topic):
    return {"source_topic":topic,"output_topic":output_topic,"messages":0,
            "first_record_ns":None,"last_record_ns":None,
            "first_header_ns":None,"last_header_ns":None,
            "header_duplicates":0,"header_regressions":0,
            "record_minus_header_min_ns":None,"record_minus_header_max_ns":None,
            "image_dimensions":[]}

def update_stats(stats,record_ns,stamp_ns):
    previous=stats["last_header_ns"]
    if previous is not None:
        stats["header_duplicates"]+=stamp_ns==previous
        stats["header_regressions"]+=stamp_ns<previous
    if stats["messages"]==0:
        stats["first_record_ns"]=record_ns;stats["first_header_ns"]=stamp_ns
    stats["last_record_ns"]=record_ns;stats["last_header_ns"]=stamp_ns
    difference=record_ns-stamp_ns
    oldmin=stats["record_minus_header_min_ns"];oldmax=stats["record_minus_header_max_ns"]
    stats["record_minus_header_min_ns"]=difference if oldmin is None else min(oldmin,difference)
    stats["record_minus_header_max_ns"]=difference if oldmax is None else max(oldmax,difference)
    stats["messages"]+=1

def convert(args):
    source=args.source.resolve()
    destination=args.output.resolve()
    if not (source/"metadata.yaml").is_file():
        raise RuntimeError("Source must be the ROS2 bag directory containing metadata.yaml and db3.")
    if source==destination or source in destination.parents:
        raise RuntimeError("Use a separate output directory; the original dataset must remain untouched.")
    if len(set(args.agents))!=len(args.agents):
        raise RuntimeError("Repeated agent names are not allowed.")
    paths={agent:destination/(agent.lower()+"_"+args.image_mode+".bag") for agent in args.agents}
    partial={a:p.with_suffix(p.suffix+".partial") for a,p in paths.items()}
    report_path=destination/"conversion.json"
    for p in list(paths.values())+list(partial.values())+[report_path]:
        if p.exists():raise RuntimeError(f"Refusing to overwrite {p}; choose a new output directory.")
    destination.mkdir(parents=True,exist_ok=True)
    ros2=get_typestore(Stores.ROS2_HUMBLE)
    report={"status":"running","source":str(source),"output":str(destination),
            "agents":args.agents,"image_mode":args.image_mode,"compression":args.compression,
            "rosbags_version":importlib.metadata.version("rosbags"),"opencv_version":cv2.__version__,
            "writer":"ROS1 Noetic native rosbag",
            "source_metadata_sha256":None,"start_offset_s":args.start_offset,
            "duration_s":args.duration,"timestamps_rebased":False,"topics":{},
            "output_bags":{a:str(p) for a,p in paths.items()}}
    import hashlib
    report["source_metadata_sha256"]=hashlib.sha256((source/"metadata.yaml").read_bytes()).hexdigest()
    report_path.write_text(json.dumps(report,indent=2)+"\n")
    started=time.monotonic()
    try:
        with AnyReader([source],default_typestore=ros2) as reader,ExitStack() as stack:
            connections={}
            lookup={}
            for agent in args.agents:
                image_topic=f"/{agent}/left_camera/compressed"
                imu_topic=f"/{agent}/imu/data"
                for topic,kind in [(image_topic,"image"),(imu_topic,"imu")]:
                    matches=[c for c in reader.connections if c.topic==topic]
                    if len(matches)!=1:
                        raise RuntimeError(f"Expected exactly one connection for {topic}, found {len(matches)}")
                    conn=matches[0]
                    expected="sensor_msgs/msg/CompressedImage" if kind=="image" else "sensor_msgs/msg/Imu"
                    if conn.msgtype!=expected:raise RuntimeError(f"Unexpected type for {topic}: {conn.msgtype}")
                    connections[topic]=conn;lookup[conn.id]=(agent,kind)
                    outtopic=topic if kind=="imu" or args.image_mode=="compressed" else f"/{agent}/left_camera/image_raw"
                    report["topics"][topic]=new_stats(topic,outtopic)
            # 源时间与 Header 时间均以整数纳秒保留，不为每台设备单独归零。 (Ref: 2026-10-24_Convert_S3E_ROS1.md)
            begin=reader.start_time+int(round(args.start_offset*1_000_000_000))
            end=None if args.duration is None else begin+int(round(args.duration*1_000_000_000))
            report["selected_record_start_ns"]=begin;report["selected_record_stop_ns"]=end
            writers={};out_connections={}
            for agent in args.agents:
                writers[agent]=stack.enter_context(rosbag.Bag(str(partial[agent]),"w",compression=args.compression))
                for topic,conn in connections.items():
                    if lookup[conn.id][0]==agent:
                        out_connections[conn.id]=report["topics"][topic]["output_topic"]
            processed=0;last_progress=started
            for conn,record_ns,raw in reader.messages(connections=list(connections.values()),start=begin,stop=end):
                agent,kind=lookup[conn.id]
                message=reader.deserialize(raw,conn.msgtype)
                stamp_ns=header_ns(message.header)
                stats=report["topics"][conn.topic]
                if kind=="image" and args.image_mode=="mono8":
                    # 只解码并灰度化，不缩放、不改像素坐标；这是可见的输入转换。 (Ref: 2026-10-24_Convert_S3E_ROS1.md)
                    gray=cv2.imdecode(np.asarray(message.data,dtype=np.uint8),cv2.IMREAD_GRAYSCALE)
                    if gray is None or gray.dtype!=np.uint8 or gray.ndim!=2:
                        raise RuntimeError(f"Image decode failed on {conn.topic} at {stamp_ns}")
                    gray=np.ascontiguousarray(gray)
                    output=RosImage()
                    output.header=RosHeader(seq=0,stamp=RosTime(message.header.stamp.sec,message.header.stamp.nanosec),frame_id=message.header.frame_id)
                    output.height=gray.shape[0];output.width=gray.shape[1]
                    output.encoding="mono8";output.is_bigendian=0;output.step=gray.shape[1]
                    output.data=gray.tobytes()
                    dimensions=[gray.shape[1],gray.shape[0]]
                    if dimensions not in stats["image_dimensions"]:stats["image_dimensions"].append(dimensions)
                else:
                    output=RosImu() if kind=="imu" else RosCompressedImage()
                    output.deserialize(bytes(reader.typestore.cdr_to_ros1(raw,conn.msgtype)))
                writers[agent].write(out_connections[conn.id],output,
                    t=RosTime(record_ns//1_000_000_000,record_ns%1_000_000_000))
                update_stats(stats,int(record_ns),stamp_ns)
                processed+=1
                now=time.monotonic()
                if now-last_progress>=5:
                    report["processed_messages"]=processed;report["elapsed_s"]=now-started
                    report_path.write_text(json.dumps(report,indent=2)+"\n")
                    print(f"converted={processed} elapsed_s={now-started:.1f}",flush=True)
                    last_progress=now
            if args.start_offset==0 and args.duration is None:
                for topic,conn in connections.items():
                    if report["topics"][topic]["messages"]!=conn.msgcount:
                        raise RuntimeError(f"Count mismatch on {topic}")
            if any(s["messages"]==0 for s in report["topics"].values()):
                raise RuntimeError("At least one selected topic has no messages in this window.")
        validation={}
        for agent in args.agents:
            topic_counts={s["output_topic"]:s["messages"] for topic,s in report["topics"].items() if topic.startswith(f"/{agent}/")}
            actual={topic:0 for topic in topic_counts}
            with rosbag.Bag(str(partial[agent])) as check:
                for topic,msg,stamp in check.read_messages():
                    if topic not in actual:raise RuntimeError("Unexpected output topic")
                    actual[topic]+=1
                    if topic.endswith("image_raw") and (msg.encoding!="mono8" or len(msg.data)!=msg.step*msg.height):
                        raise RuntimeError("Invalid output image")
            if actual!=topic_counts:raise RuntimeError(f"Native verification count mismatch for {agent}")
            validation[agent]={"native_ros1_full_read":True,"topic_counts":actual}
        report["validation"]=validation
        # 关闭并写好索引后才改为最终 bag 文件名，失败文件保留 partial 标记。 (Ref: 2026-10-24_Convert_S3E_ROS1.md)
        for agent in args.agents:partial[agent].rename(paths[agent])
        report.update(status="complete",elapsed_s=time.monotonic()-started,
                      processed_messages=processed,
                      output_bytes={a:p.stat().st_size for a,p in paths.items()})
    except BaseException as exc:
        report.update(status="failed",error=repr(exc),elapsed_s=time.monotonic()-started)
        raise
    finally:
        report_path.write_text(json.dumps(report,indent=2)+"\n")
    print(json.dumps(report,indent=2))

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--agents",nargs="+",choices=["Alpha","Bob","Carol"],default=["Alpha","Carol"])
    parser.add_argument("--image-mode",choices=["mono8","compressed"],default="mono8")
    parser.add_argument("--compression",choices=["none","lz4","bz2"],default="lz4")
    parser.add_argument("--start-offset",type=float,default=0.0,help="Seconds from dataset record start; timestamps are not rebased.")
    parser.add_argument("--duration",type=float,help="Optional short validation window.")
    args=parser.parse_args()
    import math
    if not math.isfinite(args.start_offset) or args.start_offset<0 or (
        args.duration is not None and (not math.isfinite(args.duration) or args.duration<=0)):
        parser.error("Offsets must be finite/nonnegative and duration positive.")
    convert(args)
if __name__=="__main__":
    main()
