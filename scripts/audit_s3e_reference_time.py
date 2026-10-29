#!/usr/bin/python3
"""Trace a saved loop to source image headers and GNSS-derived text references."""
import argparse
import bisect
from decimal import Decimal
import hashlib
import json
from pathlib import Path
import sqlite3
import subprocess

import cv2
import numpy as np
from rosbags.typesys import get_typestore, Stores
from rdswarm_msgs.msg import Keyframe


def stamp_ns(message):
    return int(message.header.stamp.sec)*1000000000+int(message.header.stamp.nanosec)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,default=Path('/media/walker/My PSSD/S3E'))
    parser.add_argument('--run',type=Path,default=Path('results/global_loop_recorded_40s'))
    parser.add_argument('--pair-name',default='pair_00152.json')
    parser.add_argument('--output',type=Path,default=Path('results/pair_00152_time_audit'))
    args=parser.parse_args()
    summary=json.loads((args.run/'summary.json').read_text())
    event=next(e for e in summary['accepted'] if Path(e['result_path']).name==args.pair_name)
    desired={event['local_key'],event['remote_key']}
    frames={}
    for path in (args.run/'described').glob('*.bin'):
        if path.name.endswith(('.local.bin','.peer.bin')):continue
        frame=Keyframe().deserialize(path.read_bytes())
        key='{}:{}:{}'.format(frame.key.agent_id,frame.key.session_id,frame.key.frame_id)
        if key in desired:frames[frame.key.agent_id]=frame
    db=sqlite3.connect('file:'+str(args.source/'S3E_Library_1.db3')+'?mode=ro',uri=True)
    types=get_typestore(Stores.ROS2_HUMBLE)
    topics={name:(tid,typ) for tid,name,typ in db.execute('SELECT id,name,type FROM topics')}
    result={}
    for agent,name in [(0,'Alpha'),(1,'Carol')]:
        frame=frames[agent];target=frame.header.stamp.to_nsec()
        path=args.source/(name.lower()+'_gt.txt')
        text=[]
        for line_number,line in enumerate(path.read_text().splitlines(),1):
            if not line.strip() or line.startswith('#'):continue
            fields=line.split()
            text.append(dict(line=line_number,text=line,stamp_ns=int(Decimal(fields[0])*Decimal(1000000000)),
                             position=list(map(float,fields[1:4]))))
        tid,typ=topics['/'+name+'/fix']
        fixes=[]
        for record_ns,data in db.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp',(tid,)):
            m=types.deserialize_cdr(data,typ)
            fixes.append(dict(record_ns=record_ns,header_ns=stamp_ns(m),frame_id=m.header.frame_id,
                latitude=m.latitude,longitude=m.longitude,altitude=m.altitude,
                status=int(m.status.status),covariance_type=int(m.position_covariance_type),
                covariance_enu_m2=m.position_covariance.tolist(),
                reported_std_enu_m=np.sqrt(np.maximum(0,np.asarray(m.position_covariance).reshape(3,3).diagonal())).tolist()))
        valid=[i for i,f in enumerate(fixes) if np.isfinite([f['longitude'],f['latitude'],f['altitude']]).all()]
        coordinates=''.join('{} {} {}\n'.format(fixes[i]['longitude'],fixes[i]['latitude'],fixes[i]['altitude']) for i in valid)
        run=subprocess.run(['cs2cs','+proj=longlat','+datum=WGS84','+to','+proj=utm','+zone=49','+datum=WGS84','-f','%.9f'],
                           input=coordinates,text=True,capture_output=True,check=True)
        utm=np.full((len(fixes),3),np.nan)
        utm[valid]=np.array([list(map(float,line.split())) for line in run.stdout.splitlines()])
        stamps=np.array([f['header_ns'] for f in fixes],dtype=np.int64)
        mapping=[]
        for row in text:
            nearest=int(np.argmin(np.abs(stamps-row['stamp_ns'])))
            mapping.append(dict(gt_line=row['line'],fix_index=nearest,
                text_minus_fix_ns=int(row['stamp_ns']-stamps[nearest]),
                text_minus_utm_m=(np.asarray(row['position'])-utm[nearest]).tolist()))
        i=bisect.bisect_left([r['stamp_ns'] for r in text],target)
        if i==0 or i==len(text):raise ValueError('no_bracket')
        left,right=text[i-1],text[i]
        weight=(target-left['stamp_ns'])/(right['stamp_ns']-left['stamp_ns'])
        position=np.array(left['position'])+weight*(np.array(right['position'])-left['position'])
        endpoints=[]
        for j in [i-1,i]:
            endpoints.append(dict(text_row=text[j],source_fix=fixes[mapping[j]['fix_index']],mapping=mapping[j]))
        image_tid,image_type=topics['/'+name+'/left_camera/compressed']
        images=db.execute('SELECT timestamp,data FROM messages WHERE topic_id=? AND timestamp BETWEEN ? AND ?',
                          (image_tid,target-100000000,target+100000000)).fetchall()
        image_check=None
        for record_ns,data in images:
            image=types.deserialize_cdr(data,image_type)
            if stamp_ns(image)!=target:continue
            decoded=cv2.imdecode(np.frombuffer(image.data,np.uint8),cv2.IMREAD_GRAYSCALE)
            cached=np.frombuffer(frame.image.data,np.uint8).reshape(frame.image.height,frame.image.step)[:,:frame.image.width]
            image_check=dict(source_record_ns=record_ns,source_header_ns=stamp_ns(image),
                keyframe_ns=int(frame.key.frame_id),keyframe_header_ns=target,
                cached_image_header_ns=frame.image.header.stamp.to_nsec(),
                source_and_keyframe_pixels_equal=bool(np.array_equal(decoded,cached)),
                source_decoded_sha256=hashlib.sha256(decoded.tobytes()).hexdigest(),
                keyframe_image_sha256=hashlib.sha256(cached.tobytes()).hexdigest())
        if image_check is None:raise ValueError('original_image_not_found')
        delta=np.array([m['text_minus_utm_m'] for m in mapping])
        altitude_segments=[]
        for row,offset in zip(text,delta[:,2]):
            value=round(float(offset),6)
            if not altitude_segments or abs(altitude_segments[-1]['offset_m']-value)>1e-5:
                altitude_segments.append(dict(offset_m=value,first_text_line=row['line'],
                    last_text_line=row['line'],first_ns=row['stamp_ns'],last_ns=row['stamp_ns'],rows=1))
            else:
                altitude_segments[-1].update(last_text_line=row['line'],last_ns=row['stamp_ns'],
                                             rows=altitude_segments[-1]['rows']+1)
        result[name.lower()]=dict(frame_key='{}:{}:{}'.format(agent,frame.key.session_id,frame.key.frame_id),
            image_time_audit=image_check,gt_text_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            gt_text_rows=len(text),source_fix_rows=len(fixes),
            max_abs_text_fix_time_difference_ns=max(abs(m['text_minus_fix_ns']) for m in mapping),
            max_abs_text_utm_xy_difference_m=np.max(np.abs(delta[:,:2]),axis=0).tolist(),
            text_minus_raw_gnss_altitude_m_quantiles=np.percentile(delta[:,2],[0,50,100]).tolist(),
            text_altitude_offset_segments=altitude_segments,
            all_fixes_frame_ids=sorted(set(f['frame_id'] for f in fixes)),
            interpolation=dict(target_ns=target,left=left,right=right,fraction=weight,
                position=position.tolist(),endpoints=endpoints),
            physical_clock_alignment_verified=False,body_reference_verified=False)
    a=np.array(result['alpha']['interpolation']['position']);c=np.array(result['carol']['interpolation']['position'])
    out=dict(scope='source-message identity and GNSS/text provenance; not physical clock calibration',
        projection='WGS84 -> UTM zone 49N via system PROJ cs2cs; compares source fix to text',
        pair=args.pair_name,reference=result,
        carol_minus_alpha_position_m=(c-a).tolist(),separation_m=float(np.linalg.norm(c-a)),
        loop_body_translation_norm_m=float(np.linalg.norm(event['translation'])),
        note='GNSS/text identity does not establish GNSS exposure-time synchronization or high-precision body truth')
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'report.json').write_text(json.dumps(out,indent=2,allow_nan=False)+'\n')
    print(json.dumps(out,indent=2))


if __name__=='__main__':main()
