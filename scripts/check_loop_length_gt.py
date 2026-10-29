#!/usr/bin/python3
"""Position-reference separation check; does not evaluate SE(3) accuracy or label loops."""
import argparse
import bisect
from decimal import Decimal
import hashlib
import json
from pathlib import Path

import numpy as np


def load(path):
    rows=[]
    for line in path.read_text().splitlines():
        if not line.strip() or line.startswith('#'):continue
        fields=line.split()
        rows.append((int(Decimal(fields[0])*Decimal(1000000000)),np.array(list(map(float,fields[1:4])))))
    rows.sort(key=lambda r:r[0])
    if any(a[0]>=b[0] for a,b in zip(rows,rows[1:])):
        raise ValueError('nonunique_gt_timestamps')
    return rows


def interpolate(rows,stamp,max_gap):
    i=bisect.bisect_left([r[0] for r in rows],stamp)
    if i<len(rows) and rows[i][0]==stamp:return rows[i][1],0
    if i==0 or i==len(rows):return None,None
    a,b=rows[i-1],rows[i];gap=(b[0]-a[0])/1e9
    if gap>max_gap:return None,gap
    return a[1]+(stamp-a[0])/(b[0]-a[0])*(b[1]-a[1]),gap


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--summary',type=Path,required=True)
    parser.add_argument('--gt-a',type=Path,required=True)
    parser.add_argument('--gt-b',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--max-gap-s',type=float,default=1.5)
    args=parser.parse_args()
    gt={0:load(args.gt_a),1:load(args.gt_b)}
    summary=json.loads(args.summary.read_text())
    checked=[];skipped=0
    for event in summary['accepted']:
        endpoints=[];gaps=[]
        for label in ['local_key','remote_key']:
            agent,session,stamp=event[label].split(':',2)
            point,gap=interpolate(gt[int(agent)],int(stamp),args.max_gap_s)
            endpoints.append(point);gaps.append(gap)
        if any(p is None for p in endpoints):skipped+=1;continue
        reference=float(np.linalg.norm(endpoints[1]-endpoints[0]))
        estimate=float(np.linalg.norm(event['translation']))
        checked.append(dict(local_key=event['local_key'],remote_key=event['remote_key'],
            estimate_norm_m=estimate,reference_point_separation_m=reference,
            abs_length_difference_m=abs(estimate-reference),gt_interpolation_gaps_s=gaps))
    out=dict(scope='accepted geometry only; position-reference length sanity check; not full translation error or loop precision',
        evaluated=len(checked),skipped=skipped,max_interpolation_gap_s=args.max_gap_s,
        gt_reference_to_body_verified=False,attitude_evaluated=False,
        gt_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [args.gt_a,args.gt_b]},
        abs_length_difference_m_quantiles=np.percentile([r['abs_length_difference_m'] for r in checked],
            [0,50,90,95,100]).tolist() if checked else [],pairs=checked)
    args.output.write_text(json.dumps(out,indent=2)+'\n')
    print(json.dumps({k:v for k,v in out.items() if k!='pairs'},indent=2))


if __name__=='__main__':main()
