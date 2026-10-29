#!/usr/bin/python3
"""Compare both depth sources on identical saved correspondences, without rematching."""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
from rdswarm_msgs.msg import Keyframe
from audit_loop_geometry import transform
from check_loop_length_gt import load, interpolate


def solve(pairs, source, target, reverse=False, fixed=False):
    oriented = [(b,a) for a,b in pairs] if reverse else pairs
    obj = np.asarray([[a.position_camera.x,a.position_camera.y,a.position_camera.z]
                      for a,b in oriented], np.float32)
    pixels = np.asarray([[b.normalized.x,b.normalized.y] for a,b in oriented], np.float32)
    if len(obj) < 6:
        return dict(solved=False, reason='fewer_than_six_common_points')
    cv2.setRNGSeed(17)
    if fixed:
        ok,r,t = cv2.solvePnP(obj,pixels,np.eye(3),None,flags=cv2.SOLVEPNP_EPNP)
        idx = np.arange(len(obj))
    else:
        ok,r,t,inliers = cv2.solvePnPRansac(obj,pixels,np.eye(3),None,
            iterationsCount=200,reprojectionError=.004,confidence=.999,flags=cv2.SOLVEPNP_EPNP)
        idx = inliers.reshape(-1) if inliers is not None else np.array([],int)
    if not ok or len(idx) < 6:
        return dict(solved=False, input_points=len(obj), ransac_inliers=len(idx))
    cv2.solvePnPRefineLM(obj[idx],pixels[idx],np.eye(3),None,r,t)
    singular = np.linalg.svd(obj[idx].astype(float)-obj[idx].mean(axis=0),compute_uv=False)
    spread_ok = singular[0]>1e-6 and singular[1]/singular[0]>=1e-3
    camera = np.eye(4)
    camera[:3,:3] = cv2.Rodrigues(r)[0]
    camera[:3,3] = t.reshape(3)
    # 将反向结果求逆后统一为 T_BCarol_BAlpha，即把 Alpha 机体系坐标映射到 Carol 机体系。 (Ref: 2026-10-28_Compare_Bidirectional_PnP.md)
    body = transform(source.body_T_camera) @ np.linalg.inv(camera) @ np.linalg.inv(transform(target.body_T_camera))
    common_body = np.linalg.inv(body) if reverse else body
    projected = (camera[:3,:3] @ obj.astype(float).T).T + camera[:3,3]
    errors = np.linalg.norm(projected[:,:2]/projected[:,2,None]-pixels,axis=1)
    return dict(solved=True, depth_source=int(source.key.agent_id), input_points=len(obj),
        ransac_inliers=len(idx), inlier_indices=idx.tolist(), inlier_ratio=len(idx)/len(obj),
        depth_m_quantiles=np.percentile(obj[:,2],[0,50,100]).tolist(),
        T_BCarol_BAlpha=common_body.tolist(), translation=common_body[:3,3].tolist(),
        translation_norm_m=float(np.linalg.norm(common_body[:3,3])),
        inlier_rms_normalized=float(np.sqrt(np.mean(errors[idx]**2))),
        all_point_median_error_normalized=float(np.median(errors)),
        positive_inlier_fraction=float(np.mean(projected[idx,2]>1e-6)),
        inlier_spread_ok=bool(spread_ok), fixed_without_ransac=fixed,
        passes_count_ratio_reprojection_cheirality=bool(len(idx)>=8 and len(idx)/len(obj)>=.5 and
            np.sqrt(np.mean(errors[idx]**2))<=.004 and np.mean(projected[idx,2]>1e-6)>=.9)
            if not fixed else None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,default=Path('results/global_loop_recorded_40s'))
    parser.add_argument('--pair',type=Path,default=Path('results/global_loop_recorded_40s/pair_00152.json'))
    parser.add_argument('--output',type=Path,default=Path('results/bidirectional_pair_00152'))
    parser.add_argument('--gt-a',type=Path,default=Path('/media/walker/My PSSD/S3E/alpha_gt.txt'))
    parser.add_argument('--gt-b',type=Path,default=Path('/media/walker/My PSSD/S3E/carol_gt.txt'))
    args = parser.parse_args()
    summary = json.loads((args.run/'summary.json').read_text())
    event = next(e for e in summary['accepted'] if Path(e['result_path']).name==args.pair.name)
    baseline = json.loads(args.pair.read_text())
    frames = {};paths = {}
    for path in (args.run/'described').glob('*.bin'):
        if path.name.endswith(('.local.bin','.peer.bin')):continue
        frame = Keyframe().deserialize(path.read_bytes())
        key = '{}:{}:{}'.format(frame.key.agent_id,frame.key.session_id,frame.key.frame_id)
        if key in [event['local_key'],event['remote_key']]:frames[key]=frame;paths[key]=path
    carol,alpha = frames[event['local_key']],frames[event['remote_key']]
    if carol.key.agent_id!=1 or alpha.key.agent_id!=0:
        raise ValueError('this_diagnostic_expects_Carol_to_Alpha_baseline')
    cm={f.feature_id:f for f in carol.features};am={f.feature_id:f for f in alpha.features}
    matches=baseline['correspondences']
    pairs=[(cm[int(m['local_feature_id'])],am[int(m['remote_feature_id'])]) for m in matches]
    common=[i for i,(c,a) in enumerate(pairs) if c.position_valid and a.position_valid]
    original_inliers=[i for i in common if matches[i]['inlier']]
    pa,ga=interpolate(load(args.gt_a),alpha.header.stamp.to_nsec(),1.5)
    pc,gc=interpolate(load(args.gt_b),carol.header.stamp.to_nsec(),1.5)
    if pa is None or pc is None:raise ValueError('gt_interpolation_unavailable')
    reference=float(np.linalg.norm(pa-pc))
    experiments={}
    experiments['original_Carol_3d_all20']=solve(pairs,carol,alpha)
    if abs(experiments['original_Carol_3d_all20']['translation_norm_m']-np.linalg.norm(baseline['translation']))>1e-5:
        raise AssertionError('original_pnp_not_reproduced')
    for group,indices in [('all_common3d',common),('original_inliers_common3d',original_inliers)]:
        selected=[pairs[i] for i in indices]
        forward=solve(selected,carol,alpha)
        reverse=solve(selected,alpha,carol,reverse=True)
        experiments[group+'_Carol_3d']=forward
        experiments[group+'_Alpha_3d']=reverse
        if forward['solved'] and reverse['solved']:
            joint=sorted(set(forward['inlier_indices']) & set(reverse['inlier_indices']))
            fixed=[selected[i] for i in joint]
            experiments[group+'_joint_inliers_fixed_Carol_3d']=solve(fixed,carol,alpha,fixed=True)
            experiments[group+'_joint_inliers_fixed_Alpha_3d']=solve(fixed,alpha,carol,reverse=True,fixed=True)
    for result in experiments.values():
        if result['solved']:result['abs_length_difference_m']=abs(result['translation_norm_m']-reference)
    out=dict(scope='fixed same-pair depth-source comparison; no rematching; partial GT length check',
        local_key=event['local_key'],remote_key=event['remote_key'],
        time_difference_s=(carol.header.stamp-alpha.header.stamp).to_sec(),
        reference_body_position_separation_m=reference,gt_interpolation_gap_s=dict(alpha=ga,carol=gc),
        full_pose_gt_available=False,input_matches=len(pairs),both_depth_valid=len(common),
        original_inliers_both_depth_valid=len(original_inliers),
        solver=dict(method='EPNP RANSAC then inlier LM',seed=17,iterations=200,
                    threshold_normalized=.004,confidence=.999,
                    uniform_output_direction='T_BCarol_BAlpha',rematching=False),
        shared_correspondences=[dict(original_index=i,carol_id=str(pairs[i][0].feature_id),
            alpha_id=str(pairs[i][1].feature_id),was_original_inlier=bool(matches[i]['inlier']),
            carol_depth_m=pairs[i][0].position_camera.z,alpha_depth_m=pairs[i][1].position_camera.z)
            for i in common],
        sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [args.pair,*paths.values()]},
        experiments=experiments)
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'report.json').write_text(json.dumps(out,indent=2,allow_nan=False)+'\n')
    print('common points',len(common),'original inliers common',len(original_inliers),'GT length',reference)
    for name,r in experiments.items():
        print(name,{k:r.get(k) for k in ['solved','input_points','ransac_inliers','translation_norm_m',
                                        'abs_length_difference_m','inlier_rms_normalized',
                                        'passes_count_ratio_reprojection_cheirality']})


if __name__=='__main__':main()
