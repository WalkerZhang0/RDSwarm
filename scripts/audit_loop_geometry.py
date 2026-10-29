#!/usr/bin/python3
"""Audit recorded accepted loops: depth, coverage, PnP stability and GT length checks.

All interventions use the saved correspondences. No thresholds are changed in
the online pipeline; length checks are not full pose ground truth.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.stats import spearmanr
from rdswarm_msgs.msg import Keyframe
from check_loop_length_gt import load, interpolate


def transform(pose, is_pose=False):
    q = pose.orientation if is_pose else pose.rotation
    p = pose.position if is_pose else pose.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    out = np.eye(4)
    out[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                   [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                   [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    out[:3, 3] = [p.x, p.y, p.z]
    return out


def quantiles(values):
    return np.percentile(values, [0, 25, 50, 75, 95, 100]).tolist() if len(values) else []


def hull(points, width, height):
    if len(points) < 3:
        return 0.0
    return cv2.contourArea(cv2.convexHull(np.asarray(points, np.float32))) / (width*height)


def pnp(obj, xy, a, b, seed=17, iterations=200):
    if len(obj) < 6:
        return dict(solved=False, points=len(obj))
    if min(a.image.width,a.image.height,b.image.width,b.image.height)<=0:
        raise ValueError('image_metadata_required_for_audit')
    cv2.setRNGSeed(seed)
    ok, r, t, inliers = cv2.solvePnPRansac(np.asarray(obj, np.float32), np.asarray(xy, np.float32),
        np.eye(3), None, iterationsCount=iterations, reprojectionError=.004,
        confidence=.999, flags=cv2.SOLVEPNP_EPNP)
    if not ok or inliers is None or len(inliers) < 6:
        return dict(solved=False, points=len(obj), inliers=0 if inliers is None else len(inliers))
    idx = inliers.reshape(-1)
    cv2.solvePnPRefineLM(np.asarray(obj, np.float32)[idx], np.asarray(xy, np.float32)[idx],
                       np.eye(3), None, r, t)
    rotation, _ = cv2.Rodrigues(r)
    camera = np.eye(4); camera[:3,:3] = rotation; camera[:3,3] = t.reshape(3)
    body = transform(a.body_T_camera) @ np.linalg.inv(camera) @ np.linalg.inv(transform(b.body_T_camera))
    return dict(solved=True, points=len(obj), inliers=len(idx), inlier_ratio=len(idx)/len(obj),
                translation=body[:3,3].tolist(), translation_norm_m=float(np.linalg.norm(body[:3,3])))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, default=Path('results/global_loop_recorded_40s'))
    parser.add_argument('--gt-a', type=Path, default=Path('/media/walker/My PSSD/S3E/alpha_gt.txt'))
    parser.add_argument('--gt-b', type=Path, default=Path('/media/walker/My PSSD/S3E/carol_gt.txt'))
    parser.add_argument('--output', type=Path, default=Path('results/loop_geometry_audit'))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    gt = {0: load(args.gt_a), 1: load(args.gt_b)}
    summary = json.loads((args.run/'summary.json').read_text())
    lengths = json.loads((args.run/'gt_length.json').read_text())
    gt_pairs = {(x['local_key'], x['remote_key']): x for x in lengths['pairs']}
    frames = {}
    for path in (args.run/'described').glob('*.bin'):
        if path.name.endswith(('.local.bin', '.peer.bin')):
            continue
        frame = Keyframe().deserialize(path.read_bytes())
        key = '{}:{}:{}'.format(frame.key.agent_id, frame.key.session_id, frame.key.frame_id)
        frames[key] = frame
    records = []
    for event in summary['accepted']:
        a, b = frames[event['local_key']], frames[event['remote_key']]
        result = json.loads(Path(event['result_path']).read_text())
        amap = {f.feature_id:f for f in a.features}; bmap = {f.feature_id:f for f in b.features}
        pairs = [(amap[int(m['local_feature_id'])], bmap[int(m['remote_feature_id'])])
                 for m in result['correspondences']]
        mask = np.asarray([bool(m['inlier']) for m in result['correspondences']])
        obj = np.asarray([[x.position_camera.x,x.position_camera.y,x.position_camera.z] for x,y in pairs])
        ax = np.asarray([[x.normalized.x,x.normalized.y] for x,y in pairs])
        bx = np.asarray([[y.normalized.x,y.normalized.y] for x,y in pairs])
        apx = np.asarray([[x.pixel.x,x.pixel.y] for x,y in pairs])
        bpx = np.asarray([[y.pixel.x,y.pixel.y] for x,y in pairs])
        self_error = np.linalg.norm(obj[:,:2]/obj[:,2,None]-ax,axis=1)
        centered = obj[mask]-obj[mask].mean(axis=0)
        singular = np.linalg.svd(centered,compute_uv=False)
        reference = gt_pairs[(event['local_key'],event['remote_key'])]
        focal = 1172.5678695614395 if a.key.agent_id==0 else 1176.351412228691
        cv2.setRNGSeed(17)
        fundamental, fmask = cv2.findFundamentalMat(ax,bx,cv2.FM_RANSAC,.004,.999,5000)
        repro = pnp(obj,bx,a,b)
        if not repro['solved'] or abs(repro['translation_norm_m']-reference['estimate_norm_m'])>1e-4:
            raise AssertionError('baseline_pnp_not_reproduced:'+event['result_path'])
        t_gt = reference['reference_point_separation_m']
        rec = dict(local_key=event['local_key'],remote_key=event['remote_key'],
            pair_file=event['result_path'],agent=a.key.agent_id,rank=event['rank'],
            similarity=event['similarity'],matches=len(pairs),inliers=int(mask.sum()),
            inlier_ratio=float(mask.mean()),reference_length_m=t_gt,
            estimated_length_m=reference['estimate_norm_m'],
            abs_length_difference_m=reference['abs_length_difference_m'],
            signed_length_difference_m=reference['estimate_norm_m']-t_gt,
            remote_minus_local_s=(b.header.stamp-a.header.stamp).to_sec(),
            normalized_rms=result['rms_normalized'],
            inlier_depth_m_quantiles=quantiles(obj[mask,2]),
            depth_median_m=float(np.median(obj[mask,2])),
            depth_gt20_fraction=float(np.mean(obj[mask,2]>20)),
            self_reprojection_median_px=float(np.median(self_error[mask])*focal),
            self_reprojection_p95_px=float(np.percentile(self_error[mask],95)*focal),
            hull_a=hull(apx[mask],a.image.width,a.image.height),
            hull_b=hull(bpx[mask],b.image.width,b.image.height),
            sv2_over_sv1=float(singular[1]/singular[0]),
            sv3_over_sv1=float(singular[2]/singular[0]),
            f_inliers=int(fmask.sum()) if fmask is not None else 0,
            both_3d_inliers=sum(x.position_valid and y.position_valid for (x,y),m in zip(pairs,mask) if m))
        variants = {}
        for name,keep in [('self_lt1px',self_error*focal<1),('self_lt2px',self_error*focal<2),
                         ('self_lt4px',self_error*focal<4),('depth_lt20m',obj[:,2]<20),
                         ('depth_lt40m',obj[:,2]<40)]:
            fit=pnp(obj[keep],bx[keep],a,b)
            if fit['solved']:fit['abs_length_difference_m']=abs(fit['translation_norm_m']-t_gt)
            variants[name]=fit
        ray_corrected=obj.copy();ray_corrected[:,:2]=ax*obj[:,2,None]
        fit=pnp(ray_corrected,bx,a,b)
        if fit['solved']:fit['abs_length_difference_m']=abs(fit['translation_norm_m']-t_gt)
        variants['local_ray_corrected_diagnostic']=fit
        # 固定内点做删点敏感性检查，不能把它解释为已校准的位姿协方差。 (Ref: 2026-10-28_Audit_Loop_Geometry.md)
        rng=np.random.RandomState(23)
        baseline_body=np.eye(4)
        q=result['quaternion_xyzw']
        x,y,z,w=q
        Rbody=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                       [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                       [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
        baseline_body[:3,:3]=Rbody;baseline_body[:3,3]=result['translation']
        camera=np.linalg.inv(transform(b.body_T_camera))@np.linalg.inv(baseline_body)@transform(a.body_T_camera)
        base_r=cv2.Rodrigues(camera[:3,:3])[0];base_t=camera[:3,3,None]
        bootstrap=[];rotations=[]
        for _ in range(40):
            choose=rng.choice(np.flatnonzero(mask),max(6,int(mask.sum()*.8)),replace=False)
            r,t=base_r.copy(),base_t.copy()
            cv2.solvePnPRefineLM(obj[choose].astype(np.float32),bx[choose].astype(np.float32),np.eye(3),None,r,t)
            c=np.eye(4);c[:3,:3]=cv2.Rodrigues(r)[0];c[:3,3]=t.reshape(3)
            pose=transform(a.body_T_camera)@np.linalg.inv(c)@np.linalg.inv(transform(b.body_T_camera))
            bootstrap.append(float(np.linalg.norm(pose[:3,3]-baseline_body[:3,3])))
            rotations.append(float(np.degrees(np.linalg.norm(cv2.Rodrigues(pose[:3,:3]@Rbody.T)[0]))))
        rec['drop20_translation_p95_m']=float(np.percentile(bootstrap,95))
        rec['drop20_rotation_p95_deg']=float(np.percentile(rotations,95))
        _,jac=cv2.projectPoints(obj[mask].astype(np.float64),base_r,base_t,np.eye(3),None)
        j=jac[:,:6]
        conditional_cov=np.linalg.pinv(j.T@j)*(.5*(1/1172.57**2+1/1176.35**2))
        rec['conditional_translation_std_1px_m']=float(np.sqrt(np.trace(conditional_cov[3:,3:])))
        rec['map_transform']= (transform(a.body_pose_vio,True)@baseline_body@np.linalg.inv(transform(b.body_pose_vio,True))).tolist()
        common=[(x,y) for (x,y),m in zip(pairs,mask) if m and y.position_valid]
        if len(common)>=6:
            oa=np.asarray([[x.position_camera.x,x.position_camera.y,x.position_camera.z] for x,y in common])
            ob=np.asarray([[y.position_camera.x,y.position_camera.y,y.position_camera.z] for x,y in common])
            xa=np.asarray([[x.normalized.x,x.normalized.y] for x,y in common])
            reverse=pnp(ob,xa,b,a)
            if reverse['solved']:reverse['abs_length_difference_m']=abs(reverse['translation_norm_m']-t_gt)
            rec['reverse_common3d']=reverse
            ma,mb=oa.mean(axis=0),ob.mean(axis=0)
            u,s,vh=np.linalg.svd((oa-ma).T@(ob-mb)/len(oa))
            sign=np.ones(3);sign[-1]=np.linalg.det(u@vh)
            rot=u@np.diag(sign)@vh
            scale=float(np.sum(s*sign)/np.mean(np.sum((ob-mb)**2,axis=1)))
            transformed=scale*((rot@(ob-mb).T).T)+ma
            rec['common_map_similarity']=dict(points=len(common),scale_B_to_A=scale,
                rms_3d_m=float(np.sqrt(np.mean(np.sum((transformed-oa)**2,axis=1)))))
        seeds=[pnp(obj,bx,a,b,seed=seed) for seed in range(10)]
        norms=[x['translation_norm_m'] for x in seeds if x['solved']]
        rec['seed_norm_std_m']=float(np.std(norms)) if norms else None
        rec['seed_norm_range_m']=float(np.ptp(norms)) if norms else None
        rec['seed_inlier_counts']=[x.get('inliers',0) for x in seeds]
        rec['variants']=variants
        records.append(rec)
    features=['inliers','inlier_ratio','depth_median_m','depth_gt20_fraction',
              'self_reprojection_median_px','hull_a','hull_b','sv2_over_sv1','sv3_over_sv1',
              'normalized_rms','similarity','reference_length_m','seed_norm_std_m',
              'drop20_translation_p95_m','drop20_rotation_p95_deg','conditional_translation_std_1px_m']
    def stats(rows):
        correlations={}
        for f in features:
            x=[r[f] for r in rows];y=[r['abs_length_difference_m'] for r in rows]
            rho,p=spearmanr(x,y)
            correlations[f]=dict(spearman=float(rho) if np.isfinite(rho) else None,
                                 p_descriptive_only=float(p) if np.isfinite(p) else None)
        return dict(count=len(rows),signed_length_quantiles_m=quantiles([r['signed_length_difference_m'] for r in rows]),
            metrics={f:quantiles([r[f] for r in rows]) for f in features},correlations=correlations)
    interventions={}
    for name in records[0]['variants']:
        usable=[r for r in records if r['variants'][name]['solved']]
        interventions[name]=dict(solved=len(usable),
            baseline_abs_difference_m=quantiles([r['abs_length_difference_m'] for r in usable]),
            variant_abs_difference_m=quantiles([r['variants'][name]['abs_length_difference_m'] for r in usable]),
            improved=sum(r['variants'][name]['abs_length_difference_m']<r['abs_length_difference_m'] for r in usable))
    vio=[]
    for agent in [0,1]:
        ordered=sorted((f for f in frames.values() if f.key.agent_id==agent),key=lambda f:f.header.stamp.to_nsec())
        for a in ordered:
            target=min(ordered,key=lambda b:abs((b.header.stamp-a.header.stamp).to_sec()-5))
            dt=(target.header.stamp-a.header.stamp).to_sec()
            if not 4.5<=dt<=5.5:continue
            pa,_=interpolate(gt[agent],a.header.stamp.to_nsec(),1.5)
            pb,_=interpolate(gt[agent],target.header.stamp.to_nsec(),1.5)
            if pa is None or pb is None:continue
            actual=np.linalg.norm(pb-pa)
            if actual<1:continue
            estimated=np.linalg.norm(transform(target.body_pose_vio,True)[:3,3]-transform(a.body_pose_vio,True)[:3,3])
            vio.append(dict(agent=agent,start_ns=a.header.stamp.to_nsec(),duration_s=dt,
                vio_displacement_m=float(estimated),gt_displacement_m=float(actual),ratio=float(estimated/actual)))
    out=dict(scope='accepted pairs, correlated temporal samples; associations not independent accuracy labels; GT reference lever arm unknown',
        count=len(records),overall=stats(records),by_agent={str(a):stats([r for r in records if r['agent']==a]) for a in [0,1]},
        interventions=interventions,vio_5s_displacement_ratios={str(a):quantiles([r['ratio'] for r in vio if r['agent']==a]) for a in [0,1]},
        vio_displacement_checks=vio,pairs=records)
    (args.output/'audit.json').write_text(json.dumps(out,indent=2,allow_nan=False)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes=plt.subplots(2,3,figsize=(12,7),constrained_layout=True)
    configs=[('inliers','PnP inliers'),('depth_median_m','Median inlier depth (m)'),
             ('hull_a','Local inlier hull / image area'),
             ('self_reprojection_median_px','Local self-reprojection (approx. px)'),
             ('reference_length_m','GT reference-point separation (m)'),
             ('drop20_translation_p95_m','Drop-20% sensitivity, P95 (m)')]
    for ax,(metric,label) in zip(axes.flat,configs):
        for agent,color,marker in [(0,'#0072B2','o'),(1,'#D55E00','^')]:
            rows=[r for r in records if r['agent']==agent]
            ax.scatter([r[metric] for r in rows],[r['abs_length_difference_m'] for r in rows],
                c=color,marker=marker,s=24,alpha=.75,label='Alpha 3D' if agent==0 else 'Carol 3D')
        ax.set_xlabel(label);ax.set_ylabel('Absolute length difference (m)');ax.grid(alpha=.15)
    axes[0,0].legend(fontsize=9)
    fig.savefig(args.output/'geometry_vs_length_difference.png',dpi=180)
    fig.savefig(args.output/'geometry_vs_length_difference.svg')
    plt.close(fig)
    print(json.dumps({k:v for k,v in out.items() if k not in ['pairs','vio_displacement_checks','by_agent']},indent=2))


if __name__=='__main__':main()
