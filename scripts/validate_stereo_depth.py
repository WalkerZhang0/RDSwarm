#!/usr/bin/python3
"""Offline stereo/VIO depth and fixed-correspondence PnP comparison (not ground truth)."""
import argparse
import copy
import csv
import hashlib
import json
from pathlib import Path
import sqlite3

import cv2
import numpy as np
import rosbag
from sensor_msgs.msg import CompressedImage
from rosbags.typesys import Stores, get_typestore
from rdswarm_msgs.msg import Keyframe
from scipy.spatial.transform import Rotation
from compare_bidirectional_pnp import solve
from audit_loop_geometry import transform

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def summary(values):
    values = np.asarray(values, float)
    values = values[np.isfinite(values)]
    return dict(count=len(values), p10_median_p90=np.percentile(values, [10, 50, 90]).tolist()) if len(values) else dict(count=0)


def calibration(path):
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    out = {key: fs.getNode(key).mat() for key in ['LEFT.K', 'LEFT.D', 'RIGHT.K', 'RIGHT.D', 'RIGHT.R', 'RIGHT.P', 'Tic']}
    out['baseline_m'] = fs.getNode('Camera.bf').real()/fs.getNode('Camera.fx').real()
    fs.release()
    raw = out['RIGHT.P'][:, 3]
    # 原文件末列疑似毫米平移，不能当标准 Kt；只用方向并以 bf/fx 给定尺度，结果标记为待核验。 (Ref: 2026-10-28_Validate_Stereo_Depth.md)
    out['t'] = raw/np.linalg.norm(raw)*out['baseline_m']
    out['R'] = out['RIGHT.R']
    return out


def extract(db, types, name, stamp, folder, bag):
    images, records = [], []
    for side in ['left', 'right']:
        topic = '/'+name+'/'+side+'_camera/compressed'
        tid, typ = db.execute('SELECT id,type FROM topics WHERE name=?', (topic,)).fetchone()
        candidates = []
        for rec, data in db.execute('SELECT timestamp,data FROM messages WHERE timestamp BETWEEN ? AND ? AND topic_id=?',
                                    (stamp-20000000, stamp+20000000, tid)):
            m = types.deserialize_cdr(data, typ)
            ns = int(m.header.stamp.sec)*10**9+int(m.header.stamp.nanosec)
            candidates.append((abs(ns-stamp), ns, rec, m, data))
        if not candidates:
            raise ValueError('no_stereo_image_within_20ms:'+topic)
        _, ns, rec, m, data = min(candidates, key=lambda item: item[0])
        if abs(ns-stamp) > 2000000:
            raise ValueError('stereo_image_more_than_2ms_from_left:'+topic)
        image = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise ValueError('image_decode_failed')
        filename = folder/(name.lower()+'_'+side+'.png')
        cv2.imwrite(str(filename), image)
        (folder/(name.lower()+'_'+side+'.cdr')).write_bytes(data)
        msg = CompressedImage()
        msg.header.stamp.secs = ns//10**9
        msg.header.stamp.nsecs = ns % 10**9
        msg.header.frame_id = m.header.frame_id
        msg.format = m.format
        msg.data = np.asarray(m.data, np.uint8).tobytes()
        import rospy
        bag.write(topic, msg, t=rospy.Time(rec//10**9, rec % 10**9))
        images.append(image)
        records.append(dict(topic=topic, header_ns=ns, record_ns=rec, offset_from_left_ns=ns-stamp,
                            png=str(filename), cdr_sha256=hashlib.sha256(data).hexdigest(),
                            decoded_sha256=hashlib.sha256(image.tobytes()).hexdigest()))
    return images, records


def triangulate(left, right, cal):
    a = cv2.undistortPoints(np.asarray(left, float).reshape(-1, 1, 2), cal['LEFT.K'], cal['LEFT.D']).reshape(-1, 2)
    b = cv2.undistortPoints(np.asarray(right, float).reshape(-1, 1, 2), cal['RIGHT.K'], cal['RIGHT.D']).reshape(-1, 2)
    h = cv2.triangulatePoints(np.c_[np.eye(3), np.zeros(3)], np.c_[cal['R'], cal['t']], a.T, b.T)
    return (h[:3]/h[3]).T


def ncc(left, right, p, q):
    h, w = left.shape
    if min(p[0], p[1], q[0], q[1], w-1-p[0], h-1-p[1], w-1-q[0], h-1-q[1]) < 12:
        return -1.
    a = cv2.getRectSubPix(left, (15, 15), tuple(map(float, p))).astype(float)
    b = cv2.getRectSubPix(right, (15, 15), tuple(map(float, q))).astype(float)
    a -= a.mean(); b -= b.mean()
    return float(np.sum(a*b)/max(1e-10, np.linalg.norm(a)*np.linalg.norm(b)))


def depths(frame, images, cal):
    # 左目严格沿用已有特征像素，不重新检测左目点；双向与双窗口匹配交叉检查只用于找右目对应。 (Ref: 2026-10-28_Validate_Stereo_Depth.md)
    features = list(frame.features)
    pts = np.float32([[f.pixel.x, f.pixel.y] for f in features]).reshape(-1, 1, 2)
    options = dict(maxLevel=4, criteria=(3, 50, .001))
    q, ok, _ = cv2.calcOpticalFlowPyrLK(*images, pts, None, winSize=(31, 31), **options)
    back, bok, _ = cv2.calcOpticalFlowPyrLK(images[1], images[0], q, None, winSize=(31, 31), **options)
    q2, ok2, _ = cv2.calcOpticalFlowPyrLK(*images, pts, None, winSize=(21, 21), **options)
    lr = np.linalg.norm(back-pts, axis=2)[:, 0]
    window = np.linalg.norm(q2-q, axis=2)[:, 0]
    points = triangulate(pts, q, cal)
    rp = (cal['R']@points.T).T+cal['t']
    lpix = cv2.projectPoints(points, np.zeros(3), np.zeros(3), cal['LEFT.K'], cal['LEFT.D'])[0].reshape(-1, 2)
    rpix = cv2.projectPoints(points, cv2.Rodrigues(cal['R'])[0], cal['t'], cal['RIGHT.K'], cal['RIGHT.D'])[0].reshape(-1, 2)
    reproj = np.maximum(np.linalg.norm(lpix-pts[:, 0], axis=1), np.linalg.norm(rpix-q[:, 0], axis=1))
    shifted = q.copy(); shifted[:, 0, 0] += .5
    shifted2 = q.copy(); shifted2[:, 0, 0] -= .5
    z1 = triangulate(pts, shifted, cal)[:, 2]; z2 = triangulate(pts, shifted2, cal)[:, 2]
    relative = np.maximum(abs(z1-points[:, 2]), abs(z2-points[:, 2]))/np.maximum(abs(points[:, 2]), 1e-9)
    # 同时计算正反 R 及标准投影矩阵解释的极线残差；这些检查不能确定绝对基线单位。 (Ref: 2026-10-28_Validate_Stereo_Depth.md)
    a = cv2.undistortPoints(pts, cal['LEFT.K'], cal['LEFT.D']).reshape(-1, 2)
    b = cv2.undistortPoints(q, cal['RIGHT.K'], cal['RIGHT.D']).reshape(-1, 2)
    ah = np.c_[a, np.ones(len(a))]; bh = np.c_[b, np.ones(len(b))]
    def epipolar(R, t):
        tx = np.array([[0, -t[2], t[1]], [t[2], 0, -t[0]], [-t[1], t[0], 0]])
        line = ah@(tx@R).T
        return np.abs(np.sum(bh*line, axis=1))/np.linalg.norm(line[:, :2], axis=1)*cal['RIGHT.K'][0, 0]
    epis = {label: epipolar(R, t) for label, R, t in [
        ('R_and_P_column_direction', cal['R'], cal['t']),
        ('R_transpose_and_P_column_direction', cal['R'].T, cal['t']),
        ('identity_and_P_column_direction', np.eye(3), cal['t']),
        ('literal_projection_K_inverse_P_column', cal['R'], np.linalg.solve(cal['RIGHT.K'], cal['RIGHT.P'][:, 3]))]}
    rows = []
    for i, f in enumerate(features):
        score = ncc(*images, pts[i, 0], q[i, 0])
        match_ok = bool(ok[i, 0] and bok[i, 0] and ok2[i, 0] and lr[i] <= .7 and window[i] <= 1. and score >= .85)
        geometric_ok = bool(np.isfinite(points[i]).all() and points[i, 2] > 0 and rp[i, 2] > 0 and relative[i] <= .25)
        good = bool(match_ok and geometric_ok and reproj[i] <= 2.)
        z = float(f.position_camera.z) if f.position_valid else None
        row = dict(feature_id=str(f.feature_id), left_pixel=pts[i, 0].tolist(), right_pixel=q[i, 0].tolist(),
                   vio_depth_m=z, stereo_point_left_m=points[i].tolist(), stereo_depth_m=float(points[i, 2]),
                   fb_error_px=float(lr[i]), window_difference_px=float(window[i]), ncc=score,
                   max_reprojection_px=float(reproj[i]), epipolar_error_px=float(epis['R_and_P_column_direction'][i]),
                   relative_depth_change_for_half_pixel=float(relative[i]), match_ok=match_ok,
                   geometry_ok=geometric_ok, accepted=good, strict_accepted=bool(good and reproj[i] <= 1.),
                   relaxed_accepted=bool(match_ok and geometric_ok and reproj[i] <= 3.))
        if z is not None and good:
            row.update(vio_over_stereo=z/points[i, 2], vio_minus_stereo_m=z-points[i, 2])
        rows.append(row)
    matched = np.array([r['match_ok'] for r in rows])
    diagnostics = {key: summary(vals[matched]) for key, vals in epis.items()}
    return rows, diagnostics


def calibration_sensitivity(rows, cal):
    selected = [r for r in rows if r['accepted'] and r['vio_depth_m'] is not None]
    if not selected:
        return {}
    # 固定图像对应，只扰动假定双目旋转；此处不是重标定，也不把扰动结果选作最优真值。 (Ref: 2026-10-28_Validate_Stereo_Depth.md)
    result = {}
    for axis in range(3):
        for sign in [-1, 1]:
            delta = np.zeros(3); delta[axis] = np.radians(sign*.1)
            perturbed = dict(cal, R=cv2.Rodrigues(delta)[0]@cal['R'])
            xyz = triangulate([r['left_pixel'] for r in selected], [r['right_pixel'] for r in selected], perturbed)
            good = np.isfinite(xyz).all(axis=1) & (xyz[:, 2] > 0)
            result['{}_{}0.1deg'.format('xyz'[axis], '+' if sign > 0 else '-')] = dict(
                positive_points=int(good.sum()), fixed_input_points=len(selected),
                vio_over_stereo=summary(np.asarray([r['vio_depth_m'] for r in selected])[good]/xyz[good, 2]),
                stereo_depth_m=summary(xyz[good, 2]))
    return result


def pose_change(result, baseline):
    if not result.get('solved'):
        return None
    a = np.asarray(result['T_BCarol_BAlpha']); b = np.asarray(baseline['T_BCarol_BAlpha'])
    return dict(translation_vector_difference_m=float(np.linalg.norm(a[:3, 3]-b[:3, 3])),
                translation_length_difference_m=float(np.linalg.norm(a[:3, 3])-np.linalg.norm(b[:3, 3])),
                rotation_difference_deg=float(np.degrees(Rotation.from_matrix(a[:3, :3]@b[:3, :3].T).magnitude())))


def draw(name, images, rows, loop_ids, folder):
    out = cv2.cvtColor(np.hstack(images), cv2.COLOR_GRAY2BGR); w = images[0].shape[1]
    for row in rows:
        if row['feature_id'] not in loop_ids:
            continue
        color = (0, 220, 0) if row['accepted'] else (0, 100, 255)
        p = tuple(np.round(row['left_pixel']).astype(int)); q = tuple(np.round(row['right_pixel']).astype(int)+[w, 0])
        cv2.circle(out, p, 5, color, 2); cv2.circle(out, q, 5, color, 2); cv2.line(out, p, q, color, 1)
        label = '{} V:{} S:{:.1f}'.format(row['feature_id'], '-' if row['vio_depth_m'] is None else '{:.1f}'.format(row['vio_depth_m']), row['stereo_depth_m'])
        cv2.putText(out, label, (p[0]+5, p[1]-5), cv2.FONT_HERSHEY_SIMPLEX, .45, color, 1, cv2.LINE_AA)
    cv2.imwrite(str(folder/(name+'_stereo_matches.jpg')), out)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source', type=Path, default=Path('/media/walker/My PSSD/S3E'))
    p.add_argument('--run', type=Path, default=Path('results/global_loop_recorded_40s'))
    p.add_argument('--pair-name', default='pair_00152.json')
    p.add_argument('--output', type=Path, default=Path('results/stereo_pair_00152'))
    args = p.parse_args(); args.output.mkdir(parents=True, exist_ok=True)
    cv2.setNumThreads(2)
    events = json.loads((args.run/'summary.json').read_text())['accepted']
    event = next(e for e in events if Path(e['result_path']).name == args.pair_name)
    original = json.loads((args.run/args.pair_name).read_text())
    frames = {}; paths = {}
    for path in (args.run/'described').glob('*.bin'):
        if path.name.endswith(('.local.bin', '.peer.bin')):
            continue
        f = Keyframe().deserialize(path.read_bytes())
        key = '{}:{}:{}'.format(f.key.agent_id, f.key.session_id, f.key.frame_id)
        if key in [event['local_key'], event['remote_key']]:
            frames[int(f.key.agent_id)] = f; paths[str(f.key.agent_id)] = dict(path=str(path), sha256=digest(path))
    if frames[1].header.stamp.to_nsec() != int(original['local_frame_id']) or frames[0].header.stamp.to_nsec() != int(original['remote_frame_id']):
        raise ValueError('expects_Carol_local_Alpha_remote')
    db = sqlite3.connect('file:'+str(args.source/'S3E_Library_1.db3')+'?mode=ro', uri=True)
    types = get_typestore(Stores.ROS2_HUMBLE)
    audit = {}; all_rows = {}; altered = {}; ray_maps = {}; checksums = {}
    with rosbag.Bag(str(args.output/'stereo_images.bag'), 'w') as bag:
        for agent, name in [(1, 'Carol'), (0, 'Alpha')]:
            frame = frames[agent]; path = args.source/(name.lower()+'.yaml'); cal = calibration(path)
            images, records = extract(db, types, name, frame.header.stamp.to_nsec(), args.output, bag)
            cached = np.frombuffer(frame.image.data, np.uint8).reshape(frame.image.height, frame.image.step)[:, :frame.image.width]
            if not np.array_equal(cached, images[0]):
                raise AssertionError('source_left_differs_from_cached_keyframe')
            if np.max(np.abs(transform(frame.body_T_camera)-cal['Tic'])) > 1e-6:
                raise AssertionError('body_camera_extrinsic_differs')
            rows, epipolar = depths(frame, images, cal); all_rows[agent] = {r['feature_id']: r for r in rows}
            changed = copy.deepcopy(frame)
            for f in changed.features:
                r = all_rows[agent][str(f.feature_id)]
                f.position_camera.x, f.position_camera.y, f.position_camera.z = r['stereo_point_left_m']
                f.position_valid = r['accepted']
            altered[agent] = changed
            ray_maps[agent] = {}
            for label, source in [('stereo', changed), ('VIO', frame)]:
                ray_frame = copy.deepcopy(source)
                for f in ray_frame.features:
                    f.position_camera.x = f.normalized.x*f.position_camera.z
                    f.position_camera.y = f.normalized.y*f.position_camera.z
                ray_maps[agent][label] = {str(f.feature_id): f for f in ray_frame.features}
            key = 'local_feature_id' if agent == 1 else 'remote_feature_id'
            ids = {m[key] for m in original['correspondences']}
            draw(name.lower(), images, rows, ids, args.output)
            accepted = [r for r in rows if r['accepted'] and r['vio_depth_m'] is not None]
            loop = [r for r in accepted if r['feature_id'] in ids]
            def stats(items):
                return {k: summary([r[k] for r in items]) for k in ['vio_depth_m', 'stereo_depth_m', 'vio_over_stereo', 'vio_minus_stereo_m', 'max_reprojection_px', 'relative_depth_change_for_half_pixel']}
            audit[name.lower()] = dict(images=records, left_pixel_identity_verified=True, calibration_sha256=digest(path),
                calibration_status='unverified_RIGHT.P_last_column_semantics_and_RIGHT.R_direction; exploratory_only',
                baseline_from_bf_m=cal['baseline_m'], P_last_column=cal['RIGHT.P'][:, 3].tolist(),
                baseline_if_P_column_is_mm=float(np.linalg.norm(cal['RIGHT.P'][:, 3])*.001),
                baseline_if_literal_projection_m=float(np.linalg.norm(np.linalg.solve(cal['RIGHT.K'], cal['RIGHT.P'][:, 3]))),
                R_right_from_left_assumed=cal['R'].tolist(), t_right_from_left_assumed_m=cal['t'].tolist(),
                epipolar_diagnostics_px=epipolar, features=len(rows), photometric_matches=sum(r['match_ok'] for r in rows),
                accepted=sum(r['accepted'] for r in rows), strict_accepted=sum(r['strict_accepted'] for r in rows),
                relaxed_accepted=sum(r['relaxed_accepted'] for r in rows), vio_depth_comparison=stats(accepted),
                loop_depth_comparison=stats(loop), calibration_rotation_sensitivity=calibration_sensitivity(
                    [r for r in rows if r['feature_id'] in ids], cal), points=rows)
            checksums[str(path)] = digest(path)
            print(name, 'features', len(rows), 'matches', audit[name.lower()]['photometric_matches'],
                  'stereo accepted', audit[name.lower()]['accepted'], 'loop depths', stats(loop), flush=True)
    db.close()
    maps = {i: {str(f.feature_id): f for f in frames[i].features} for i in frames}
    smaps = {i: {str(f.feature_id): f for f in altered[i].features} for i in frames}
    mm = original['correspondences']
    pairs = [(maps[1][m['local_feature_id']], maps[0][m['remote_feature_id']]) for m in mm]
    stereo_pairs = [(smaps[1][m['local_feature_id']], smaps[0][m['remote_feature_id']]) for m in mm]
    experiments = {'original_Carol_VIO_all20': solve(pairs, frames[1], frames[0])}
    if abs(experiments['original_Carol_VIO_all20']['translation_norm_m']-np.linalg.norm(original['translation'])) > 1e-5:
        raise AssertionError('baseline_not_reproduced')
    for agent, name in [(1, 'Carol'), (0, 'Alpha')]:
        reverse = agent == 0
        for gate in ['accepted', 'strict_accepted', 'relaxed_accepted']:
            key = 'local_feature_id' if agent == 1 else 'remote_feature_id'
            ids = [i for i, m in enumerate(mm) if all_rows[agent][m[key]][gate]]
            shared = [i for i in ids if (pairs[i][0] if agent == 1 else pairs[i][1]).position_valid]
            for label, selected in [('all_stereo_available', ids), ('same_VIO_stereo_subset', shared),
                                    ('same_original_inlier_subset', [i for i in shared if mm[i]['inlier']])]:
                for depth, pp in [('stereo', stereo_pairs), ('VIO', pairs)]:
                    if depth == 'VIO' and label == 'all_stereo_available':
                        continue
                    result = solve([pp[i] for i in selected], frames[agent], frames[1-agent], reverse=reverse,
                                   fixed=label == 'same_original_inlier_subset')
                    result['original_match_indices'] = selected
                    if result.get('solved'):
                        result['change_from_original'] = pose_change(result, experiments['original_Carol_VIO_all20'])
                    experiments[name+'_'+gate+'_'+label+'_'+depth] = result
                    if gate == 'accepted' and label == 'same_original_inlier_subset':
                        ray_pairs = [(ray_maps[1][depth][mm[i]['local_feature_id']],
                                      ray_maps[0][depth][mm[i]['remote_feature_id']]) for i in selected]
                        ray_result = solve(ray_pairs, frames[agent], frames[1-agent], reverse=reverse, fixed=True)
                        ray_result['original_match_indices'] = selected
                        experiments[name+'_depth_only_original_inliers_'+depth] = ray_result
            vkey = name+'_'+gate+'_same_VIO_stereo_subset_VIO'; skey = name+'_'+gate+'_same_VIO_stereo_subset_stereo'
            if experiments[vkey].get('solved') and experiments[skey].get('solved'):
                joint = sorted(set(experiments[vkey]['inlier_indices']) & set(experiments[skey]['inlier_indices']))
                selected = [shared[j] for j in joint]
                for depth, pp in [('stereo', stereo_pairs), ('VIO', pairs)]:
                    result = solve([pp[i] for i in selected], frames[agent], frames[1-agent], reverse=reverse, fixed=True)
                    result['original_match_indices'] = selected
                    experiments[name+'_'+gate+'_same_joint_inliers_fixed_'+depth] = result
            if gate == 'accepted':
                for group in ['same_VIO_stereo_subset', 'same_original_inlier_subset', 'same_joint_inliers_fixed']:
                    skey = name+'_'+gate+'_'+group+'_stereo'; vkey = name+'_'+gate+'_'+group+'_VIO'
                    if skey in experiments and vkey in experiments and experiments[vkey].get('solved'):
                        experiments[skey]['change_from_same_subset_VIO'] = pose_change(experiments[skey], experiments[vkey])
    out = dict(scope='offline stereo triangulation at existing left pixels; original cross-agent matches unchanged',
               calibration_validated=False, metric_truth_available=False, frame_provenance=paths,
               source_db=str(args.source/'S3E_Library_1.db3'), pair_sha256=digest(args.run/args.pair_name),
               calibration_checksums=checksums,
               matching=dict(method='pyramidal LK, bidirectional and 21/31px window consistency, 15px ZNCC',
                             fb_max_px=.7, window_max_px=1., ncc_min=.85,
                             main_max_reprojection_px_per_camera=2., strict_max_reprojection_px=1.,
                             relaxed_max_reprojection_px=3., half_pixel_depth_relative_change_max=.25,
                             left_feature_redetection=False, cross_agent_rematching=False),
               agents=audit, experiments=experiments)
    (args.output/'report.json').write_text(json.dumps(out, indent=2, allow_nan=False)+'\n')
    plot_depths(audit, original, args.output)
    with (args.output/'depths.csv').open('w') as stream:
        fields = ['agent', 'feature_id', 'vio_depth_m', 'stereo_depth_m', 'accepted', 'strict_accepted',
                  'fb_error_px', 'window_difference_px', 'ncc', 'max_reprojection_px', 'relative_depth_change_for_half_pixel']
        writer = csv.DictWriter(stream, fields); writer.writeheader()
        for name, info in audit.items():
            for row in info['points']:
                writer.writerow({k: name if k == 'agent' else row[k] for k in fields})
    for key, value in experiments.items():
        if value.get('solved'):
            print(key, 'n', value['input_points'], 'inliers', value['ransac_inliers'],
                  'length', value['translation_norm_m'], 'change', value.get('change_from_original'), flush=True)
    print('report', args.output/'report.json')


def plot_depths(audit, original, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.8))
    for ax, (name, agent) in zip(axes, audit.items()):
        key = 'local_feature_id' if name == 'carol' else 'remote_feature_id'
        ids = {m[key] for m in original['correspondences']}
        rows = [r for r in agent['points'] if r['accepted'] and r['vio_depth_m'] is not None]
        loop = [r for r in rows if r['feature_id'] in ids]
        ax.scatter([r['stereo_depth_m'] for r in rows], [r['vio_depth_m'] for r in rows],
                   s=20, alpha=.5, label='All VIO points passing stereo gates')
        ax.scatter([r['stereo_depth_m'] for r in loop], [r['vio_depth_m'] for r in loop],
                   s=48, facecolors='none', edgecolors='#c23e26', label='Original loop correspondences')
        limit = max([max(r['stereo_depth_m'], r['vio_depth_m']) for r in rows]+[1])*1.05
        ax.plot([0, limit], [0, limit], '--', color='gray', linewidth=1)
        ax.set(xlim=(0, limit), ylim=(0, limit), xlabel='Stereo depth Z (m)', ylabel='VIO depth Z (m)',
               title=name.capitalize()+' — assumed calibration')
        ax.grid(alpha=.2); ax.legend(fontsize=8)
    fig.suptitle('pair_00152: independent stereo check, calibration unverified')
    fig.tight_layout()
    fig.savefig(str(output/'depth_comparison.png'), dpi=180)
    fig.savefig(str(output/'depth_comparison.svg'))
    plt.close(fig)


if __name__ == '__main__':
    main()
