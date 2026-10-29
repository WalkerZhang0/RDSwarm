#!/usr/bin/python3
"""Causal global retrieval + the existing C++ SP/BF/PnP on recorded VIO keyframes."""
import argparse
import copy
import io
import json
import subprocess
import time
from pathlib import Path

import numpy as np
import rosbag
from rdswarm_msgs.msg import Keyframe, LocalFeaturePacket
from rdswarm_runtime.global_descriptor import MobileNetVLAD, mono8
from rdswarm_runtime.records import EventLog, frame_path, key_text, write_message


def packet(frame, xyz):
    out = LocalFeaturePacket()
    out.header, out.key, out.body_T_camera = frame.header, frame.key, frame.body_T_camera
    out.local_descriptor_model = frame.local_descriptor_model
    out.features = [copy.deepcopy(f) for f in frame.features if f.descriptor_valid]
    out.descriptor_dimension = len(out.features[0].descriptor) if out.features else 0
    if not xyz:
        for feature in out.features:
            feature.position_valid = feature.depth_variance_valid = False
            feature.position_camera.x = feature.position_camera.y = feature.position_camera.z = 0
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--keyframes', type=Path, required=True)
    parser.add_argument('--global-model', type=Path, default=Path('models/mobilenetvlad.onnx'))
    parser.add_argument('--sp-model', type=Path, default=Path('models/superpoint_dense.onnx'))
    parser.add_argument('--calib-a', type=Path, default=Path(__file__).resolve().parents[1]/'src/rdswarm_bringup/config/s3e/alpha_cam0.yaml')
    parser.add_argument('--calib-b', type=Path, default=Path(__file__).resolve().parents[1]/'src/rdswarm_bringup/config/s3e/carol_cam0.yaml')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--devel', type=Path, default=Path('devel_cpu'))
    parser.add_argument('--top-k', type=int, default=3)
    parser.add_argument('--min-similarity', type=float, default=-1)
    parser.add_argument('--width', type=int, default=320)
    parser.add_argument('--height', type=int, default=240)
    parser.add_argument('--threads', type=int, default=2)
    parser.add_argument('--max-points', type=int, default=400)
    parser.add_argument('--limit', type=int, default=0)
    args = parser.parse_args()
    if args.top_k < 1 or args.max_points < 1 or args.limit < 0 or not -1 <= args.min_similarity <= 1:
        parser.error('invalid_evaluation_options')
    args.output.mkdir(parents=True, exist_ok=True)
    if (args.output / 'events.jsonl').exists():
        parser.error('use_a_new_output_directory')
    log = EventLog(args.output / 'events.jsonl')
    model = MobileNetVLAD(args.global_model, args.width, args.height, args.threads)
    describe = args.devel.resolve() / 'lib/rdswarm_loop/keyframe_describe_probe'
    verify = args.devel.resolve() / 'lib/rdswarm_loop/loop_pair_probe'
    raw_paths, packet_paths, descriptor_rows, metadata = {}, {}, [], []
    calibrations = {0: args.calib_a, 1: args.calib_b}
    attempts, accepted, global_times, sp_times, results = set(), [], [], [], []

    def ensure_packet(key):
        if key in packet_paths:
            return packet_paths[key]
        raw_path, frame_key = raw_paths[key]
        enriched_path = frame_path(args.output / 'described', frame_key)
        enriched_path.parent.mkdir(parents=True, exist_ok=True)
        report_path = enriched_path.with_suffix('.json')
        start = time.perf_counter()
        run = subprocess.run([str(describe), str(raw_path), str(args.sp_model.resolve()),
            str(calibrations[frame_key.agent_id].resolve()), str(enriched_path), str(report_path),
            '1', str(args.max_points)], capture_output=True, text=True, timeout=120)
        if run.returncode:
            raise RuntimeError(run.stdout + run.stderr)
        enriched = Keyframe().deserialize(enriched_path.read_bytes())
        timing = json.loads(report_path.read_text())
        sp_times.append(timing['sp_infer_ms'] + timing['sp_enrich_ms'])
        log('describe', key=key, service_wall_ms=(time.perf_counter()-start)*1000, **timing)
        local_path, peer_path = enriched_path.with_suffix('.local.bin'), enriched_path.with_suffix('.peer.bin')
        write_message(local_path, packet(enriched, True))
        write_message(peer_path, packet(enriched, False))
        packet_paths[key] = (local_path, peer_path)
        return packet_paths[key]

    start = time.perf_counter()
    with rosbag.Bag(str(args.keyframes)) as bag:
        for _, dynamic, arrival in bag.read_messages():
            if getattr(dynamic, '_type', '') != 'rdswarm_msgs/Keyframe':
                continue
            buffer = io.BytesIO(); dynamic.serialize(buffer)
            frame = Keyframe().deserialize(buffer.getvalue())
            if frame.key.agent_id not in calibrations:
                raise ValueError('provide_calibration_for_every_agent')
            key = key_text(frame.key)
            if key in raw_paths:
                continue
            path = frame_path(args.output / 'raw', frame.key)
            write_message(path, frame); raw_paths[key] = (path, frame.key)
            if not frame.image_included or frame.image.header.stamp != frame.header.stamp:
                raise ValueError('invalid_keyframe_image')
            row, timing = model.compute(mono8(frame.image))
            global_times.append(timing['total_ms'])
            # 只检索此前实际读到的邻机摘要，不利用未来帧或 GT 构造候选。 (Ref: 2026-10-28_Add_Global_Retrieval.md)
            peers = [i for i, m in enumerate(metadata) if m['agent'] != frame.key.agent_id]
            ranked = []
            if peers:
                scores = np.asarray(descriptor_rows, np.float32)[peers] @ row
                ranked = sorted(zip(peers, scores.tolist()), key=lambda item: (-item[1], metadata[item[0]]['key']))[:args.top_k]
            log('global_descriptor', key=key, capture_ns=frame.header.stamp.to_nsec(),
                arrival_ns=arrival.to_nsec(), model_id=model.model_id, trigger='keyframe',
                candidates=len(ranked), dimension=4096, **timing)
            current = dict(key=key, agent=frame.key.agent_id, capture_ns=frame.header.stamp.to_nsec())
            metadata.append(current); descriptor_rows.append(row)
            for rank, (index, similarity) in enumerate(ranked):
                if similarity < args.min_similarity:
                    continue
                remote = metadata[index]
                pair_id = tuple(sorted((key, remote['key'])))
                if pair_id in attempts:
                    continue
                attempts.add(pair_id)
                result_path = args.output / ('pair_{:05d}.json'.format(len(attempts)))
                event = dict(local_key=key, remote_key=remote['key'], rank=rank+1,
                             similarity=similarity, arrival_ns=arrival.to_nsec())
                try:
                    local_path = ensure_packet(key)[0]
                    peer_path = ensure_packet(remote['key'])[1]
                    run = subprocess.run([str(verify), str(local_path), str(peer_path), str(result_path)],
                                         capture_output=True, text=True, timeout=30)
                    if run.returncode not in (0, 1):
                        raise RuntimeError(run.stdout + run.stderr)
                    result = json.loads(result_path.read_text())
                    event.update(success=bool(result['success']), status=result['status'],
                        matches=result['matches'], inliers=result['inliers'],
                        matching_ms=result['matching_ms'], geometry_ms=result['geometry_ms'],
                        result_path=str(result_path))
                    if result['success']:
                        event.update(translation=result['translation'], quaternion_xyzw=result['quaternion_xyzw'])
                        accepted.append(event.copy())
                except Exception as error:
                    event.update(success=False, status='processing_error', error=str(error))
                log('verification', **event); results.append(event)
            print('frames={} candidates={} accepted={}'.format(len(metadata), len(attempts), len(accepted)), flush=True)
            if args.limit and len(metadata) >= args.limit:
                break
    np.savez(args.output / 'global_descriptors.npz', descriptors=np.asarray(descriptor_rows),
             keys=np.asarray([m['key'] for m in metadata]), model_id=model.model_id)
    summary = dict(scope='recorded VIO keyframes; causal retrieval; geometry acceptance is not ground-truth precision',
        global_model_id=model.model_id, dimension=4096, top_k=args.top_k,
        min_similarity=args.min_similarity, max_points=args.max_points, grid=[16,12],
        keyframes=len(metadata), per_agent={str(a):sum(m['agent']==a for m in metadata) for a in calibrations},
        unique_pairs_attempted=len(attempts), geometry_accepted=len(accepted),
        processing_errors=sum(r['status']=='processing_error' for r in results),
        sp_described_keyframes=len(packet_paths), elapsed_s=time.perf_counter()-start,
        global_ms_quantiles=np.percentile(global_times,[0,50,95,100]).tolist() if global_times else [],
        sp_ms_quantiles=np.percentile(sp_times,[0,50,95,100]).tolist() if sp_times else [],
        accepted=accepted, full_pose_gt_evaluated=False)
    (args.output / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({k:v for k,v in summary.items() if k!='accepted'}, indent=2))
    return 0 if summary['processing_errors']==0 else 1


if __name__ == '__main__':
    raise SystemExit(main())
