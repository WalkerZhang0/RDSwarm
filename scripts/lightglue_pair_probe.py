#!/usr/bin/env python3
"""Run matcher-only LightGlue ONNX on two serialized RDSwarm packets.

The packet feature indices are preserved and the resulting matches are passed
to the existing C++ PnP verification core. This does not run SuperPoint image
extraction; the packet descriptors are the fixed input for a matcher A/B test.
"""
import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

import numpy as np


def read_packet(path):
    from rdswarm_msgs.msg import LocalFeaturePacket

    packet = LocalFeaturePacket()
    packet.deserialize(Path(path).read_bytes())
    return packet


def packet_features(packet, local, image_size):
    selected = []
    for index, feature in enumerate(packet.features):
        if not feature.descriptor_valid:
            continue
        if local and not feature.position_valid:
            continue
        if len(feature.descriptor) != packet.descriptor_dimension:
            raise ValueError("descriptor_dimension_mismatch")
        selected.append((index, feature))
    if not selected:
        raise ValueError("no_features_for_lightglue")
    descriptors = np.asarray([feature.descriptor for _, feature in selected], dtype=np.float32)
    norms = np.linalg.norm(descriptors, axis=1, keepdims=True)
    if not np.isfinite(descriptors).all() or np.any(norms < 1e-10):
        raise ValueError("nonfinite_or_zero_descriptor")
    descriptors /= norms
    points = np.asarray([[feature.pixel.x, feature.pixel.y] for _, feature in selected], dtype=np.float32)
    width, height = image_size
    if width <= 0 or height <= 0:
        raise ValueError("packet_image_size_missing")
    size = np.asarray([width, height], dtype=np.float32)
    points_normalized = (points - size / 2.0) / (size.max() / 2.0)
    return selected, points_normalized.astype(np.float32), descriptors


def descriptor_feed(info, descriptors):
    shape = info.shape
    if len(shape) != 3:
        raise ValueError(f"unsupported_descriptor_input_shape:{shape}")
    if shape[2] == 256:
        return descriptors[None]
    if shape[1] == 256:
        return descriptors.T[None]
    raise ValueError(f"unsupported_descriptor_layout:{shape}")


def run_lightglue(packet_a, packet_b, model_path, threshold, threads, size_a, size_b):
    try:
        import onnxruntime as ort
    except ImportError as error:
        raise RuntimeError("onnxruntime_python_required") from error

    selected_a, points_a, descriptors_a = packet_features(packet_a, local=True, image_size=size_a)
    selected_b, points_b, descriptors_b = packet_features(packet_b, local=False, image_size=size_b)
    options = ort.SessionOptions()
    options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    if threads > 0:
        options.intra_op_num_threads = threads
    load_start = time.perf_counter()
    session = ort.InferenceSession(str(model_path), sess_options=options,
                                   providers=["CPUExecutionProvider"])
    load_ms = (time.perf_counter() - load_start) * 1000.0
    inputs = {item.name: item for item in session.get_inputs()}
    required = {"keypoints_0", "keypoints_1", "descriptors_0", "descriptors_1"}
    if not required.issubset(inputs):
        raise ValueError("lightglue_matcher_input_names_mismatch")
    feed = {
        "keypoints_0": points_a[None],
        "keypoints_1": points_b[None],
        "descriptors_0": descriptor_feed(inputs["descriptors_0"], descriptors_a),
        "descriptors_1": descriptor_feed(inputs["descriptors_1"], descriptors_b),
    }
    warmup_start = time.perf_counter()
    session.run(None, feed)
    warmup_ms = (time.perf_counter() - warmup_start) * 1000.0
    infer_start = time.perf_counter()
    output = session.run(None, feed)
    infer_ms = (time.perf_counter() - infer_start) * 1000.0
    scores = np.asarray(output[0])[0]
    if scores.shape != (len(selected_a), len(selected_b)):
        raise ValueError(f"lightglue_score_shape_mismatch:{scores.shape}")
    decode_start = time.perf_counter()
    best_b = scores.argmax(axis=1)
    best_a = scores.argmax(axis=0)
    row_indices = np.arange(scores.shape[0])
    log_scores = scores[row_indices, best_b]
    confidence = np.exp(np.clip(log_scores, -80.0, 0.0))
    mutual = best_a[best_b] == row_indices
    keep = mutual & (confidence >= threshold)
    matches = [
        (selected_a[i][0], selected_b[int(best_b[i])][0], float(confidence[i]))
        for i in row_indices[keep]
    ]
    decode_ms = (time.perf_counter() - decode_start) * 1000.0
    return matches, {
        "lightglue_model": str(model_path),
        "lightglue_provider": "CPUExecutionProvider",
        "lightglue_local_features": len(selected_a),
        "lightglue_remote_features": len(selected_b),
        "lightglue_match_threshold": threshold,
        "lightglue_candidate_matches": len(matches),
        "lightglue_model_load_ms": load_ms,
        "lightglue_warmup_ms": warmup_ms,
        "lightglue_inference_ms": infer_ms,
        "lightglue_decode_ms": decode_ms,
        "lightglue_total_ms": load_ms + warmup_ms + infer_ms + decode_ms,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("local_packet", type=Path)
    parser.add_argument("remote_packet", type=Path)
    parser.add_argument("result_json", type=Path)
    parser.add_argument("--model", type=Path,
                        default=Path("/home/walker/LightGlue-ONNX/weights/512/superpoint_lightglue.onnx"))
    parser.add_argument("--loop-probe", type=Path,
                        default=Path("devel_cpu/lib/rdswarm_loop/loop_pair_probe"))
    parser.add_argument("--score-threshold", type=float, default=0.10)
    parser.add_argument("--size-a", type=int, nargs=2, required=True, metavar=("WIDTH", "HEIGHT"))
    parser.add_argument("--size-b", type=int, nargs=2, required=True, metavar=("WIDTH", "HEIGHT"))
    parser.add_argument("--threads", type=int, default=0)
    parser.add_argument("--keep-matches", action="store_true")
    args = parser.parse_args()
    if not 0.0 < args.score_threshold <= 1.0:
        parser.error("score-threshold must be in (0, 1]")
    if not args.model.is_file():
        raise SystemExit(f"model_not_found:{args.model}")
    loop_probe = args.loop_probe if args.loop_probe.is_absolute() else Path.cwd() / args.loop_probe
    if not loop_probe.is_file():
        raise SystemExit(f"loop_probe_not_found:{loop_probe}")

    packet_a = read_packet(args.local_packet)
    packet_b = read_packet(args.remote_packet)
    if packet_a.local_descriptor_model != packet_b.local_descriptor_model:
        raise SystemExit("descriptor_model_mismatch")
    if packet_a.descriptor_dimension != 256 or packet_b.descriptor_dimension != 256:
        raise SystemExit("lightglue_superpoint_requires_256d_descriptors")
    matches, timing = run_lightglue(packet_a, packet_b, args.model, args.score_threshold,
                                    args.threads, tuple(args.size_a), tuple(args.size_b))

    args.result_json.parent.mkdir(parents=True, exist_ok=True)
    match_path = args.result_json.with_suffix(".lightglue_matches.txt")
    with match_path.open("w") as output:
        output.write("# local_packet_feature_index remote_packet_feature_index confidence\n")
        for local_index, remote_index, confidence in matches:
            output.write(f"{local_index} {remote_index} {confidence:.9g}\n")
    probe_result = args.result_json.with_suffix(".pnp.json")
    command = [str(loop_probe), str(args.local_packet), str(args.remote_packet),
               str(probe_result), str(match_path)]
    run = subprocess.run(command, capture_output=True, text=True)
    if run.returncode not in (0, 1):
        raise SystemExit(run.stderr.strip() or "loop_probe_failed")
    result = json.loads(probe_result.read_text())
    result.update(timing)
    result["lightglue_matches_file"] = str(match_path)
    result["lightglue_probe_stdout"] = run.stdout.strip()
    args.result_json.write_text(json.dumps(result, indent=2) + "\n")
    if not args.keep_matches:
        match_path.unlink(missing_ok=True)
    print(json.dumps(result, indent=2))
    return 0 if run.returncode == 0 else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(str(error), file=sys.stderr)
        raise SystemExit(2)
