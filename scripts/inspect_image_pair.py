#!/usr/bin/env python3
"""Inspect two explicitly extracted SuperPoint images; optionally compare VIO feature packets.

No ROS master, VIO node, retrieval database or service is started. Detection is a separate command.
"""
import argparse
import copy
import hashlib
import io
import json
from pathlib import Path
import subprocess

import cv2
import numpy as np


def read_sparse(path):
    storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not storage.isOpened():
        raise ValueError(f"cannot_read_features:{path}")
    points = storage.getNode("points").mat()
    descriptors = storage.getNode("descriptors").mat()
    valid = storage.getNode("valid").mat()
    model_path = Path(storage.getNode("model_path").string())
    image_path = Path(storage.getNode("image_path").string())
    size = (int(storage.getNode("image_width").real()), int(storage.getNode("image_height").real()))
    options = {name: storage.getNode(name).real() for name in ["score_threshold", "max_points", "min_distance", "border"]
               if not storage.getNode(name).empty()}
    storage.release()
    if points is None:
        points, descriptors, valid = np.empty((0, 2)), np.empty((0, 256)), np.empty(0)
    keep = valid.reshape(-1).astype(bool)
    sha = hashlib.sha256(model_path.read_bytes()).hexdigest()
    return {
        "points": np.asarray(points[keep], dtype=np.float32),
        "descriptors": np.asarray(descriptors[keep], dtype=np.float32),
        "ids": np.flatnonzero(keep).tolist(), "size": size,
        "model_id": "superpoint:" + sha, "image": str(image_path),
        "image_sha256": hashlib.sha256(image_path.read_bytes()).hexdigest(),
        "detector_options": options,
    }


def read_calibration(path, size):
    storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not storage.isOpened() or storage.getNode("model_type").string() != "PINHOLE":
        raise ValueError(f"pinhole_calibration_required:{path}")
    actual = (int(storage.getNode("image_width").real()), int(storage.getNode("image_height").real()))
    if actual != size:
        raise ValueError(f"image_calibration_size_mismatch:{actual}:{size}")
    projection, distortion = storage.getNode("projection_parameters"), storage.getNode("distortion_parameters")
    fx, fy, cx, cy = [projection.getNode(k).real() for k in ["fx", "fy", "cx", "cy"]]
    matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    coefficients = np.array([distortion.getNode(k).real() for k in ["k1", "k2", "p1", "p2"]])
    storage.release()
    return matrix, coefficients


def normalized(points, calibration):
    if len(points) == 0:
        return np.empty((0, 2), dtype=np.float64)
    return cv2.undistortPoints(points.reshape(-1, 1, 2), *calibration).reshape(-1, 2)


def read_vio(path, reference, calibration):
    from rdswarm_msgs.msg import LocalFeaturePacket
    packet = LocalFeaturePacket()
    packet.deserialize(path.read_bytes())
    if packet.local_descriptor_model != reference["model_id"]:
        raise ValueError("vio_and_image_descriptor_model_mismatch")
    indices = [i for i, f in enumerate(packet.features) if f.descriptor_valid]
    features = [packet.features[i] for i in indices]
    points = np.array([[f.pixel.x, f.pixel.y] for f in features], dtype=np.float32).reshape(-1, 2)
    actual_norm = np.array([[f.normalized.x, f.normalized.y] for f in features]).reshape(-1, 2)
    normalized_delta = np.linalg.norm(normalized(points, calibration) - actual_norm, axis=1)
    return {
        "points": points, "descriptors": np.array([f.descriptor for f in features], dtype=np.float32).reshape(-1, 256),
        "normalized": actual_norm, "ids": [f.feature_id for f in features],
        "size": reference["size"], "packet": packet,
        "valid_3d": sum(f.position_valid for f in features),
        "normalized_coordinate_check_max": float(normalized_delta.max()) if len(features) else None,
    }


def match(a, b, ratio, distance):
    stages = {"features_a": len(a["points"]), "features_b": len(b["points"]),
              "knn_queries": 0, "ratio_pass": 0, "distance_pass": 0, "mutual_pass": 0}
    if len(a["points"]) == 0 or len(b["points"]) < 2:
        return [], stages
    rows = []
    for item in [a, b]:
        descriptors = item["descriptors"]
        norms = np.linalg.norm(descriptors, axis=1, keepdims=True)
        if not np.isfinite(descriptors).all() or np.any(norms < 1e-10):
            raise ValueError("nonfinite_or_zero_descriptor")
        rows.append(np.asarray(descriptors / norms, dtype=np.float32))
    matcher = cv2.BFMatcher(cv2.NORM_L2)
    nearest = matcher.knnMatch(rows[0], rows[1], k=2)
    reverse = matcher.match(rows[1], rows[0])
    kept = []
    for pair in nearest:
        if len(pair) != 2:
            continue
        stages["knn_queries"] += 1
        first, second = pair
        if first.distance >= ratio * second.distance:
            continue
        stages["ratio_pass"] += 1
        if first.distance > distance:
            continue
        stages["distance_pass"] += 1
        if reverse[first.trainIdx].trainIdx != first.queryIdx:
            continue
        stages["mutual_pass"] += 1
        kept.append(first)
    return kept, stages


def coverage(points, size):
    if len(points) < 3:
        return 0.0
    return float(cv2.contourArea(cv2.convexHull(np.asarray(points, dtype=np.float32)))) / (size[0] * size[1])


def geometry(a, b, matches, threshold):
    mask = np.zeros(len(matches), dtype=bool)
    result = {"fundamental_inliers": 0, "homography_inliers": 0,
              "fundamental_threshold_normalized": threshold}
    if len(matches) < 8:
        result["status"] = "insufficient_matches"
        return result, mask
    xy_a = np.array([a["normalized"][m.queryIdx] for m in matches])
    xy_b = np.array([b["normalized"][m.trainIdx] for m in matches])
    cv2.setRNGSeed(17)
    fundamental, accepted = cv2.findFundamentalMat(xy_a, xy_b, cv2.FM_RANSAC, threshold, 0.999, 5000)
    if fundamental is not None and fundamental.shape == (3, 3) and accepted is not None:
        mask = accepted.reshape(-1).astype(bool)
        result["fundamental_inliers"] = int(mask.sum())
        result["fundamental_inlier_fraction"] = float(mask.mean())
        result["inlier_hull_fraction_a"] = coverage(np.array([a["points"][m.queryIdx] for m in matches])[mask], a["size"])
        result["inlier_hull_fraction_b"] = coverage(np.array([b["points"][m.trainIdx] for m in matches])[mask], b["size"])
    cv2.setRNGSeed(17)
    homography, homography_mask = cv2.findHomography(xy_a, xy_b, cv2.RANSAC, threshold, maxIters=5000, confidence=0.999)
    if homography is not None and homography_mask is not None:
        result["homography_inliers"] = int(homography_mask.sum())
    result["status"] = "2d_consensus_only_no_metric_pose"
    return result, mask


def write_overlay(a, b, matches, mask, path, title):
    images = [cv2.imread(item["image"], cv2.IMREAD_COLOR) for item in [a, b]]
    if any(image is None for image in images):
        raise ValueError("cannot_read_visualization_image")
    keypoints = [[cv2.KeyPoint(float(x), float(y), 3) for x, y in item["points"]] for item in [a, b]]
    chosen = [m for m, keep in zip(matches, mask) if keep]
    canvas = cv2.drawMatches(images[0], keypoints[0], images[1], keypoints[1], chosen, None,
                            matchColor=(60, 220, 60), flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    cv2.putText(canvas, title, (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 230, 255), 2)
    if not cv2.imwrite(str(path), canvas):
        raise ValueError("cannot_write_visualization")


def run_pnp(local, remote, detected, output, probe):
    from rdswarm_msgs.msg import TrackedFeature
    # 保留本机三维点；对端新检测点只有自己的二维/描述子，绝不借用旧点深度。 (Ref: 2026-10-25_Diagnose_Two_Image_Matching.md)
    packet = copy.deepcopy(remote["packet"])
    if detected is not None:
        packet.features = []
        for index, (pixel, norm, descriptor) in enumerate(zip(detected["points"], detected["normalized"], detected["descriptors"])):
            feature = TrackedFeature(feature_id=index, descriptor_valid=True, position_valid=False)
            feature.pixel.x, feature.pixel.y = map(float, pixel)
            feature.normalized.x, feature.normalized.y = map(float, norm)
            feature.normalized.z = 1.0
            feature.descriptor = descriptor.tolist()
            packet.features.append(feature)
    for feature in packet.features:
        feature.position_valid = False
        feature.position_camera.x = feature.position_camera.y = feature.position_camera.z = 0.0
        feature.depth_variance_valid = False
    packet.local_descriptor_model = local["packet"].local_descriptor_model
    paths = []
    for label, message in [("local", local["packet"]), ("peer", packet)]:
        stream = io.BytesIO()
        message.serialize(stream)
        path = output.with_name(output.stem + "_" + label + ".bin")
        path.write_bytes(stream.getvalue())
        paths.append(path)
    # 与在线 PnP 核心相同的默认门限，不随 2D 诊断参数调节。 (Ref: 2026-10-25_Diagnose_Two_Image_Matching.md)
    run = subprocess.run([str(probe), *map(str, paths), str(output)], capture_output=True, text=True)
    if run.returncode not in [0, 1]:
        raise ValueError(run.stderr)
    return json.loads(output.read_text())


def depth_diagnostics(local, pnp, focal):
    lookup = {f.feature_id: f for f in local["packet"].features if f.position_valid}
    inlier_ids = [int(m["local_feature_id"]) for m in pnp["correspondences"] if m["inlier"]]
    matched_ids = [int(m["local_feature_id"]) for m in pnp["correspondences"]]
    result = {}
    for label, ids in [("matched", matched_ids), ("inlier", inlier_ids)]:
        points = [lookup[i] for i in ids if i in lookup]
        if not points:
            continue
        depths = [f.position_camera.z for f in points]
        errors = [np.hypot(f.position_camera.x / f.position_camera.z - f.normalized.x,
                           f.position_camera.y / f.position_camera.z - f.normalized.y) * focal for f in points]
        result[label] = {"depth_z_quantiles_m": np.percentile(depths, [0, 50, 90, 100]).tolist(),
                         "local_self_reprojection_quantiles_px": np.percentile(errors, [0, 50, 90, 100]).tolist()}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--features-a", type=Path, required=True)
    parser.add_argument("--features-b", type=Path, required=True)
    parser.add_argument("--calib-a", type=Path, required=True)
    parser.add_argument("--calib-b", type=Path, required=True)
    parser.add_argument("--vio-a-packet", type=Path)
    parser.add_argument("--vio-b-packet", type=Path)
    parser.add_argument("--loop-probe", type=Path, default=Path("devel_cpu/lib/rdswarm_loop/loop_pair_probe"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--ratio", type=float, default=0.8)
    parser.add_argument("--max-distance", type=float, default=0.9)
    parser.add_argument("--geometry-threshold", type=float, default=0.004)
    args = parser.parse_args()
    if not (0 < args.ratio < 1) or not (0 < args.max_distance <= 2) or not (0 < args.geometry_threshold < 1):
        parser.error("invalid matching or geometry threshold")
    if bool(args.vio_a_packet) != bool(args.vio_b_packet):
        parser.error("supply both VIO packets or neither")
    if (args.output / "report.json").exists():
        parser.error("report already exists; use a new output directory")
    args.output.mkdir(parents=True, exist_ok=True)
    images = [read_sparse(path) for path in [args.features_a, args.features_b]]
    if images[0]["model_id"] != images[1]["model_id"]:
        raise ValueError("image_descriptor_model_mismatch")
    calibrations = [read_calibration(path, item["size"]) for path, item in zip([args.calib_a, args.calib_b], images)]
    for item, calibration in zip(images, calibrations):
        item["normalized"] = normalized(item["points"], calibration)
    modes = [("superpoint_superpoint", images[0], images[1])]
    vio = None
    if args.vio_a_packet:
        vio = [read_vio(path, image, calibration) for path, image, calibration in
               zip([args.vio_a_packet, args.vio_b_packet], images, calibrations)]
        for item, image in zip(vio, images):
            item["image"] = image["image"]
        limited = []
        for image, tracks in zip(images, vio):
            count = min(len(image["points"]), len(tracks["points"]))
            item = dict(image)
            for field in ["points", "descriptors", "ids", "normalized"]:
                item[field] = image[field][:count]
            limited.append(item)
        # 限制为同样的特征预算，区分检测位置差异与单纯增加点数的作用。 (Ref: 2026-10-25_Diagnose_Two_Image_Matching.md)
        modes = [("vio_vio", *vio), ("vio_superpoint", vio[0], images[1]),
                 ("superpoint_equal_budget", *limited), *modes]
    report = {"scope": "fixed two-image local matching; not database retrieval or full-pose ground truth",
              "model_id": images[0]["model_id"],
              "images": [{k: image[k] for k in ["image", "image_sha256", "size", "detector_options"]} for image in images],
              "superpoint_counts": [len(image["points"]) for image in images],
              "matching": {"ratio": args.ratio, "max_distance": args.max_distance}, "modes": {}}
    if vio:
        report["vio_counts"] = [len(item["points"]) for item in vio]
        report["vio_valid_3d_counts"] = [item["valid_3d"] for item in vio]
        report["vio_calibration_normalized_max_difference"] = [item["normalized_coordinate_check_max"] for item in vio]
    for name, a, b in modes:
        matches, stages = match(a, b, args.ratio, args.max_distance)
        consensus, mask = geometry(a, b, matches, args.geometry_threshold)
        record = {"stages": stages, "two_view_geometry": consensus}
        if vio and name in ["vio_vio", "vio_superpoint"]:
            record["pnp"] = run_pnp(vio[0], vio[1], None if name == "vio_vio" else images[1],
                                      args.output / (name + "_pnp.json"), args.loop_probe.resolve())
            record["local_depth_diagnostics"] = depth_diagnostics(
                vio[0], record["pnp"], float((calibrations[0][0][0, 0] + calibrations[0][0][1, 1]) / 2))
        report["modes"][name] = record
        write_overlay(a, b, matches, mask, args.output / (name + "_2d_inliers.jpg"),
                      f"{name}: {len(a['points'])}/{len(b['points'])} points, {len(matches)} matches, {int(mask.sum())} F inliers")
        if "pnp" in record and record["pnp"]["success"]:
            selected = {(int(m["local_feature_id"]), int(m["remote_feature_id"])) for m in record["pnp"]["correspondences"] if m["inlier"]}
            pnp_mask = [(a["ids"][m.queryIdx], b["ids"][m.trainIdx]) in selected for m in matches]
            write_overlay(a, b, matches, pnp_mask, args.output / (name + "_pnp_inliers.jpg"),
                          f"{name}: {record['pnp']['inliers']} PnP inliers (same default thresholds)")
        print(name, stages, "F_inliers=", consensus["fundamental_inliers"],
              "PnP_inliers=", record.get("pnp", {}).get("inliers", "requires local depth"), flush=True)
    (args.output / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    main()
