#!/usr/bin/env python3
"""Visualize every LightGlue match for a serialized RDSwarm packet pair.

All matches are drawn. PnP inliers are green and the remaining LightGlue
matches are red only to make the geometric quality visible; no match is hidden.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def read_packet(path):
    from rdswarm_msgs.msg import LocalFeaturePacket

    packet = LocalFeaturePacket()
    packet.deserialize(Path(path).read_bytes())
    return packet


def read_matches(path):
    matches = []
    for line in Path(path).read_text().splitlines():
        if not line or line.startswith("#"):
            continue
        local_index, remote_index, confidence = line.split()[:3]
        matches.append((int(local_index), int(remote_index), float(confidence)))
    return matches


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image_a", type=Path)
    parser.add_argument("image_b", type=Path)
    parser.add_argument("packet_a", type=Path)
    parser.add_argument("packet_b", type=Path)
    parser.add_argument("matches_txt", type=Path)
    parser.add_argument("pnp_json", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()

    image_a = cv2.imread(str(args.image_a), cv2.IMREAD_COLOR)
    image_b = cv2.imread(str(args.image_b), cv2.IMREAD_COLOR)
    if image_a is None or image_b is None:
        raise SystemExit("cannot_read_input_image")
    packet_a, packet_b = read_packet(args.packet_a), read_packet(args.packet_b)
    matches = read_matches(args.matches_txt)
    pnp = json.loads(args.pnp_json.read_text())
    inlier_pairs = {
        (int(item["local_feature_id"]), int(item["remote_feature_id"]))
        for item in pnp.get("correspondences", []) if item.get("inlier")
    }
    if image_a.shape[:2] != image_b.shape[:2]:
        raise SystemExit("image_size_mismatch")
    height, width = image_a.shape[:2]
    canvas = np.hstack([image_a, image_b])
    all_points_a, all_points_b = [], []
    for local_index, remote_index, confidence in matches:
        if local_index >= len(packet_a.features) or remote_index >= len(packet_b.features):
            raise SystemExit("match_index_out_of_range")
        local = packet_a.features[local_index]
        remote = packet_b.features[remote_index]
        point_a = (int(round(local.pixel.x)), int(round(local.pixel.y)))
        point_b = (int(round(remote.pixel.x)) + width, int(round(remote.pixel.y)))
        pair = (int(local.feature_id), int(remote.feature_id))
        is_inlier = pair in inlier_pairs
        color = (45, 220, 70) if is_inlier else (40, 45, 235)
        cv2.line(canvas, point_a, point_b, color, 1, cv2.LINE_AA)
        cv2.circle(canvas, point_a, 3, color, -1, cv2.LINE_AA)
        cv2.circle(canvas, point_b, 3, color, -1, cv2.LINE_AA)
        all_points_a.append(point_a)
        all_points_b.append(point_b)

    cv2.rectangle(canvas, (0, 0), (width - 1, 42), (18, 18, 18), -1)
    candidate_inliers = int(pnp.get("inliers", 0))
    accepted = pnp.get("success", 0) == 1
    cv2.putText(canvas, f"LightGlue all matches: {len(matches)}  |  PnP RANSAC candidates: {candidate_inliers}  |  accepted: {accepted}",
                (16, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (240, 240, 240), 2, cv2.LINE_AA)
    cv2.putText(canvas, "red = every retained LightGlue match (no matches removed from this image)",
                (width + 16, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (240, 240, 240), 1, cv2.LINE_AA)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(args.output), canvas, [cv2.IMWRITE_JPEG_QUALITY, 92]):
        raise SystemExit("cannot_write_output")
    print(json.dumps({"output": str(args.output), "matches": len(matches),
                      "pnp_ransac_candidate_inliers": candidate_inliers,
                      "pnp_inlier_pairs_marked": len(inlier_pairs),
                      "pnp_accepted": accepted, "image_size": [width, height]}, indent=2))


if __name__ == "__main__":
    main()
