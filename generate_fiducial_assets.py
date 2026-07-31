#!/usr/bin/env python3
"""Generate printable AprilTag workspace markers and a ChArUco board."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2

from fiducial_localization import DICTIONARY_NAME, aruco_dictionary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=Path("data/fiducial_assets"))
    parser.add_argument("--dpi", type=int, default=300)
    parser.add_argument("--marker-mm", type=float, default=50.0)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    dictionary = aruco_dictionary(DICTIONARY_NAME)
    marker_px = round(args.marker_mm / 25.4 * args.dpi)
    files = []
    for marker_id in range(4):
        marker = cv2.aruco.generateImageMarker(dictionary, marker_id, marker_px, borderBits=1)
        path = args.output / f"workspace_tag_{marker_id}_{int(args.marker_mm)}mm.png"
        cv2.imwrite(str(path), marker)
        files.append(path.name)
    object_marker_mm = 30.0
    object_marker_px = round(object_marker_mm / 25.4 * args.dpi)
    object_files = []
    for marker_id in range(10, 26):
        marker = cv2.aruco.generateImageMarker(dictionary, marker_id, object_marker_px, borderBits=1)
        path = args.output / f"object_tag_{marker_id}_30mm.png"
        cv2.imwrite(str(path), marker)
        object_files.append(path.name)
    board = cv2.aruco.CharucoBoard((7, 5), 0.03, 0.022, dictionary)
    board_path = args.output / "charuco_apriltag_36h11_7x5.png"
    board_width_mm, board_height_mm = 7 * 30.0, 5 * 30.0
    board_size_px = (
        round(board_width_mm / 25.4 * args.dpi),
        round(board_height_mm / 25.4 * args.dpi),
    )
    cv2.imwrite(str(board_path), board.generateImage(board_size_px, marginSize=0, borderBits=1))
    manifest = {
        "dictionary": DICTIONARY_NAME,
        "workspaceMarkerIds": [0, 1, 2, 3],
        "workspaceMarkerSizeMm": args.marker_mm,
        "objectMarkerIds": list(range(10, 26)),
        "objectMarkerSizeMm": object_marker_mm,
        "printDpi": args.dpi,
        "printScale": "100%; disable fit-to-page scaling and verify with a ruler",
        "charuco": {
            "squaresX": 7, "squaresY": 5, "squareLengthM": 0.03, "markerLengthM": 0.022,
            "printedWidthMm": board_width_mm, "printedHeightMm": board_height_mm,
        },
        "files": files + object_files + [board_path.name],
    }
    (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps(manifest, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
