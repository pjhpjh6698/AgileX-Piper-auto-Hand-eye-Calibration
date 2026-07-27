#!/usr/bin/env python3
"""Generate the ArUco marker texture used by the Gazebo calibration target.

The printed/rendered plate is LARGER than the marker itself: ArUco needs a
white quiet zone around the black border or detection fails at an angle. So:

    plate_size = marker_length * (1 + 2 * pad_ratio)

The Gazebo box uses ``plate_size`` while ``marker_length`` in aruco.yaml is the
black marker's side. Getting this pair wrong scales the recovered depth, which
is one of the classic silent hand-eye failures -- hence the printed summary.

Usage (defaults match config/sim_aruco.yaml):
    python3 generate_aruco_texture.py
    python3 generate_aruco_texture.py --marker-id 1 --marker-length 0.10
"""

import argparse
import os

import cv2
import numpy as np


def build_texture(dictionary_name: str, marker_id: int,
                  marker_px: int, pad_ratio: float) -> np.ndarray:
    aruco = cv2.aruco
    dict_id = getattr(aruco, dictionary_name, None)
    if dict_id is None:
        raise SystemExit(f"unknown dictionary '{dictionary_name}'")
    if hasattr(aruco, "getPredefinedDictionary"):
        dictionary = aruco.getPredefinedDictionary(dict_id)
    else:  # very old OpenCV
        dictionary = aruco.Dictionary_get(dict_id)

    # version-safe marker rendering
    if hasattr(aruco, "generateImageMarker"):          # OpenCV >= 4.7
        marker = aruco.generateImageMarker(dictionary, marker_id, marker_px)
    else:                                              # OpenCV 4.5/4.6
        marker = aruco.drawMarker(dictionary, marker_id, marker_px)

    pad = int(round(marker_px * pad_ratio))
    plate = np.full((marker_px + 2 * pad, marker_px + 2 * pad), 255, np.uint8)
    plate[pad:pad + marker_px, pad:pad + marker_px] = marker
    return plate


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    default_out = os.path.join(here, "..", "models", "aruco_marker",
                               "materials", "textures", "aruco_marker.png")

    ap = argparse.ArgumentParser()
    ap.add_argument("--dictionary", default="DICT_4X4_50")
    ap.add_argument("--marker-id", type=int, default=1)
    ap.add_argument("--marker-length", type=float, default=0.10,
                    help="black marker side length in METERS")
    ap.add_argument("--marker-px", type=int, default=600)
    ap.add_argument("--pad-ratio", type=float, default=0.25,
                    help="white quiet zone per side, as a fraction of the marker")
    ap.add_argument("--out", default=os.path.normpath(default_out))
    args = ap.parse_args()

    plate = build_texture(args.dictionary, args.marker_id,
                          args.marker_px, args.pad_ratio)
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    cv2.imwrite(args.out, plate)

    plate_size = args.marker_length * (1.0 + 2.0 * args.pad_ratio)
    print(f"wrote {args.out}  ({plate.shape[1]}x{plate.shape[0]} px)")
    print("-" * 68)
    print(f"  dictionary       : {args.dictionary}")
    print(f"  marker id        : {args.marker_id}")
    print(f"  marker_length    : {args.marker_length:.4f} m   <- put THIS in aruco.yaml")
    print(f"  plate (box) size : {plate_size:.4f} m   <- put THIS in the .sdf box")
    print("-" * 68)


if __name__ == "__main__":
    main()
