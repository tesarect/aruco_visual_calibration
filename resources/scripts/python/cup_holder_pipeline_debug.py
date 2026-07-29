#!/usr/bin/env python3
"""Standalone (no ROS) classical image-processing pipeline dump for tuning
cup_holder_detector_node's thresholds offline, against a single saved
frame — not a live tool, just a batch of intermediate-stage images to
visually compare.

Why this exists: cup_holder_detector_node.cpp currently thresholds on
PLAIN GRAYSCALE brightness only (cv::threshold, THRESH_BINARY /
THRESH_BINARY_INV) to separate the white cup_holder disc from its 4
reddish-brown holes. Live testing (2026-07-29) found the disc itself
detected fine, but 0 holes detected — hole_thresh may be wrong, OR
grayscale brightness alone may not be able to separate the holes'
reddish-brown color from the disc's own shadowed/anti-aliased edge
pixels no matter what cutoff is picked (a hue/color problem, not a
threshold-number problem). This script produces every candidate
intermediate stage as its own saved image so that can be judged by eye
before committing to a specific approach/number in
cup_holder_detector_sim.yaml.

Get a frame to run this against via capture_camera.py first, e.g.:
    python3 capture_camera.py --env sim --out ~/captures --count 3
then pick one saved frame (they're plain image files, any format
cv2.imread can read) and pass it here.

Usage:
    python3 cup_holder_pipeline_debug.py --image ~/captures/frame_0.png --out ~/cupholder_debug

Produces, all under --out (created if missing), each prefixed with a
stage number so `ls` sorts them in pipeline order:
    00_original.png                 — input, untouched, for side-by-side reference
    01_grayscale.png                — cvtColor BGR2GRAY
    02_grayscale_blurred.png        — gray + GaussianBlur (reduces per-pixel noise
                                       before thresholding — a single stray bright/dark
                                       pixel can otherwise become its own tiny "contour")
    03_hsv.png                      — cvtColor BGR2HSV, false-colored for viewing
                                       (H channel repeated across all 3 output channels
                                       so hue differences are visible as gray-level
                                       differences in a normal image viewer)
    04_hue_channel.png              — H channel alone, grayscale
    05_saturation_channel.png       — S channel alone, grayscale
    06_value_channel.png            — V channel alone, grayscale (~= brightness,
                                       for comparison against 01_grayscale.png)
    07_gray_thresh_<N>.png          — THRESH_BINARY at several candidate grayscale
                                       cutoffs (a small sweep, not just one number)
    08_gray_thresh_inv_<N>.png      — THRESH_BINARY_INV at the same cutoffs (this is
                                       the actual mode cup_holder_detector_node.cpp
                                       uses for hole_thresh — pixels <= N kept)
    09_hue_thresh_<LO>_<HI>.png     — inRange on the HUE channel only, a few candidate
                                       bands (tests whether hue alone separates the
                                       reddish holes from the neutral-gray disc/shadow,
                                       independent of brightness)
    10_hsv_inrange_<name>.png       — inRange on full HSV bounds for a couple of named
                                       candidate "reddish-brown hole" guesses
    11_contours_on_<source>.png     — cv2.findContours + drawContours overlay, run on
                                       whichever single mask (from the stages above)
                                       looks most promising by eye — see --contour-stage

Nothing here is wired into cup_holder_detector_node.cpp automatically —
this is a visual comparison tool only. Once a stage/cutoff is confirmed
by eye to cleanly isolate the 4 holes, that number gets hand-copied into
cup_holder_detector_sim.yaml and re-verified live via ros2 param set.
"""

import argparse
import os

import cv2
import numpy as np


GRAY_THRESH_CANDIDATES = [60, 75, 90, 105, 120, 135]

# A few candidate reddish-brown hue bands to try, OpenCV hue range 0-179.
# Red wraps around 0/179 in OpenCV's HSV, so a reddish-brown might sit
# near either end — both a low-end and high-end band are included as
# starting guesses, NOT confirmed against this project's actual sim
# rendering yet.
HUE_BAND_CANDIDATES = [(0, 15), (0, 25), (160, 179)]

HSV_NAMED_CANDIDATES = {
    # (lower_h, lower_s, lower_v), (upper_h, upper_s, upper_v) — first-pass
    # guesses for a dark reddish-brown, wide net on purpose since these are
    # meant to be judged by eye, not trusted as final.
    "reddish_brown_wide": ((0, 40, 20), (25, 255, 160)),
    "reddish_brown_narrow": ((0, 60, 30), (15, 255, 120)),
}


def save(out_dir, filename, image):
    path = os.path.join(out_dir, filename)
    cv2.imwrite(path, image)
    print(f"  wrote {path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", required=True, help="Path to a saved BGR frame (any cv2.imread-readable format)")
    parser.add_argument("--out", default="~/cupholder_debug", help="Output directory for stage images")
    parser.add_argument(
        "--contour-stage",
        default="08_gray_thresh_inv_90",
        help="Which stage filename (without extension/prefix path) to run findContours on for stage 11 "
        "— pick this AFTER looking at the other stages, re-run with this flag once you know which mask looks cleanest",
    )
    args = parser.parse_args()

    image_path = os.path.expanduser(args.image)
    out_dir = os.path.expanduser(args.out)
    os.makedirs(out_dir, exist_ok=True)

    image = cv2.imread(image_path)
    if image is None:
        raise SystemExit(f"Could not read image: {image_path}")

    print(f"Loaded {image_path}, shape={image.shape}")
    print(f"Writing stages to {out_dir}\n")

    save(out_dir, "00_original.png", image)

    # --- Stage 1-2: grayscale + blur ---
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    save(out_dir, "01_grayscale.png", gray)

    gray_blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    save(out_dir, "02_grayscale_blurred.png", gray_blurred)

    # --- Stage 3-6: HSV and its channels ---
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hue, sat, val = cv2.split(hsv)

    hsv_viewable = cv2.merge([hue, hue, hue])
    save(out_dir, "03_hsv.png", hsv_viewable)
    save(out_dir, "04_hue_channel.png", hue)
    save(out_dir, "05_saturation_channel.png", sat)
    save(out_dir, "06_value_channel.png", val)

    # --- Stage 7-8: grayscale threshold sweep (BINARY and BINARY_INV) ---
    for t in GRAY_THRESH_CANDIDATES:
        _, mask = cv2.threshold(gray_blurred, t, 255, cv2.THRESH_BINARY)
        save(out_dir, f"07_gray_thresh_{t}.png", mask)

        _, mask_inv = cv2.threshold(gray_blurred, t, 255, cv2.THRESH_BINARY_INV)
        save(out_dir, f"08_gray_thresh_inv_{t}.png", mask_inv)

    # --- Stage 9: hue-only band threshold (brightness-independent) ---
    for lo, hi in HUE_BAND_CANDIDATES:
        hue_mask = cv2.inRange(hue, lo, hi)
        save(out_dir, f"09_hue_thresh_{lo}_{hi}.png", hue_mask)

    # --- Stage 10: full HSV inRange, named candidate guesses ---
    for name, (lower, upper) in HSV_NAMED_CANDIDATES.items():
        hsv_mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
        save(out_dir, f"10_hsv_inrange_{name}.png", hsv_mask)

    # --- Stage 11: contours drawn on whichever stage looks best ---
    contour_source_path = os.path.join(out_dir, args.contour_stage + ".png")
    if os.path.exists(contour_source_path):
        mask = cv2.imread(contour_source_path, cv2.IMREAD_GRAYSCALE)
        contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        overlay = image.copy()
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 10:  # skip single-pixel noise contours, not a real filter
                continue
            perimeter = cv2.arcLength(contour, True)
            circularity = 0.0 if perimeter <= 0 else 4 * np.pi * area / (perimeter * perimeter)
            (cx, cy), radius = cv2.minEnclosingCircle(contour)
            color = (0, 255, 0) if circularity > 0.6 else (0, 0, 255)
            cv2.circle(overlay, (int(cx), int(cy)), int(radius), color, 2)
            cv2.putText(
                overlay, f"a={int(area)} c={circularity:.2f}",
                (int(cx) - 30, int(cy) - int(radius) - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1,
            )
        save(out_dir, f"11_contours_on_{args.contour_stage}.png", overlay)
        print(f"\nContour stage found {len(contours)} raw contours (area>=10) on {args.contour_stage}.png")
        print("Green circle = circularity > 0.6 (would currently pass hole_min_circularity), red = would not.")
    else:
        print(
            f"\n--contour-stage '{args.contour_stage}' not found at {contour_source_path} — "
            "skipping stage 11. Look through the other stages first, then re-run with "
            "--contour-stage set to whichever filename (no extension) looks cleanest."
        )

    print("\nDone. Pull the output directory back and look through stages in order:")
    print("  01/02 vs 06 — does grayscale alone show the holes clearly darker than the disc?")
    print("  04/05 — do the holes show a clearly different hue/saturation than the disc and its shadow?")
    print("  07/08 — at which threshold N do the 4 holes appear as clean white blobs, with nothing else?")
    print("  09/10 — does a hue-only or full-HSV mask isolate the holes better than grayscale did?")


if __name__ == "__main__":
    main()