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

    --- Added 2026-07-30: the cup_holder DISC itself turned out to have
    almost no brightness separation from the background wall in the actual
    sim frame (live-tested: holes threshold perfectly at gray~120-135, but
    no single global cutoff isolates the disc from the wall — they're
    nearly the same brightness, only the disc's thin rim/edge differs).
    These stages try edge/contrast-based approaches instead of a flat
    brightness cutoff:
    12_canny_<lo>_<hi>.png          — cv2.Canny edge detection, a few threshold pairs.
                                       Should show the disc's rim as a thin closed loop
                                       even where flat-region brightness doesn't separate.
    13_canny_dilated_<lo>_<hi>.png  — same Canny result + a small dilate — Canny edges
                                       are 1px and often have small gaps; findContours
                                       on a broken loop won't close into one clean
                                       circular contour. Dilating bridges small gaps so
                                       the rim becomes one continuous closed shape.
    14_adaptive_thresh_<blocksize>_<C>.png — cv2.adaptiveThreshold (Gaussian), a local
                                       (per-neighborhood) threshold instead of one global
                                       cutoff — should pick up the disc/wall boundary
                                       even if their ABSOLUTE brightness is similar, as
                                       long as there's a local edge between them.
    15_morph_close_on_<source>.png  — cv2.morphologyEx MORPH_CLOSE on whichever binary
                                       mask looks most promising — fills small gaps/holes
                                       inside a candidate disc blob (e.g. the 4 dark holes
                                       poking through a bright-disc mask) without changing
                                       its outer boundary, so the disc reads as one solid
                                       blob for contour/circularity fitting. See
                                       --morph-close-stage.
    16_contours_on_<source>.png     — same as stage 11 but for the edge/adaptive stages,
                                       see --contour-stage-2.

    --- Added 2026-07-30 (round 2): 13_canny_dilated_80_200 and
    14_adaptive_thresh_25_5 both came back clean (disc rim as one closed
    loop, all 4 holes visible) — confirms edge/local-contrast detection
    works where flat-brightness thresholding didn't. Given a clean edge
    map, cv2.HoughCircles is a more direct alternative to
    findContours+minEnclosingCircle+circularity-filter — it searches for
    circular shapes directly via gradient voting and returns (x, y,
    radius) with no separate circularity scoring step needed. Added here
    to compare against the contour path on the SAME image.
    17_hough_circles_<source>.png   — cv2.HoughCircles run on `source` (grayscale
                                       input, NOT a binary mask — Hough uses its own
                                       internal Canny + accumulator, so this can take
                                       02_grayscale_blurred directly), every detected
                                       circle drawn + labeled with radius. See
                                       --hough-source and --hough-param2 (accumulator
                                       threshold — LOWER finds more circles, including
                                       false positives; HIGHER is stricter).

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

# (low, high) threshold pairs for cv2.Canny — OpenCV's own docs recommend a
# high:low ratio of roughly 2:1 to 3:1; a small sweep around that, not a
# single guess.
CANNY_CANDIDATES = [(30, 90), (50, 150), (80, 200)]

# (blockSize, C) pairs for cv2.adaptiveThreshold — blockSize must be odd
# and larger than the feature you're trying to separate (the disc's rim is
# a thin line, but blockSize also needs to span enough of the disc/wall
# transition to compute a meaningful local mean — too small picks up noise,
# too large approaches a global threshold and loses the local-contrast
# advantage). C is subtracted from the local mean before the cutoff —
# higher C = stricter (fewer pixels pass).
ADAPTIVE_THRESH_CANDIDATES = [(11, 2), (25, 5), (51, 5), (75, 8)]


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
    parser.add_argument(
        "--morph-close-stage",
        default=None,
        help="Which stage filename (without extension) to run MORPH_CLOSE on for stage 15 "
        "— e.g. one of the 14_adaptive_thresh_* outputs once you've picked one. Skipped if not set.",
    )
    parser.add_argument(
        "--morph-close-kernel",
        type=int,
        default=9,
        help="Square kernel size (pixels) for stage 15's MORPH_CLOSE — bigger closes larger gaps "
        "(e.g. the 4 holes poking through a disc mask) but can also merge nearby unrelated blobs.",
    )
    parser.add_argument(
        "--contour-stage-2",
        default=None,
        help="Which stage filename (without extension) to run findContours on for stage 16 "
        "— pick this AFTER looking at the edge/adaptive/morph-close stages.",
    )
    parser.add_argument(
        "--hough-source",
        default="02_grayscale_blurred",
        help="Which stage filename (without extension) to run cv2.HoughCircles on for stage 17 "
        "— defaults to the blurred grayscale (Hough's normal input; it does its own internal "
        "edge detection, so a pre-thresholded binary mask is NOT required/recommended here).",
    )
    parser.add_argument(
        "--hough-param2",
        type=int,
        default=30,
        help="cv2.HoughCircles accumulator threshold — lower finds MORE circles (more false "
        "positives), higher is stricter (may miss the real one). Tune this after looking at "
        "the first run's output.",
    )
    parser.add_argument(
        "--hough-min-radius", type=int, default=10,
        help="cv2.HoughCircles minRadius in pixels — set from eyeballing a hole's radius in "
        "the 00_original.png / 11_contours_on_* images.",
    )
    parser.add_argument(
        "--hough-max-radius", type=int, default=150,
        help="cv2.HoughCircles maxRadius in pixels — set from eyeballing the disc's radius.",
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
    run_contour_stage(out_dir, image, args.contour_stage, "11_contours_on")

    # --- Stage 12-13: Canny edge detection, raw and dilated ---
    for lo, hi in CANNY_CANDIDATES:
        edges = cv2.Canny(gray_blurred, lo, hi)
        save(out_dir, f"12_canny_{lo}_{hi}.png", edges)

        # 3x3 dilate, 1 iteration: bridges small gaps in the 1px Canny line
        # so a rim that's a closed loop in reality, but has a few broken
        # pixels in the raw edge map, becomes one continuous closed contour
        # for findContours to pick up as a single shape.
        dilated = cv2.dilate(edges, np.ones((3, 3), np.uint8), iterations=1)
        save(out_dir, f"13_canny_dilated_{lo}_{hi}.png", dilated)

    # --- Stage 14: adaptive threshold (local, not global, cutoff) ---
    for block_size, c in ADAPTIVE_THRESH_CANDIDATES:
        adaptive = cv2.adaptiveThreshold(
            gray_blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,
            block_size, c)
        save(out_dir, f"14_adaptive_thresh_{block_size}_{c}.png", adaptive)

    # --- Stage 15: morphological close on a chosen stage (fills small gaps,
    # e.g. the 4 dark holes poking through an otherwise-solid disc mask,
    # without changing the outer boundary) ---
    if args.morph_close_stage:
        morph_source_path = os.path.join(out_dir, args.morph_close_stage + ".png")
        if os.path.exists(morph_source_path):
            mask = cv2.imread(morph_source_path, cv2.IMREAD_GRAYSCALE)
            kernel = np.ones((args.morph_close_kernel, args.morph_close_kernel), np.uint8)
            closed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            save(out_dir, f"15_morph_close_on_{args.morph_close_stage}.png", closed)
        else:
            print(
                f"\n--morph-close-stage '{args.morph_close_stage}' not found at {morph_source_path} — skipping stage 15."
            )
    else:
        print(
            "\nSkipping stage 15 (morph close) — pass --morph-close-stage <filename, no extension> "
            "once you've picked a promising stage from 12/13/14 to close gaps on."
        )

    # --- Stage 16: contours on whichever edge/adaptive/morph-close stage
    # looks best (same logic as stage 11, different candidate source) ---
    if args.contour_stage_2:
        run_contour_stage(out_dir, image, args.contour_stage_2, "16_contours_on")
    else:
        print(
            "\nSkipping stage 16 (second contour pass) — pass --contour-stage-2 "
            "<filename, no extension> once you've picked a promising edge/adaptive/morph-close stage."
        )

    # --- Stage 17: cv2.HoughCircles — direct circle detection, no
    # findContours/circularity-filter step needed. Compares against the
    # contour-based path on the same source image. ---
    hough_source_path = os.path.join(out_dir, args.hough_source + ".png")
    if os.path.exists(hough_source_path):
        hough_input = cv2.imread(hough_source_path, cv2.IMREAD_GRAYSCALE)
        circles = cv2.HoughCircles(
            hough_input, cv2.HOUGH_GRADIENT, dp=1, minDist=30,
            param1=100,  # internal Canny high threshold — Hough runs its own edge pass
            param2=args.hough_param2,  # accumulator threshold — see --hough-param2 help
            minRadius=args.hough_min_radius, maxRadius=args.hough_max_radius)

        overlay = image.copy()
        count = 0
        if circles is not None:
            for x, y, radius in np.round(circles[0]).astype(int):
                count += 1
                cv2.circle(overlay, (x, y), radius, (0, 255, 0), 2)
                cv2.circle(overlay, (x, y), 2, (0, 0, 255), 3)
                cv2.putText(
                    overlay, f"r={radius}", (x - 20, y - radius - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        save(out_dir, f"17_hough_circles_{args.hough_source}.png", overlay)
        print(f"\nHoughCircles on {args.hough_source}.png found {count} circle(s).")
        print(
            "Too many/noisy → raise --hough-param2. Missing the disc/holes → lower --hough-param2 "
            "or check --hough-min-radius/--hough-max-radius bound the shapes you want."
        )
    else:
        print(
            f"\n--hough-source '{args.hough_source}' not found at {hough_source_path} — skipping stage 17."
        )

    print("\nDone. Pull the output directory back and look through stages in order:")
    print("  01/02 vs 06 — does grayscale alone show the holes clearly darker than the disc?")
    print("  04/05 — do the holes show a clearly different hue/saturation than the disc and its shadow?")
    print("  07/08 — at which threshold N do the 4 holes appear as clean white blobs, with nothing else?")
    print("  09/10 — does a hue-only or full-HSV mask isolate the holes better than grayscale did?")
    print("  12/13 — does Canny show the disc's rim as a clean (or dilate-closeable) closed loop?")
    print("  14     — does adaptive threshold separate disc-from-wall better than a global cutoff did?")
    print("  15     — after morph-close, does the disc read as ONE solid blob (holes filled in)?")
    print("  16     — final check: does findContours on your best edge/adaptive stage fit a clean circle to the disc?")
    print("  17     — does HoughCircles find the disc + 4 holes directly, with fewer false positives than contours?")


def run_contour_stage(out_dir, original_image, source_stage_name, output_prefix):
    """Loads <out_dir>/<source_stage_name>.png as a binary mask, runs
    cv2.findContours, and draws every contour (area>=10) on a copy of
    original_image with its area/circularity annotated — green if
    circularity > 0.6 (would currently pass this project's
    hole_min_circularity/cup_holder_min_circularity defaults), red
    otherwise. Saves as <out_dir>/<output_prefix>_<source_stage_name>.png.
    No-ops with a print if source_stage_name isn't found — this script is
    meant to be re-run iteratively as you narrow down which stage to
    inspect, not to fail hard on a not-yet-chosen stage.
    """
    source_path = os.path.join(out_dir, source_stage_name + ".png")
    if not os.path.exists(source_path):
        print(
            f"\n'{source_stage_name}' not found at {source_path} — skipping {output_prefix} stage. "
            "Look through the other stages first, then re-run pointing at whichever filename looks cleanest."
        )
        return

    mask = cv2.imread(source_path, cv2.IMREAD_GRAYSCALE)
    contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    overlay = original_image.copy()
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
    save(out_dir, f"{output_prefix}_{source_stage_name}.png", overlay)
    print(f"\n{output_prefix}: found {len(contours)} raw contours (area>=10) on {source_stage_name}.png")
    print("Green circle = circularity > 0.6, red = would not currently pass this project's circularity filters.")


if __name__ == "__main__":
    main()