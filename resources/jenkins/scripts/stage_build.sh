#!/bin/bash
# Jenkins "Build" stage — colcon build, package list mirrors aliases.sh's
# vcbuild()/vcbuildsymlink() EXACTLY (read fresh from aliases.sh each time
# this stage script is touched — that list has grown over the session, do
# not hardcode a stale copy from memory or from an older doc). Those
# aliases are effectively this project's existing "semi-pipeline" — this
# stage formalizes the same sequence with logging/archiving, not a new one.
#
# As of this writing (verified against aliases.sh 2026-07-24):
#   sim_ur3e_moveit_config real_ur3e_moveit_config visual_calibration_msgs
#   visual_calibration_moveit aruco_perception aruco_perception_yolo_bridge
#   depth_perception orchestrator calibration_validation
#   real_ur3e_description robotiq_85_msgs
#
# depth_perception IS included in the build list (matches vcbuild) even
# though it is NOT part of the bringup_full_*/orchestrator-pipeline chain
# and gets no dedicated Jenkins stage below (see stage_orchestrator_pipeline.sh's
# header and the Jenkinsfiles) — it builds because vcbuild builds it, same
# as any other package in that list; whether to also RUN it is a separate,
# still-open question flagged to the user, not decided here.
#
# BUILD (env var, set by the Jenkinsfile from its BUILD choice parameter —
# complete_build|vcpkgs_build|none, default vcpkgs_build): three distinct
# modes, added after a move_group startup failure in the Base(sim) stage
# raised the question of whether a stale/partial build was the real root
# cause (rather than a readiness-check timing issue) —
#   complete_build — matches aliases.sh's allcleanbuild(): wipes the
#     ENTIRE build/, install/, and log/ directories, then a plain
#     `colcon build` (whole workspace, no --packages-up-to, no
#     --symlink-install). Slowest, most thorough — rules out ANY stale
#     artifact anywhere in the workspace, not just this project's own
#     packages.
#   vcpkgs_build — matches aliases.sh's vccleanbuildsymlink(): wipes
#     build/install for ONLY the package list above, then
#     `colcon build --packages-up-to ... --symlink-install`. Faster than
#     complete_build (doesn't touch instructor-provided/vendored
#     packages' existing build artifacts), still guarantees a fresh build
#     of everything this project actually owns.
#   none — skips colcon build entirely, assumes `~/ros2_ws/install`
#     already has a working build from a prior run. Fastest — for
#     re-testing a LATER stage (e.g. Base (sim)) without re-touching the
#     build at all.
BUILD="${BUILD:-vcpkgs_build}"

set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=./pipeline_common.sh
source "$SCRIPT_DIR/pipeline_common.sh"

BUILD_LOG="$LOG_DIR/build_colcon.log"

cd "$HOME/ros2_ws" || { echo "[stage_build] ~/ros2_ws not found — failing stage."; exit 1; }

PACKAGES="sim_ur3e_moveit_config real_ur3e_moveit_config visual_calibration_msgs \
    visual_calibration_moveit aruco_perception aruco_perception_yolo_bridge \
    depth_perception orchestrator calibration_validation real_ur3e_description \
    robotiq_85_msgs"

case "$BUILD" in
  none)
    echo "=== [stage_build] BUILD=none — skipping colcon build entirely (assumes ~/ros2_ws/install is already built) ==="
    : > "$BUILD_LOG"
    echo "=== [stage_build] Build stage complete (skipped) ==="
    exit 0
    ;;

  complete_build)
    echo "=== [stage_build] BUILD=complete_build — wiping build/, install/, log/ entirely (matches allcleanbuild) ==="
    rm -rf build/ install/ log/
    echo "=== [stage_build] colcon build (whole workspace, no --packages-up-to, no --symlink-install) ==="
    colcon build \
        > "$BUILD_LOG" 2>&1
    BUILD_STATUS=$?
    ;;

  vcpkgs_build)
    echo "=== [stage_build] BUILD=vcpkgs_build — wiping build/install for the package list only (matches vccleanbuildsymlink) ==="
    rm -rf build/sim_ur3e_moveit_config build/real_ur3e_moveit_config \
        build/visual_calibration_msgs build/visual_calibration_moveit \
        build/aruco_perception build/aruco_perception_yolo_bridge \
        build/depth_perception build/orchestrator build/calibration_validation \
        build/real_ur3e_description build/robotiq_85_msgs
    rm -rf install/sim_ur3e_moveit_config install/real_ur3e_moveit_config \
        install/visual_calibration_msgs install/visual_calibration_moveit \
        install/aruco_perception install/aruco_perception_yolo_bridge \
        install/depth_perception install/orchestrator install/calibration_validation \
        install/real_ur3e_description install/robotiq_85_msgs
    echo "=== [stage_build] colcon build --packages-up-to ... --symlink-install (see header for full list) ==="
    # shellcheck disable=SC2086
    colcon build --packages-up-to $PACKAGES --symlink-install \
        > "$BUILD_LOG" 2>&1
    BUILD_STATUS=$?
    ;;

  *)
    echo "[stage_build] Unknown BUILD '$BUILD' (expected complete_build|vcpkgs_build|none) — failing stage."
    exit 1
    ;;
esac

if [ "$BUILD_STATUS" -ne 0 ]; then
    echo "[stage_build] colcon build FAILED (exit $BUILD_STATUS) — see build_colcon.log — failing stage."
    exit 1
fi

echo "=== [stage_build] Build stage complete ==="
