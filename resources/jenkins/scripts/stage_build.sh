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
# BUILD_MODE (env var, set by the Jenkinsfile from its BUILD_MODE choice
# parameter — dev|deploy, default dev): "dev" runs an incremental
# --symlink-install build (matches vcbuildsymlink — fast re-runs, for
# iterating on the pipeline itself); "deploy" wipes build/install for the
# packages above first (matches vccleanbuildsymlink) then builds clean —
# for actual "fresh, verified, presentation-ready" runs. Defaults to "dev"
# if unset so this script still works standalone (e.g. run by hand outside
# Jenkins) without needing the var exported first.
BUILD_MODE="${BUILD_MODE:-dev}"

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

if [ "$BUILD_MODE" = "deploy" ]; then
    echo "=== [stage_build] BUILD_MODE=deploy — wiping build/install for the package list first (matches vccleanbuildsymlink) ==="
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
elif [ "$BUILD_MODE" != "dev" ]; then
    echo "[stage_build] Unknown BUILD_MODE '$BUILD_MODE' (expected dev|deploy) — failing stage."
    exit 1
fi

echo "=== [stage_build] BUILD_MODE=$BUILD_MODE — colcon build --packages-up-to ... --symlink-install (see header for full list) ==="
# shellcheck disable=SC2086
colcon build --packages-up-to $PACKAGES --symlink-install \
    > "$BUILD_LOG" 2>&1
BUILD_STATUS=$?

if [ "$BUILD_STATUS" -ne 0 ]; then
    echo "[stage_build] colcon build FAILED (exit $BUILD_STATUS) — see build_colcon.log — failing stage."
    exit 1
fi

echo "=== [stage_build] Build stage complete ==="
