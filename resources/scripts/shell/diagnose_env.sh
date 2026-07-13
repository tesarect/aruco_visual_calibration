#!/bin/bash
# Dumps everything needed to compare sim vs. real before writing/fixing
# the real-robot YAML configs (aruco_detector_real.yaml,
# calibration_broadcaster_real.yaml, trajectory_planner_real.yaml,
# scene_objects_real.yaml — see progress.md/error-mitigation.md for why
# these are still placeholders).
#
# Run this on BOTH sim and real, paste both outputs back for comparison.
#
# Usage:
#   bash diagnose_env.sh sim    # run against the running sim stack
#   bash diagnose_env.sh real   # run against the real robot (Zenoh bridge
#                                # must already be up, ROS_DOMAIN_ID=1 set,
#                                # CYCLONEDDS_URI unset — see CLAUDE.md)
#
# Output: a single timestamped text file in ~/diagnostics/, safe to
# attach/paste back.

set -o pipefail  # not -e: individual ros2 commands may time out/fail and
                  # we want the script to keep going and record that fact

ENV="${1:-}"
if [[ "$ENV" != "sim" && "$ENV" != "real" ]]; then
    echo "Usage: bash diagnose_env.sh [sim|real]"
    exit 1
fi

OUT_DIR="$HOME/diagnostics"
mkdir -p "$OUT_DIR"
STAMP=$(date +%Y%m%d_%H%M%S)
OUT_FILE="$OUT_DIR/diagnose_${ENV}_${STAMP}.txt"

# Sourced before -u is enabled: colcon's generated setup.bash references
# conditionally-unset vars (e.g. COLCON_TRACE) and isn't nounset-safe.
# shellcheck disable=SC1090
source ~/ros2_ws/install/setup.bash
set -u

{
    echo "=================================================================="
    echo "Environment: $ENV"
    echo "Timestamp: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
    echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>}"
    echo "CYCLONEDDS_URI=${CYCLONEDDS_URI:-<unset>}"
    echo "=================================================================="

    echo ""
    echo "--- ros2 topic list -t ---"
    timeout 10 ros2 topic list -t 2>&1

    echo ""
    echo "--- ros2 node list ---"
    timeout 10 ros2 node list 2>&1

    echo ""
    echo "--- ros2 service list ---"
    timeout 10 ros2 service list 2>&1

    echo ""
    echo "--- ros2 action list ---"
    timeout 10 ros2 action list 2>&1

    echo ""
    echo "--- Camera-related topics: type + one message sample each ---"
    CAMERA_TOPICS=$(timeout 10 ros2 topic list 2>/dev/null | grep -iE "image|camera|depth|color|d415|rgbd")
    if [ -z "$CAMERA_TOPICS" ]; then
        echo "(no camera-related topics found)"
    else
        for t in $CAMERA_TOPICS; do
            echo ""
            echo "  Topic: $t"
            echo "  Type:"
            timeout 5 ros2 topic type "$t" 2>&1 | sed 's/^/    /'
            echo "  Hz (3s sample):"
            timeout 5 ros2 topic hz "$t" --window 20 2>&1 | tail -5 | sed 's/^/    /'
        done
    fi

    echo ""
    echo "--- camera_info sample (first camera_info topic found, if any) ---"
    CI_TOPIC=$(timeout 10 ros2 topic list 2>/dev/null | grep -i "camera_info" | head -1)
    if [ -n "$CI_TOPIC" ]; then
        echo "  Topic: $CI_TOPIC"
        timeout 5 ros2 topic echo "$CI_TOPIC" --once 2>&1
    else
        echo "(no camera_info topic found)"
    fi

    echo ""
    echo "--- /joint_states sample ---"
    timeout 5 ros2 topic echo /joint_states --once 2>&1

    echo ""
    echo "--- ros2 control list_controllers ---"
    timeout 10 ros2 control list_controllers 2>&1

    echo ""
    echo "--- Full TF tree (tf2_ros tf2_echo requires two frames; instead dump raw /tf_static + /tf snapshot) ---"
    echo "  /tf_static (one message):"
    timeout 5 ros2 topic echo /tf_static --once 2>&1
    echo ""
    echo "  /tf (one message, live transforms):"
    timeout 5 ros2 topic echo /tf --once 2>&1

    echo ""
    echo "--- tf2_tools view_frames (generates frames.pdf/frames.gv in CWD) ---"
    (cd "$OUT_DIR" && timeout 15 ros2 run tf2_tools view_frames 2>&1)
    echo "  (if successful, see $OUT_DIR/frames.pdf / frames.gv)"

    echo ""
    echo "--- known base_link -> candidate camera frame lookups ---"
    for target in wrist_rgbd_camera_depth_optical_frame camera_depth_optical_frame camera_color_optical_frame D415_color_optical_frame; do
        echo "  base_link -> $target:"
        timeout 3 ros2 run tf2_ros tf2_echo base_link "$target" --once 2>&1 | head -10 | sed 's/^/    /'
        echo ""
    done

    echo ""
    echo "--- rg2_gripper_aruco_link presence check ---"
    timeout 3 ros2 run tf2_ros tf2_echo base_link rg2_gripper_aruco_link --once 2>&1 | head -10

    echo ""
    echo "--- installed aruco_perception / visual_calibration_moveit params files present ---"
    ls -la ~/ros2_ws/src/visual_calibration/aruco_perception/config/ 2>&1
    ls -la ~/ros2_ws/src/visual_calibration/visual_calibration_moveit/config/ 2>&1

    echo ""
    echo "=================================================================="
    echo "END OF REPORT"
    echo "=================================================================="

} > "$OUT_FILE" 2>&1

echo "Diagnostics saved to: $OUT_FILE"
echo "Paste this file's contents back for comparison."