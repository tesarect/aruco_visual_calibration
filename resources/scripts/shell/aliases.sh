
# # Add this to you bashrc
# [ -f "$HOME/ros2_ws/src/visual_calibration/resources/scripts/shell/aliases.sh" ] && . "$HOME/ros2_ws/src/visual_calibration/resources/scripts/shell/aliases.sh"

# TMUX_CONF_SRC="$HOME/ros2_ws/src/visual_calibration/resources/scripts/tmux/tmux.conf"
# TMUX_CONF_DST="$HOME/.tmux.conf"

# #- copies only if not present at ~/
# if [ -f "$TMUX_CONF_SRC" ] && [ ! -f "$TMUX_CONF_DST" ]; then
#     cp "$TMUX_CONF_SRC" "$TMUX_CONF_DST"
# fi
# export NVM_DIR="$HOME/.nvm"
# [ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"  # This loads nvm
# [ -s "$NVM_DIR/bash_completion" ] && \. "$NVM_DIR/bash_completion"  # This loads nvm bash_completion
# =====================================================================================================

alias srcrc="source ~/.bashrc"
alias vcdir="cd ~/ros2_ws/src/visual_calibration/"
alias shdir="cd ~/ros2_ws/src/visual_calibration/resources/scripts/shell/"
alias tmuxdir="cd ~/ros2_ws/src/visual_calibration/resources/scripts/tmux/"
alias pydir="cd ~/ros2_ws/src/visual_calibration/resources/scripts/python/"
alias viewcam="ros2 run rqt_image_view rqt_image_view /wrist_rgbd_depth_sensor/image_raw"
alias viewoverlaycam="ros2 run rqt_image_view rqt_image_view /aruco_perception/overlay_image"

# Curses TUI: green/yellow/red status of every titled tmux pane across all
# sessions (base/trajcal/percep/webstack/...), cross-referenced against
# `ros2 node list` for hung detection. See node_dashboard.py's docstring.
alias nodedash="python3 ~/ros2_ws/src/visual_calibration/resources/scripts/python/node_dashboard.py"

alias installbase="bash ~/ros2_ws/src/visual_calibration/resources/scripts/shell/setup.sh"
alias installbasereal="bash ~/ros2_ws/src/visual_calibration/resources/scripts/shell/setup_real.sh"
alias installweb="bash ~/webpage_ws/setup_rosject.sh"
alias initweb="source ~/webpage_ws/scripts/session_init.sh"
alias statusweb="bash ~/webpage_ws/scripts/session_status.sh"

# Standalone Jenkins lifecycle — deliberately NOT part of installweb/
# tmuxwebstacksim. Jenkins is meant to be the thing that TRIGGERS sim/
# trajectory/Zenoh/etc. nodes itself via pipeline stages, so it must be
# startable on its own, independent of whether the web dashboard is ever
# touched this session (and vice versa — the dashboard must not require
# Jenkins to be up either). Idempotent — safe to call every session.
alias startjenkins="bash ~/ros2_ws/src/visual_calibration/resources/jenkins/install_jenkins.sh"

# Jenkins is launched via setsid (see install_jenkins.sh) specifically so
# it survives tmux/session teardown — it will NOT stop on its own, or when
# tmuxwebstacksim/any tmux session is killed. This is the only way to stop
# it. Same pkill pattern used everywhere else this session (install_
# jenkins.sh's own already-running check, stop_stale.sh) — not reinvented.
alias killjenkins="pkill -f 'java .*jenkins\.war' && echo 'Jenkins stopped.' || echo 'No Jenkins process found.'"

# Convenience only — brings up Jenkins AND the web dashboard together in
# one command, correct order (webpage_ws/start_all.sh). startjenkins and
# `cd webpage_ws/app && npm run start` still work independently on their
# own any time — this doesn't replace that, it's just the "I want both,
# right now" shortcut. Usage: startall sim   (or: startall real)
startall() {
    local env="${1:-sim}"
    bash ~/webpage_ws/start_all.sh --env "$env"
}

alias startrosbridge="ros2 launch rosbridge_server rosbridge_websocket_launch.xml"

# git
cleanpull() {
    git reset --hard HEAD
    git clean -fd
    git pull
}

killsim() {
    pkill gzclient*
}

customkill() {
    local key="$1"
    shift

    declare -A commands=(
        [gzclient]='pkill -f "^gzclient"'
        [basetmux]='tmux kill-session -t base_term'
        [baserealtmux]='tmux kill-session -t base_real_term'
        [trajcaltmux]='tmux kill-session -t trajcal_term'
        [trajcalrealtmux]='tmux kill-session -t trajcal_real_term'
        [yolotmux]='tmux kill-session -t yolo_term'
        [yolorealtmux]='tmux kill-session -t yolo_real_term'
        [perceptmux]='tmux kill-session -t percep_term'
        [webstacktmux]='tmux kill-session -t webstack_term'
        [webstackrealtmux]='tmux kill-session -t webstack_real_term'
        [debugtmux]='tmux kill-session -t debug_term'
        [gittmux]='tmux kill-session -t GIT'
        [termstmux]='tmux kill-session -t terminals'
    )

    # Remaining args (only meaningful with key="all") are exclude=<name>
    # tokens, e.g. `customkill all exclude=webstacktmux exclude=debugtmux`
    # — kills everything except the listed keys.
    declare -A excluded=()
    for arg in "$@"; do
        if [[ "$arg" =~ ^exclude=([a-zA-Z0-9_]+)$ ]]; then
            excluded["${BASH_REMATCH[1]}"]=1
        fi
    done

    if [[ "$key" == "all" ]]; then
        for target_key in "${!commands[@]}"; do
            if [[ -n "${excluded[$target_key]:-}" ]]; then
                echo "Skipping (excluded): $target_key"
                continue
            fi
            echo "Killing: $target_key"
            eval "${commands[$target_key]}"
        done
    elif [[ -n "${commands[$key]}" ]]; then
        eval "${commands[$key]}"
    else
        echo "Unknown target: $key"
        echo "Available: ${!commands[*]} all"
        return 1
    fi
}

# Per-pane logging is opt-in — pass <pane_name>=on args straight through,
# e.g. `tmuxbasesim move_group=on` or `tmuxbasesim move_group=on rviz=on`.
# See logging.sh for pane names per script and what gets captured/where.
# `tmuxbasesim move_group=on`
tmuxbasesim() {
    cd ~/ros2_ws/src/visual_calibration/resources/scripts/tmux/
    bash ./sim_tmux_base.sh "$@"
}

# Real robot equivalent of tmuxbasesim — Zenoh bridge, move_group, rviz,
# planning scene. Does NOT start the robot driver itself. Driver status
# check is opt-in: plain `tmuxbasereal` skips it and goes straight to
# tmux; pass `stat_check=on` to run a fail-fast check (stops at the first
# missing/silent thing, doesn't wait for a full report) before starting
# — e.g. `tmuxbasereal stat_check=on`. For the full report any time
# (mid-session too), use `realrobotstatuscheck` (see real_tmux_base.sh
# header).
tmuxbasereal() {
    cd ~/ros2_ws/src/visual_calibration/resources/scripts/tmux/
    bash ./real_tmux_base.sh "$@"
}

# tf_debug_markers.py + rqt overlay image + rqt_graph + a large scratch
# shell at ~/ros2_ws, in its own session — pulled out of tmuxbasesim/
# tmuxbasereal so this optional visual debugging isn't tied to the base
# session's lifecycle. Requires the matching base session already up.
# Usage: `tmuxdebug` (sim, default) or `tmuxdebug real`.
tmuxdebug() {
    cd ~/ros2_ws/src/visual_calibration/resources/scripts/tmux/
    bash ./debug_tmux.sh "$@"
}

# `tmuxtrajcalreal trajectory_planner=on calibration_broadcaster=on calibration_orchestrator=on`
tmuxtrajcalsim() {
    cd ~/ros2_ws/src/visual_calibration/resources/scripts/tmux/
    bash ./sim_tmux_trajcal.sh "$@"
}

# YOLO/hybrid-detector session, sim — inference_server.py +
# yolo_marker_bridge_node. Split out of tmuxtrajcalsim (2026-07-24,
# matching tmuxyoloreal's earlier split) to keep that session down to 4
# panes. Run `tmuxbasesim` first (needs move_group). Independent of
# tmuxtrajcalsim's own panes, but calibration_orchestrator_node (started
# by tmuxtrajcalsim) is what actually calls ~/set_detector_mode to switch
# into hybrid — start both sessions if you intend to test that switch.
# `tmuxyolosim inference_server=on yolo_marker_bridge=on`
tmuxyolosim() {
    cd ~/ros2_ws/src/visual_calibration/resources/scripts/tmux/
    bash ./sim_tmux_yolo.sh "$@"
}

# Real robot equivalent of tmuxtrajcalsim — trajectory_planner,
# aruco_detector_node, calibration_broadcaster_node, calibration_
# orchestrator_node. Run `tmuxbasereal` first — this session's panes poll
# for move_group + planning scene readiness, which come from there.
# NONE of this has been tested live yet (calibration_broadcaster_real.yaml
# was just added 2026-07-18) — expect to iterate.
# `tmuxtrajcalreal trajectory_planner=on calibration_broadcaster=on calibration_orchestrator=on`
tmuxtrajcalreal() {
    cd ~/ros2_ws/src/visual_calibration/resources/scripts/tmux/
    bash ./real_tmux_trajcal.sh "$@"
}

# YOLO/hybrid-detector session, real robot — inference_server.py +
# yolo_marker_bridge_node. Split out of tmuxtrajcalreal (2026-07-23) to
# keep that session down to 4 panes. Run `tmuxbasereal` first (needs
# move_group). Independent of tmuxtrajcalreal's own panes, but
# calibration_orchestrator_node (started by tmuxtrajcalreal) is what
# actually calls ~/set_detector_mode to switch into hybrid — start both
# sessions if you intend to test that switch.
# `tmuxyoloreal inference_server=on yolo_marker_bridge=on`
tmuxyoloreal() {
    cd ~/ros2_ws/src/visual_calibration/resources/scripts/tmux/
    bash ./real_tmux_yolo.sh "$@"
}

# `tmuxwebstacksim` defaults to extracting sim's URDF; pass `real` to
# extract real's instead, e.g. `tmuxwebstacksim real`.
tmuxwebstacksim() {
    cd ~/ros2_ws/src/visual_calibration/resources/scripts/tmux/
    bash ./sim_tmux_webstack.sh "$@"
}

tmuxpercepsim() {
    cd ~/ros2_ws/src/visual_calibration/resources/scripts/tmux/
    bash ./sim_tmux_percep.sh
}

tmuxgit() {
    cd ~/ros2_ws/src/visual_calibration/resources/scripts/tmux/
    bash ./tmuxsh.sh
}

tmuxterms() {
    cd ~/ros2_ws/src/visual_calibration/resources/scripts/tmux/
    bash ./misl_terms.sh
}

startsim() {
    source ~/ros2_ws/install/setup.bash
    ros2 launch the_construct_office_gazebo starbots_ur3e.launch.xml
}

startmoveitconfig() {
    source /home/simulations/ros2_sims_ws/install/setup.bash
    ros2 launch moveit_setup_assistant setup_assistant.launch.py
}

# env: sim -> sim_ur3e_moveit_config (project-owned, joint_trajectory_controller)
#      real -> real_ur3e_moveit_config (project-owned, scaled_joint_trajectory_controller)
# Both are project-owned copies under visual_calibration/, NOT
# universal_robot_ros2/ur3e_moveit_config directly — that instructor
# package's moveit_controllers.yaml was found to have drifted to
# joint_trajectory_controller (sim's controller name) at some point;
# real_ur3e_moveit_config preserves the correct scaled_ version. See
# sim_ur3e_moveit_config/package.xml's description for the same reasoning
# on the sim side.
startmoveitgroup() {
    local env="${1:-sim}"
    local pkg="sim_ur3e_moveit_config"
    [[ "$env" == "real" ]] && pkg="real_ur3e_moveit_config"
    source ~/ros2_ws/install/setup.bash
    ros2 launch "$pkg" move_group.launch.py
}

startrviz() {
    local env="${1:-sim}"
    local pkg="sim_ur3e_moveit_config"
    [[ "$env" == "real" ]] && pkg="real_ur3e_moveit_config"
    source ~/ros2_ws/install/setup.bash
    ros2 launch "$pkg" moveit_rviz.launch.py
}

viewtf() {
    ros2 run tf2_tools view_frames
    # ros2 run rqt_tf_tree rqt_tf_tree
}

viewtfbtw() {
    local source_frame="${1}"
    local target_frame="${2}"
    ros2 run tf2_ros tf2_echo "$source_frame" "$target_frame"
}

startplanningscene() {
    local env="${1:-sim}"
    source ~/ros2_ws/install/setup.bash
    ros2 launch visual_calibration_moveit planning_scene_setup.launch.py env:="$env"
}

starttrajectoryplanner() {
    local env="${1:-sim}"
    source ~/ros2_ws/install/setup.bash
    ros2 launch visual_calibration_moveit trajectory_planner.launch.py env:="$env"
}

startarucodetector() {
    local env="${1:-sim}"
    source ~/ros2_ws/install/setup.bash
    ros2 run aruco_perception aruco_detector_node --ros-args \
        --params-file ~/ros2_ws/src/visual_calibration/aruco_perception/config/aruco_detector_"$env".yaml
}

# inference_server.py — the always-on YOLO model server (plain Flask
# process inside ~/yolo_venv, NOT a ROS node — never call this from a shell
# that also sources ROS's setup.bash and imports cv_bridge, see the
# ABI-isolation rule in error-mitigation.md #15). Backgrounds itself and
# returns immediately; pair with waitforinferenceserver to block until
# ready. See aruco_perception_yolo_bridge/resources/scripts/shell/
# start_inference_server.sh for what this actually does.
startinferenceserver() {
    local env="${1:-real}"
    bash ~/ros2_ws/src/visual_calibration/aruco_perception_yolo_bridge/resources/scripts/shell/start_inference_server.sh "$env"
}

waitforinferenceserver() {
    local timeout="${1:-30}"
    local expected_env="${2:-}"
    bash ~/ros2_ws/src/visual_calibration/aruco_perception_yolo_bridge/resources/scripts/shell/wait_for_inference_server.sh "$timeout" "$expected_env"
}

# yolo_marker_bridge_node — the YOLO-backed alternative to
# startarucodetector, for the classical/hybrid switch (see
# calibration_orchestrator_node's ~/set_detector_mode). This IS a normal
# ROS node (rclpy, imports cv_bridge) despite calling out to
# inference_server.py over HTTP — safe to run from a plain ROS shell, same
# as startarucodetector. Requires inference_server.py already running
# (startinferenceserver) — it does not start that itself.
startyolomarkerbridge() {
    local env="${1:-sim}"
    source ~/ros2_ws/install/setup.bash
    ros2 run aruco_perception_yolo_bridge yolo_marker_bridge_node.py --ros-args \
        --params-file ~/ros2_ws/src/visual_calibration/aruco_perception_yolo_bridge/config/yolo_marker_bridge_"$env".yaml
}

# Switches which detector actively publishes /aruco_perception/marker_pose
# — "classical" (aruco_detector_node) or "hybrid" (yolo_marker_bridge_node).
# See calibration_orchestrator_node's handleSetDetectorMode and
# visual_calibration_msgs/srv/SetDetectorMode.srv. Both detector nodes must
# already be running (aruco_detector_node always is by default;
# yolo_marker_bridge_node needs startyolomarkerbridge — and
# startinferenceserver before that — run first) or this call fails cleanly
# (response.success=false), it does not start either node itself.
setdetectormode() {
    local mode="${1:?Usage: setdetectormode <classical|hybrid>}"
    source ~/ros2_ws/install/setup.bash
    ros2 service call /calibration_orchestrator_node/set_detector_mode \
        visual_calibration_msgs/srv/SetDetectorMode "{mode: '$mode'}"
}

startcalibrationbroadcaster() {
    local env="${1:-sim}"
    source ~/ros2_ws/install/setup.bash
    ros2 run aruco_perception calibration_broadcaster_node --ros-args \
        --params-file ~/ros2_ws/src/visual_calibration/aruco_perception/config/calibration_broadcaster_"$env".yaml
}

# calibration_orchestrator_node: chains cal_ready -> optional auto-center ->
# ~/calibrate into one node/action, see orchestrator package. Started by
# both sim_tmux_trajcal.sh and real_tmux_trajcal.sh.
startcalibrationorchestrator() {
    local env="${1:-sim}"
    source ~/ros2_ws/install/setup.bash
    ros2 run orchestrator calibration_orchestrator_node --ros-args \
        --params-file ~/ros2_ws/src/visual_calibration/orchestrator/config/calibration_orchestrator_"$env".yaml
}

# Moves through trajectory_planner's whole polygon in one call — useful
# standalone (e.g. to sanity-check the arm actually moves/reaches all
# corners) but NOT used by calibration anymore: calibration_broadcaster_node
# now orchestrates trajectory_planner itself, one waypoint at a time, via
# ~/get_polygon_waypoints + ~/trace_path (see calibration_broadcaster_node.hpp) —
# so trace_polygon's single blocking call (no pause between corners for
# sampling) is no longer in that loop.
tracepolygon() {
    ros2 service call /trajectory_planner/trace_polygon std_srvs/srv/Trigger {}
}

# Read-only: computes and returns the standoff pose (in front of
# camera_frame) WITHOUT moving the arm — trajectory_planner no longer
# auto-moves to standoff on startup (see todo.txt item 1). Falls back to
# the "standoff" preset (preset_poses_sim.yaml/_real.yaml) if the camera
# TF isn't available — check the response's used_fallback field to see
# which source was used.
getstandoffpose() {
    ros2 service call /trajectory_planner/get_standoff_pose \
        visual_calibration_msgs/srv/GetStandoffPose {}
}

# Read-only: returns a named preset pose (e.g. "standoff") WITHOUT moving
# the arm — see preset_poses_sim.yaml/_real.yaml. Usage: getpresetpose standoff
getpresetpose() {
    local name="${1:?Usage: getpresetpose <preset_name>}"
    ros2 service call /trajectory_planner/get_preset_pose \
        visual_calibration_msgs/srv/GetPresetPose "{name: '$name'}"
}

# Moves the arm to a NAMED preset (e.g. "home", "standby",
# "baristastandby") — looks the pose up via ~/get_preset_pose then sends
# it to ~/trace_path, so it always matches whatever's actually recorded in
# preset_poses_sim.yaml/_real.yaml (see move_to_preset.py). Joint-space by
# default; pass "cartesian" as the 2nd arg for a straight-line move
# instead. Usage: movetopreset baristastandby
#                 movetopreset baristastandby cartesian
movetopreset() {
    local name="${1:?Usage: movetopreset <preset_name> [joint_space|cartesian]}"
    local mode="${2:-joint_space}"
    local mode_flag="--joint-space"
    [[ "$mode" == "cartesian" ]] && mode_flag="--cartesian"
    source ~/ros2_ws/install/setup.bash
    python3 ~/ros2_ws/src/visual_calibration/resources/scripts/python/move_to_preset.py \
        "$name" "$mode_flag"
}

# Records the current tool0 position (base_link-relative) as one corner of
# a planning-scene box object being measured — see measure_scene_box.py.
# Jog the arm's end-effector to touch a corner via RViz first, THEN call
# this (reads live TF only, triggers no motion). Call twice per object
# (once per opposite corner), appending to the same --out file.
measurecorner() {
    local out="${1:?Usage: measurecorner <out_file.txt>}"
    python3 ~/ros2_ws/src/visual_calibration/resources/scripts/python/measure_scene_box.py \
        corner --out "$out"
}

# Computes a box's center pose + size from 2 recorded corners (see
# measurecorner) and prints scene_objects_real.yaml-ready lines for the
# given object name (e.g. coffee_machine, cupholder). Usage:
#   measurecompute ~/coffee_machine_corners.txt coffee_machine
measurecompute() {
    local in_file="${1:?Usage: measurecompute <in_file.txt> <object_name>}"
    local name="${2:?Usage: measurecompute <in_file.txt> <object_name>}"
    python3 ~/ros2_ws/src/visual_calibration/resources/scripts/python/measure_scene_box.py \
        compute --in "$in_file" --name "$name"
}

# Sends the ~/calibrate action goal and blocks, printing live feedback
# (samples_collected/samples_total) until the action completes — includes
# the final orientation spread (max/mean degrees) in the result.
# calibration_broadcaster_node drives the whole sequence itself (fetch
# waypoints, move one at a time, wait for the arm to settle, sample) — no
# separate polygon-tracing loop needed alongside this anymore. Assumes the
# arm is ALREADY at cal_ready/standoff (e.g. via getstandoffpose + a manual
# trace_path, or startautocalibration below) — does not move there itself.
startcalibration() {
    ros2 action send_goal /calibration_broadcaster_node/calibrate \
        visual_calibration_msgs/action/Calibrate {} --feedback
}

# Sends the ~/auto_calibrate action goal and blocks, printing live feedback
# (stage name, then samples_collected/samples_total once the calibrate
# stage starts) until the action completes. calibration_orchestrator_node
# drives the WHOLE sequence: moves to cal_ready itself (no need to call
# getstandoffpose/trace_path first), optionally auto-centers on the marker
# (see calibration_orchestrator_sim/real.yaml's auto_center_enabled), then
# calls calibration_broadcaster_node's ~/calibrate as a client. Prefer this
# over startcalibration for a from-scratch run — startcalibration alone
# still works for testing calibration in isolation once already at
# cal_ready.
startautocalibration() {
    ros2 action send_goal /calibration_orchestrator_node/auto_calibrate \
        visual_calibration_msgs/action/AutoCalibrate {} --feedback
}

# Sim-only accuracy check: compares calibration_broadcaster_node's
# broadcast TF against sim's own ground-truth camera TF. Run once
# startcalibration has finished (a static TF, once broadcast, stays in
# the tree — no need to re-run calibration first if it already succeeded).
validatecalibrationsim() {
    source ~/ros2_ws/install/setup.bash
    ros2 run calibration_validation validate_calibration_sim.py
}

# Isolated YOLO venv management — see install_yolo.sh/remove_yolo.sh for
# what these actually do and why the venv is kept fully separate from
# ROS's system Python (cv_bridge/OpenCV 4.5.4 ABI conflict risk).
installyolo() {
    bash ~/ros2_ws/src/visual_calibration/resources/scripts/shell/install_yolo.sh
}

removeyolo() {
    bash ~/ros2_ws/src/visual_calibration/resources/scripts/shell/remove_yolo.sh
}

# Activates the YOLO venv in the current shell. Do not run ROS nodes that
# import cv_bridge in a shell where this is active.
yoloenv() {
    if [ ! -d "$HOME/yolo_venv" ]; then
        echo "No YOLO venv found — run installyolo first."
        return 1
    fi
    # shellcheck disable=SC1091
    source "$HOME/yolo_venv/bin/activate"
}

vcpkgbuild() {
    local pkg="${1}"
    cd ~/ros2_ws || return
    colcon build --packages-up-to "$pkg"
    source install/setup.bash
}

vcpkgbuildsymlink() {
    local pkg="${1}"
    cd ~/ros2_ws || return
    colcon build --packages-up-to "$pkg" --symlink-install
    source install/setup.bash
}

vcbuild() {
    cd ~/ros2_ws || return
    # colcon build --packages-up-to aruco_moveit_config visual_calibration_msgs visual_calibration_moveit aruco_perception orchestrator calibration_validation real_ur3e_description
    colcon build --packages-up-to sim_ur3e_moveit_config real_ur3e_moveit_config visual_calibration_msgs visual_calibration_moveit aruco_perception aruco_perception_yolo_bridge depth_perception orchestrator calibration_validation real_ur3e_description robotiq_85_msgs
    source install/setup.bash
}

vcbuildsymlink() {
    cd ~/ros2_ws || return
    # colcon build --packages-up-to aruco_moveit_config visual_calibration_msgs visual_calibration_moveit aruco_perception orchestrator calibration_validation real_ur3e_description --symlink-install
    colcon build --packages-up-to sim_ur3e_moveit_config real_ur3e_moveit_config visual_calibration_msgs visual_calibration_moveit aruco_perception aruco_perception_yolo_bridge depth_perception orchestrator calibration_validation real_ur3e_description robotiq_85_msgs --symlink-install
    source install/setup.bash
}

vccleanbuild() {
    cd ~/ros2_ws || return

    rm -rf build/sim_ur3e_moveit_config \
        build/real_ur3e_moveit_config \
        build/visual_calibration_msgs \
        build/visual_calibration_moveit \
        build/aruco_perception \
        build/aruco_perception_yolo_bridge \
        build/depth_perception \
        build/orchestrator \
        build/calibration_validation \
        build/real_ur3e_description \
        build/robotiq_85_msgs
    rm -rf install/sim_ur3e_moveit_config \
        install/real_ur3e_moveit_config \
        install/visual_calibration_msgs \
        install/visual_calibration_moveit \
        install/aruco_perception \
        install/aruco_perception_yolo_bridge \
        install/depth_perception \
        install/orchestrator \
        install/calibration_validation \
        install/real_ur3e_description \
        install/robotiq_85_msgs

    colcon build --packages-up-to sim_ur3e_moveit_config real_ur3e_moveit_config visual_calibration_msgs visual_calibration_moveit aruco_perception aruco_perception_yolo_bridge depth_perception orchestrator calibration_validation real_ur3e_description robotiq_85_msgs
    source install/setup.bash
}

vccleanbuildsymlink() {
    cd ~/ros2_ws || return

    rm -rf build/sim_ur3e_moveit_config \
        build/real_ur3e_moveit_config \
        build/visual_calibration_msgs \
        build/visual_calibration_moveit \
        build/aruco_perception \
        build/aruco_perception_yolo_bridge \
        build/depth_perception \
        build/orchestrator \
        build/calibration_validation \
        build/real_ur3e_description \
        build/robotiq_85_msgs
    rm -rf install/sim_ur3e_moveit_config \
        install/real_ur3e_moveit_config \
        install/visual_calibration_msgs \
        install/visual_calibration_moveit \
        install/aruco_perception \
        install/aruco_perception_yolo_bridge \
        install/depth_perception \
        install/orchestrator \
        install/calibration_validation \
        install/real_ur3e_description \
        install/robotiq_85_msgs

    colcon build --packages-up-to sim_ur3e_moveit_config real_ur3e_moveit_config visual_calibration_msgs visual_calibration_moveit aruco_perception aruco_perception_yolo_bridge depth_perception orchestrator calibration_validation real_ur3e_description robotiq_85_msgs --symlink-install
    source install/setup.bash
}

allcleanbuild() {
    cd ~/ros2_ws || return

    rm -rf build/
    rm -rf install/
    rm -rf log/

    colcon build
    source install/setup.bash
}

cleanlogs() {
    cd ~/ros2_ws || return
    
    rm -rf log/
}

# Ensures every script under resources/scripts/{shell,tmux,python} is
# executable. Needed because a plain file copy (cp -r / rsync without -p,
# scp, drag-and-drop in some editors) does not preserve the executable
# bit — this bit us repeatedly with tmux/python scripts run as
# `./script.sh` or invoked directly by other scripts (not via `bash
# script.sh`, which works regardless of the bit). Safe to run anytime,
# idempotent, and called automatically by completesimsetup/completerealsetup
# so a fresh rosject copy never needs a manual chmod pass again.
fixscriptperms() {
    find ~/ros2_ws/src/visual_calibration/resources/scripts \
        \( -name "*.sh" -o -name "*.py" \) -exec chmod +x {} +
    echo "🛂🔑-Fixed executable permissions under resources/scripts/"
}

completesimsetup() {
    fixscriptperms
    # installbase
    bash ~/ros2_ws/src/visual_calibration/resources/scripts/shell/setup.sh
    # installweb (--env required by setup_rosject.sh)
    bash ~/webpage_ws/setup_rosject.sh --env sim
    # initweb
    source ~/webpage_ws/scripts/session_init.sh
    # statusweb
    bash ~/webpage_ws/scripts/session_status.sh
    # # install yolo
    # sudo apt install -y python3.10-venv
    # bash ~/ros2_ws/src/visual_calibration/resources/scripts/shell/install_yolo.sh
}

completerealsetup() {
    fixscriptperms
    # installbasereal
    bash ~/ros2_ws/src/visual_calibration/resources/scripts/shell/setup_real.sh
    # installweb (--env required by setup_rosject.sh)
    bash ~/webpage_ws/setup_rosject.sh --env real
    # initweb
    source ~/webpage_ws/scripts/session_init.sh
    # statusweb
    bash ~/webpage_ws/scripts/session_status.sh
    # Install zehno - camera driver - done by setup_real.sh
    # install yolo
    sudo apt install -y python3.10-venv
    bash ~/ros2_ws/src/visual_calibration/resources/scripts/shell/install_yolo.sh
}

webstatuscheck() {
    cd ~/webpage_ws/scripts/
    bash session_status.sh
}

srcweb() {
    cd ~/webpage_ws/scripts/
    source session_init.sh
}

realrobotstatuscheck(){
    bash ~/ros2_ws/src/visual_calibration/resources/scripts/shell/check_real_driver.sh
}

# Checks scaled_joint_trajectory_controller's state on the real robot's
# controller_manager and activates it if it dropped to inactive (seen
# happening intermittently on real — root cause not yet diagnosed). Safe
# to run repeatedly: no-ops if already active. Requires controller_manager
# to be up (i.e. the real robot driver running) — run realrobotstatuscheck
# first if unsure. Wraps ensure_controller_active.sh, which takes any
# controller_manager/controller_name pair — call that directly for other
# controllers (e.g. gripper_controller) if needed.
ensurerealcontroller() {
    source ~/ros2_ws/install/setup.bash
    bash ~/ros2_ws/src/visual_calibration/resources/scripts/shell/ensure_controller_active.sh \
        /controller_manager scaled_joint_trajectory_controller
}

shadcnadd() {
    cd ~/webpage_ws/app
    npx shadcn@latest add input
    npm install
    npm run start
}