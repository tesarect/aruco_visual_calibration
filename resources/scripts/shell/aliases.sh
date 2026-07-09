
# Add this to you bashrc
# [ -f "$HOME/ros2_ws/src/visual_calibration/resources/scripts/shell/aliases.sh" ] && . "$HOME/ros2_ws/src/visual_calibration/resources/scripts/shell/aliases.sh"
#               or this
# grep -qxF '[ -f "$HOME/ros2_ws/src/visual_calibration/resources/scripts/shell/aliases.sh" ] && . "$HOME/ros2_ws/src/visual_calibration/resources/scripts/shell/aliases.sh"' ~/.bashrc ||
# echo '[ -f "$HOME/ros2_ws/src/visual_calibration/resources/scripts/shell/aliases.sh" ] && . "$HOME/ros2_ws/src/visual_calibration/resources/scripts/shell/aliases.sh"' >> ~/.bashrc

alias srcrc="source ~/.bashrc"
alias vcdir="cd ~/ros2_ws/src/visual_calibration/"
alias shdir="cd ~/ros2_ws/src/visual_calibration/resources/scripts/shell/"
alias tmuxdir="cd ~/ros2_ws/src/visual_calibration/resources/scripts/tmux/"
alias pydir="cd ~/ros2_ws/src/visual_calibration/resources/scripts/python/"
alias viewcam="ros2 run rqt_image_view rqt_image_view /wrist_rgbd_depth_sensor/image_raw"
alias viewoverlaycam="ros2 run rqt_image_view rqt_image_view /aruco_perception/overlay_image"

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

    declare -A commands=(
        [gzclient]='pkill -f "^gzclient"'
        [basetmux]='tmux kill-session -t base_term'
        [main1tmux]='tmux kill-session -t main1_term'
    )

    if [[ "$key" == "all" ]]; then
        for target_key in "${!commands[@]}"; do
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

tmuxbasesim() {
    cd ~/ros2_ws/src/visual_calibration/resources/scripts/tmux/
    bash ./sim_tmux_base.sh
}

tmuxmain1sim() {
    cd ~/ros2_ws/src/visual_calibration/resources/scripts/tmux/
    bash ./sim_tmux_main1.sh
}

startsim() {
    source ~/ros2_ws/install/setup.bash
    ros2 launch the_construct_office_gazebo starbots_ur3e.launch.xml
}

startmoveitconfig() {
    source /home/simulations/ros2_sims_ws/install/setup.bash
    ros2 launch moveit_setup_assistant setup_assistant.launch.py
}

startmoveitgroup() {
    source ~/ros2_ws/install/setup.bash
    ros2 launch aruco_moveit_config move_group.launch.py
}

startrviz() {
    source ~/ros2_ws/install/setup.bash
    ros2 launch aruco_moveit_config moveit_rviz.launch.py
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

startcalibrationbroadcaster() {
    local env="${1:-sim}"
    source ~/ros2_ws/install/setup.bash
    ros2 run aruco_perception calibration_broadcaster_node --ros-args \
        --params-file ~/ros2_ws/src/visual_calibration/aruco_perception/config/calibration_broadcaster_"$env".yaml
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

# Sends the ~/calibrate action goal and blocks, printing live feedback
# (samples_collected/samples_total) until the action completes — includes
# the final orientation spread (max/mean degrees) in the result.
# calibration_broadcaster_node drives the whole sequence itself (fetch
# waypoints, move one at a time, wait for the arm to settle, sample) — no
# separate polygon-tracing loop needed alongside this anymore.
startcalibration() {
    ros2 action send_goal /calibration_broadcaster_node/calibrate \
        visual_calibration_msgs/action/Calibrate {} --feedback
}

# Sim-only accuracy check: compares calibration_broadcaster_node's
# broadcast TF against sim's own ground-truth camera TF. Run once
# startcalibration has finished (a static TF, once broadcast, stays in
# the tree — no need to re-run calibration first if it already succeeded).
validatecalibrationsim() {
    source ~/ros2_ws/install/setup.bash
    ros2 run calibration_validation validate_calibration_sim.py
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
    colcon build --packages-up-to aruco_moveit_config visual_calibration_msgs visual_calibration_moveit aruco_perception calibration_validation
    source install/setup.bash
}

vcbuildsymlink() {
    cd ~/ros2_ws || return
    colcon build --packages-up-to aruco_moveit_config visual_calibration_msgs visual_calibration_moveit aruco_perception calibration_validation --symlink-install
    source install/setup.bash
}

vccleanbuild() {
    cd ~/ros2_ws || return

    rm -rf build/aruco_moveit_config \
        build/visual_calibration_msgs \
        build/visual_calibration_moveit \
        build/aruco_perception \
        build/calibration_validation
    rm -rf install/aruco_moveit_config \
        install/visual_calibration_msgs \
        install/visual_calibration_moveit \
        install/aruco_perception \
        install/calibration_validation

    colcon build --packages-up-to aruco_moveit_config visual_calibration_msgs visual_calibration_moveit aruco_perception calibration_validation
    source install/setup.bash
}

vccleanbuildsymlink() {
    cd ~/ros2_ws || return

    rm -rf build/aruco_moveit_config \
        build/visual_calibration_msgs \
        build/visual_calibration_moveit \
        build/aruco_perception \
        build/calibration_validation
    rm -rf install/aruco_moveit_config \
        install/visual_calibration_msgs \
        install/visual_calibration_moveit \
        install/aruco_perception \
        install/calibration_validation

    colcon build --packages-up-to aruco_moveit_config visual_calibration_msgs visual_calibration_moveit aruco_perception calibration_validation --symlink-install
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
