
# Add this to you bashrc
# [ -f "$HOME/ros2_ws/src/visual_calibration/resources/scripts/aliases.sh" ] && . "$HOME/ros2_ws/src/visual_calibration/resources/scripts/aliases.sh"
#               or this
# grep -qxF '[ -f "$HOME/ros2_ws/src/visual_calibration/resources/scripts/aliases.sh" ] && . "$HOME/ros2_ws/src/visual_calibration/resources/scripts/aliases.sh"' ~/.bashrc ||
# echo '[ -f "$HOME/ros2_ws/src/visual_calibration/resources/scripts/aliases.sh" ] && . "$HOME/ros2_ws/src/visual_calibration/resources/scripts/aliases.sh"' >> ~/.bashrc

alias srcrc="source ~/.bashrc"
alias vcdir="cd ~/ros2_ws/src/visual_calibration/"
alias shdir="cd ~/ros2_ws/src/visual_calibration/resources/scripts/"
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
    local var="${1}"
    # TODO:
    # accept var, if var is equal to string execute specific command 
    # if 
    # pkill gzclient*
    # if 
    # tmux kill-session -t visual_calibration
}

tmuxbasesim() {
    cd ~/ros2_ws/src/visual_calibration/resources/scripts/
    bash ./sim_tmux_base.sh
}

tmuxmain1sim() {
    cd ~/ros2_ws/src/visual_calibration/resources/scripts/
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

tracepolygon() {
    ros2 service call /trajectory_planner/trace_polygon std_srvs/srv/Trigger {}
}

startcalibration() {
    ros2 service call /calibration_broadcaster_node/start_calibration std_srvs/srv/Trigger {}
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
    colcon build --packages-up-to aruco_moveit_config visual_calibration_msgs visual_calibration_moveit aruco_perception
    source install/setup.bash
}

vcbuildsymlink() {
    cd ~/ros2_ws || return
    colcon build --packages-up-to aruco_moveit_config visual_calibration_msgs visual_calibration_moveit aruco_perception --symlink-install
    source install/setup.bash
}

vccleanbuild() {
    cd ~/ros2_ws || return

    rm -rf build/aruco_moveit_config \
        build/visual_calibration_msgs \
        build/visual_calibration_moveit \
        build/aruco_perception
    rm -rf install/aruco_moveit_config \
        install/visual_calibration_msgs \
        install/visual_calibration_moveit \
        install/aruco_perception

    colcon build --packages-up-to aruco_moveit_config visual_calibration_msgs visual_calibration_moveit aruco_perception
    source install/setup.bash
}

vccleanbuildsymlink() {
    cd ~/ros2_ws || return

    rm -rf build/aruco_moveit_config \
        build/visual_calibration_msgs \
        build/visual_calibration_moveit \
        build/aruco_perception
    rm -rf install/aruco_moveit_config \
        install/visual_calibration_msgs \
        install/visual_calibration_moveit \
        install/aruco_perception

    colcon build --packages-up-to aruco_moveit_config visual_calibration_msgs visual_calibration_moveit aruco_perception --symlink-install
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
