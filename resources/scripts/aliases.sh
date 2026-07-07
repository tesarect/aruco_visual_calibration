
# Add this to you bashrc
# [ -f "$HOME/ros2_ws/src/visual_calibration/resources/scripts/aliases.sh" ] && . "$HOME/ros2_ws/src/visual_calibration/resources/scripts/aliases.sh"
#               or this
# grep -qxF '[ -f "$HOME/ros2_ws/src/visual_calibration/resources/scripts/aliases.sh" ] && . "$HOME/ros2_ws/src/visual_calibration/resources/scripts/aliases.sh"' ~/.bashrc ||
# echo '[ -f "$HOME/ros2_ws/src/visual_calibration/resources/scripts/aliases.sh" ] && . "$HOME/ros2_ws/src/visual_calibration/resources/scripts/aliases.sh"' >> ~/.bashrc

alias srcrc="source ~/.bashrc"
alias vcdir="cd ~/ros2_ws/src/visual_calibration"
alias viewcam="ros2 run rqt_image_view rqt_image_view /wrist_rgbd_depth_sensor/image_raw"
alias overlayviewcam="ros2 run rqt_image_view rqt_image_view /aruco_perception/overlay_image"

# git
cleanpull() {
    git reset --hard HEAD
    git clean -fd
    git pull
}

killsim() {
    pkill gzclient*
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
    colcon build --packages-up-to aruco_moveit_config visual_calibration_moveit aruco_perception
    source install/setup.bash
}

vcbuildsymlink() {
    cd ~/ros2_ws || return
    colcon build --packages-up-to aruco_moveit_config visual_calibration_moveit aruco_perception --symlink-install
    source install/setup.bash
}

vccleanbuild() {
    cd ~/ros2_ws || return

    rm -rf build/aruco_moveit_config \
        build/visual_calibration_moveit \
        build/aruco_perception
    rm -rf install/aruco_moveit_config \
        install/visual_calibration_moveit \
        install/aruco_perception

    colcon build --packages-up-to aruco_moveit_config visual_calibration_moveit aruco_perception
    source install/setup.bash
}

vccleanbuildsymlink() {
    cd ~/ros2_ws || return

    rm -rf build/aruco_moveit_config \
        build/visual_calibration_moveit \
        build/aruco_perception
    rm -rf install/aruco_moveit_config \
        install/visual_calibration_moveit \
        install/aruco_perception

    colcon build --packages-up-to aruco_moveit_config visual_calibration_moveit aruco_perception --symlink-install
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
