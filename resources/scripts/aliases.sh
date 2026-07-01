
# Add this to you bashrc
# [ -f "$HOME/ros2_ws/src/visual_calibration/resources/scripts/aliases.sh" ] && . "$HOME/ros2_ws/src/visual_calibration/resources/scripts/aliases.sh"
#               or this
# grep -qxF '[ -f "$HOME/ros2_ws/src/visual_calibration/resources/scripts/aliases.sh" ] && . "$HOME/ros2_ws/src/visual_calibration/resources/scripts/aliases.sh"' ~/.bashrc ||
# echo '[ -f "$HOME/ros2_ws/src/visual_calibration/resources/scripts/aliases.sh" ] && . "$HOME/ros2_ws/src/visual_calibration/resources/scripts/aliases.sh"' >> ~/.bashrc

alias srcrc="source ~/.bashrc"
alias vcdir="cd ~/ros2_ws/src/visual_calibration"

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

vcbuild() {
    cd ~/ros2_ws || return
    colcon build --packages-up-to aruco_moveit_config
    source install/setup.bash
}

vcbuildsymlink() {
    cd ~/ros2_ws || return
    colcon build --packages-up-to aruco_moveit_config --symlink-install
    source install/setup.bash
}

vccleanbuild() {
    cd ~/ros2_ws || return

    rm -rf build/aruco_moveit_config
    rm -rf install/aruco_moveit_config

    colcon build --packages-up-to aruco_moveit_config --symlink-install
    source install/setup.bash
}