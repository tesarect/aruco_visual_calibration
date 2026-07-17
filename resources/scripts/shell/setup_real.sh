#!/bin/bash

set -e  # exit on any error

# This rosject container doesn't run systemd as PID 1 (common in
# Docker-based cloud IDEs). zenoh-bridge-ros2dds's postinstall script tries
# to register/start a systemd service and hard-fails without this — which
# then aborts apt's whole transaction (via set -e below), taking tmux/xclip
# down with it even though they're unrelated packages. SYSTEMD_OFFLINE=1
# tells that postinstall script to skip the systemd calls instead of
# failing. Exported for the whole script since apt is called multiple
# times below. dpkg --configure -a defensively re-runs configuration for
# anything left half-installed from a PRIOR failed run of this script
# (e.g. if the container was reset mid-install last time) — safe/no-op if
# nothing is broken.
export SYSTEMD_OFFLINE=1
sudo -E dpkg --configure -a || true

echo "🔧 Removing old ROS sources..."
sudo rm -f /etc/apt/sources.list.d/ros*.list

echo "🔧 Creating keyrings directory..."
sudo mkdir -p /usr/share/keyrings

echo "🔧 Adding ROS key..."
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  | sudo tee /usr/share/keyrings/ros-archive-keyring.gpg > /dev/null

echo "🔧 Adding ROS2 repository..."
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu focal main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "🌍 Switching to automatic mirror selection..."
sudo sed -i 's|http://archive.ubuntu.com/ubuntu|mirror://mirrors.ubuntu.com/mirrors.txt|g' /etc/apt/sources.list
sudo sed -i 's|http://security.ubuntu.com/ubuntu|mirror://mirrors.ubuntu.com/mirrors.txt|g' /etc/apt/sources.list

echo "🔄 Updating package lists..."
sudo apt-get update || true

echo ">_ Installing Tmux..."
sudo -E apt install -y xclip
sudo -E apt install -y tmux

echo "🖱️  Installing tmux.conf (mouse mode, prefix, clipboard, etc.)..."
TMUX_CONF_SRC="$HOME/ros2_ws/src/visual_calibration/resources/scripts/tmux/tmux.conf"
TMUX_CONF_DST="$HOME/.tmux.conf"
# Overwrites any existing ~/.tmux.conf — see setup.sh's matching block for
# why (this was previously a commented-out no-op on every rosject).
if [ -f "$TMUX_CONF_SRC" ]; then
    cp "$TMUX_CONF_SRC" "$TMUX_CONF_DST"
fi

echo "🎦 Installing Zenoh..."
cd ~/ros2_ws/src/zenoh-pointcloud/
./install_zenoh.sh

# echo "🎦 Start Zenoh..."
# cd ~/ros2_ws/src/zenoh-pointcloud/init
# ./rosject.sh

echo "✅ Setup complete!"