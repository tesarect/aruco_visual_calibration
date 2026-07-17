#!/bin/bash

set -e  # exit on any error

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
sudo apt install -y xclip
sudo apt install -y tmux

echo "🖱️  Installing tmux.conf (mouse mode, prefix, clipboard, etc.)..."
TMUX_CONF_SRC="$HOME/ros2_ws/src/visual_calibration/resources/scripts/tmux/tmux.conf"
TMUX_CONF_DST="$HOME/.tmux.conf"
# Overwrites any existing ~/.tmux.conf — this was previously a
# commented-out no-op block, meaning tmux.conf's settings (mouse on,
# prefix, clipboard) never actually reached tmux on any rosject, no
# matter how the file itself was edited. Always overwrite (not just
# copy-if-missing) so project changes to tmux.conf propagate on every
# fresh setup run, rather than only the very first one.
if [ -f "$TMUX_CONF_SRC" ]; then
    cp "$TMUX_CONF_SRC" "$TMUX_CONF_DST"
fi

echo "✅ Setup complete!"