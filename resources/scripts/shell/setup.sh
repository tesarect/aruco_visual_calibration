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
# Codename must match the actual OS (jammy/22.04 on this rosject) — was
# hardcoded to "focal" here, which silently pointed every fresh setup at
# the wrong Ubuntu 20.04 package set (still resolves via packages.ros.org,
# so apt update/install didn't error, just quietly served focal-built
# packages/versions on a jammy system; caught when ros-humble-swri-console
# 404'd as "unable to locate package" despite existing for jammy/humble).
UBUNTU_CODENAME="$(. /etc/os-release && echo "$UBUNTU_CODENAME")"
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "🌍 Pinning apt to a fixed mirror (archive.ubuntu.com)..."
# mirror://mirrors.ubuntu.com/mirrors.txt (automatic mirror selection) was
# the cause of the "Ign"/long-hanging GET behavior seen on this rosject —
# every apt update/install first fetches the mirror LIST, then probes
# candidates for responsiveness, and on this network that probing itself
# was the slow part, not the actual package download. Pinning to Ubuntu's
# plain default archive removes that probe step entirely: one fixed,
# well-provisioned URL, no per-run mirror selection overhead. If this was
# already pinned by a previous run, the sed below is a harmless no-op (the
# mirror:// pattern won't match anything left to replace).
sudo sed -i 's|mirror://mirrors.ubuntu.com/mirrors.txt|http://archive.ubuntu.com/ubuntu|g' /etc/apt/sources.list

# Timeout=15/Retries=2 so a genuinely unreachable/slow source fails fast on
# EVERY apt call below (update AND install) — apt's own default is
# effectively "wait a very long time" per source, which is the other half
# of the "Ign ... taking forever" symptom this block exists to fix. Written
# to a drop-in config file (rather than repeating -o flags on each apt
# call) so it applies uniformly without editing every install line.
echo 'Acquire::http::Timeout "15";
Acquire::https::Timeout "15";
Acquire::Retries "2";' | sudo tee /etc/apt/apt.conf.d/99-fast-fail-timeout > /dev/null

echo "🔄 Updating package lists..."
sudo apt-get update || true

echo ">_ Installing Tmux..."
sudo apt install -y xclip
sudo apt install -y tmux

echo "📋 Installing swri_console (rosout GUI log viewer)..."
sudo apt install -y ros-humble-swri-console

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