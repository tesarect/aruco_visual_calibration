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
# Codename must match the actual OS (jammy/22.04 on this rosject) — see
# setup.sh's matching comment for the full story (was hardcoded "focal",
# silently served the wrong Ubuntu 20.04 package set on a jammy system).
UBUNTU_CODENAME="$(. /etc/os-release && echo "$UBUNTU_CODENAME")"
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "🌍 Pinning apt to a fixed mirror (archive.ubuntu.com)..."
# See setup.sh's matching comment — mirror://mirrors.ubuntu.com/mirrors.txt
# (automatic mirror selection) was the cause of the "Ign"/long-hanging GET
# behavior seen on this rosject; pinning to Ubuntu's plain default archive
# removes the per-run mirror-list-fetch-and-probe step entirely. Harmless
# no-op if already pinned by a previous run.
sudo sed -i 's|mirror://mirrors.ubuntu.com/mirrors.txt|http://archive.ubuntu.com/ubuntu|g' /etc/apt/sources.list

# Timeout=15/Retries=2 so a genuinely unreachable/slow source fails fast on
# every apt call below — see setup.sh's matching comment for why this is a
# drop-in config file rather than per-call -o flags.
echo 'Acquire::http::Timeout "15";
Acquire::https::Timeout "15";
Acquire::Retries "2";' | sudo tee /etc/apt/apt.conf.d/99-fast-fail-timeout > /dev/null

echo "🔄 Updating package lists..."
sudo apt-get update || true

echo ">_ Installing Tmux..."
sudo -E apt install -y xclip
sudo -E apt install -y tmux

echo "📋 Installing swri_console (rosout GUI log viewer)..."
sudo -E apt install -y ros-humble-swri-console

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

# HUSARNET-REENABLE-BLOCK (added 2026-07-26): Husarnet's apt signing key
# (/etc/apt/sources.list.d/husarnet.list) is currently EXPIRED
# (EXPKEYSIG 197D62F68A4C7BD6) — every `apt update` hard-errors on it,
# which was blocking unrelated installs (e.g. ros-humble-swri-console)
# earlier this session and had to be worked around by manually renaming
# the file to husarnet.list.disabled. This block re-enables it IF it was
# left disabled from that workaround, so a fresh setup_real.sh run doesn't
# silently inherit a disabled Husarnet repo from a prior session's manual
# fix. Best-effort only (|| true) — if the key is still expired, apt
# update will fail on this repo again exactly as before; that's a
# Husarnet-side problem (their signing key), not something this script can
# fix. Husarnet itself is only used for the real robot's Zenoh bridge (see
# CLAUDE.md), which is why this lives in setup_real.sh and not setup.sh.
# SAFE TO REMOVE: if this block ever causes trouble, delete from the
# "HUSARNET-REENABLE-BLOCK" comment above down to "END HUSARNET-REENABLE-
# BLOCK" below — that fully reverts this file to its pre-2026-07-26 state.
if [ -f /etc/apt/sources.list.d/husarnet.list.disabled ]; then
    echo "🔧 Re-enabling Husarnet apt source (was left disabled from a prior manual workaround)..."
    sudo mv /etc/apt/sources.list.d/husarnet.list.disabled /etc/apt/sources.list.d/husarnet.list
    sudo apt update || true
fi
# END HUSARNET-REENABLE-BLOCK

echo "✅ Setup complete!"