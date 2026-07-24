#!/bin/bash
# Rollback for install_yolo.sh: removes the isolated YOLO venv and its
# cached model checkpoints, restoring the system to its pre-install state.
# Does not touch anything ROS-side (system python, cv_bridge, dist-packages)
# since install_yolo.sh never wrote to them in the first place.
#
# Usage:
#   bash remove_yolo.sh

set -euo pipefail

VENV_DIR="$HOME/yolo_venv"

if [ -d "$VENV_DIR" ]; then
    echo "Removing venv: $VENV_DIR"
    rm -rf "$VENV_DIR"
else
    echo "No venv found at $VENV_DIR (already removed or never installed)."
fi

# ultralytics caches downloaded weights/config under ~/.config/Ultralytics
# and torch caches under ~/.cache/torch — both outside the venv, so clean
# them separately for a true full rollback.
CONFIG_DIR="$HOME/.config/Ultralytics"
TORCH_CACHE_DIR="$HOME/.cache/torch"

if [ -d "$CONFIG_DIR" ]; then
    echo "Removing $CONFIG_DIR"
    rm -rf "$CONFIG_DIR"
fi

if [ -d "$TORCH_CACHE_DIR" ]; then
    echo "Removing $TORCH_CACHE_DIR"
    rm -rf "$TORCH_CACHE_DIR"
fi

# Any *.pt checkpoint files ultralytics may have dropped in the CWD when
# YOLO('yolo11n.pt') was run from an arbitrary directory.
find "$HOME" -maxdepth 2 -iname "yolo*.pt" -print -delete 2>/dev/null || true

echo ""
echo "YOLO venv and caches removed. System restored to pre-install state."