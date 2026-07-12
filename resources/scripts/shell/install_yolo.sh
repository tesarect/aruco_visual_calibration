#!/bin/bash
# Installs an isolated Python venv for YOLO (ultralytics) at ~/yolo_venv,
# fully separate from ROS's system Python / dist-packages (which cv_bridge
# and image_transport are compiled against, on OpenCV 4.5.4 — see
# error-mitigation.md #15). No --system-site-packages, so ultralytics'
# own bundled OpenCV can never shadow or conflict with the ROS-side one.
#
# CPU-only build (no nvidia-smi found on this rosject) — skips CUDA wheels,
# which alone run 2-4GB and would blow past the "unpersisted beyond ~2GB"
# constraint on this box for no benefit without a GPU.
#
# Idempotent: safe to re-run any time (e.g. after a rosject reset wipes
# ~/yolo_venv) — just re-creates it from scratch. Companion script:
# remove_yolo.sh, for a full rollback.
#
# Usage:
#   bash install_yolo.sh

set -euo pipefail

VENV_DIR="$HOME/yolo_venv"

if [ -d "$VENV_DIR" ]; then
    echo "Removing existing venv at $VENV_DIR before reinstall..."
    rm -rf "$VENV_DIR"
fi

echo "Creating venv at $VENV_DIR (isolated, no ROS system-site-packages)..."
python3 -m venv "$VENV_DIR"

# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"

echo "Upgrading pip..."
pip install --upgrade pip --quiet

echo "Installing CPU-only torch (skip if already resolved as CPU by ultralytics' own pin)..."
pip install --quiet torch --index-url https://download.pytorch.org/whl/cpu

echo "Installing ultralytics (YOLO) + opencv-python-headless..."
pip install --quiet ultralytics opencv-python-headless

echo "Fetching the nano checkpoint (smallest, ~6MB) so first run doesn't stall on a cold download..."
python3 -c "from ultralytics import YOLO; YOLO('yolo11n.pt')"

deactivate

echo ""
echo "Done. Venv installed at: $VENV_DIR"
du -sh "$VENV_DIR"
echo ""
echo "To use it:"
echo "  source $VENV_DIR/bin/activate"
echo "  python3 -c \"from ultralytics import YOLO; print('ok')\""
echo "  deactivate"
echo ""
echo "Do NOT source this venv in the same shell as a sourced ROS setup.bash"
echo "when running nodes that import cv_bridge — keep YOLO inference and"
echo "ROS/cv_bridge work in separate processes (see architecture notes)."