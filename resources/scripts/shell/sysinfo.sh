#!/bin/bash
# Reports the cloud rosject's hardware/resource specs — CPU, GPU, RAM,
# disk, and general OS/kernel info — to the terminal. Read-only, makes no
# changes. Companion to diagnose_env.sh (ROS graph/TF state) and
# check_real_driver.sh (robot driver health) — this one is plain machine
# specs, no ROS involved at all.
#
# Usage: bash sysinfo.sh

echo "=== OS / Kernel ==="
if [ -f /etc/os-release ]; then
    . /etc/os-release
    echo "$PRETTY_NAME"
fi
uname -a
echo

echo "=== CPU ==="
if command -v lscpu >/dev/null 2>&1; then
    lscpu | grep -E "Model name|Socket|Core|Thread|CPU\(s\)|MHz"
else
    echo "lscpu not found — falling back to /proc/cpuinfo summary"
    grep -m1 "model name" /proc/cpuinfo
    echo "Logical CPUs: $(nproc)"
fi
echo

echo "=== RAM ==="
if command -v free >/dev/null 2>&1; then
    free -h
else
    echo "free not found — falling back to /proc/meminfo"
    grep -E "MemTotal|MemAvailable|SwapTotal" /proc/meminfo
fi
echo

echo "=== Disk / storage ==="
if command -v df >/dev/null 2>&1; then
    df -h --total 2>/dev/null | grep -vE "tmpfs|udev" || df -h
else
    echo "df not found — cannot report disk usage"
fi
echo

echo "=== GPU ==="
# This rosject has been observed with no nvidia-smi at all (see
# install_yolo.sh's CPU-only-build comment) — report that plainly instead
# of erroring, same "degrade gracefully" convention as
# wait_for_inference_server.sh's optional expected_env mismatch warning.
if command -v nvidia-smi >/dev/null 2>&1; then
    nvidia-smi --query-gpu=name,memory.total,memory.used,memory.free,utilization.gpu,driver_version --format=csv
elif command -v lspci >/dev/null 2>&1 && lspci | grep -qi vga; then
    echo "nvidia-smi not found, but a VGA/display controller is present:"
    lspci | grep -i vga
else
    echo "No GPU detected (no nvidia-smi, no lspci VGA controller found)."
fi
echo

echo "=== Cloud / virtualization hint ==="
# Best-effort only — cloud rosjects don't always expose a DMI product
# name, and this container may not be running as PID 1 under systemd
# (see setup_real.sh's SYSTEMD_OFFLINE note), so systemd-detect-virt can
# be absent or return "none" even when actually virtualized.
if command -v systemd-detect-virt >/dev/null 2>&1; then
    echo "Virtualization: $(systemd-detect-virt 2>/dev/null || echo unknown)"
fi
if [ -r /sys/class/dmi/id/product_name ]; then
    echo "DMI product name: $(cat /sys/class/dmi/id/product_name 2>/dev/null)"
fi
echo

echo "✅ sysinfo complete."
