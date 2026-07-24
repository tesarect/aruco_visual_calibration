"""NEW, additive file — does not replace or modify yolo_marker_bridge.launch.py,
start_inference_server.sh, or wait_for_inference_server.sh, all left
untouched. See visual_calibration_moveit/launch/bringup_full_sim_README.md
(and bringup_full_real_README.md) for the full staged-bringup design this
file is part of.

Combines inference_server.py + yolo_marker_bridge_node into one launch file.
inference_server.py is NOT a ROS node (plain Flask process inside
~/yolo_venv, never imports rclpy — see start_inference_server.sh's header
and error-mitigation.md #15), so it cannot be started via launch_ros's Node
action at all — started here via ExecuteProcess instead, invoking the
venv's python3 interpreter directly (NOT `source .../activate`, which is a
shell-only construct ExecuteProcess doesn't get for free without wrapping
in `bash -c`).

Readiness gate mirrors wait_for_inference_server.sh: poll GET
http://127.0.0.1:8600/health for `"status": "ok"` in the response body,
via Python's urllib (no extra dependency) rather than shelling out to curl.
yolo_marker_bridge_node then additionally waits for move_group, matching
real_tmux_trajcal.sh/sim_tmux_trajcal.sh's pane 5 chain
(wait_for_inference_server.sh && wait_for_node.sh move_group).
"""

import json
import os
import time
import urllib.request
import urllib.error

import rclpy
from rclpy.node import Node as RclpyNode

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


HEALTH_URL = "http://127.0.0.1:8600/health"
INFERENCE_SERVER_WAIT_TIMEOUT_SEC = 30.0
INFERENCE_SERVER_POLL_INTERVAL_SEC = 1.0
MOVE_GROUP_WAIT_TIMEOUT_SEC = 30.0
MOVE_GROUP_POLL_INTERVAL_SEC = 1.0

YOLO_PIPELINE_DIR = os.path.expanduser("~/YOLO-pipeline")
VENV_PYTHON = os.path.expanduser("~/yolo_venv/bin/python3")


def _wait_for_inference_server(timeout_sec, interval_sec):
    """Mirrors wait_for_inference_server.sh: poll HEALTH_URL until the
    response body contains a "status": "ok" field, or timeout_sec elapses.
    Returns True if ready, False on timeout — caller decides whether to
    proceed anyway (same convention as the shell script).
    """
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        try:
            with urllib.request.urlopen(HEALTH_URL, timeout=2) as resp:
                body = json.loads(resp.read().decode())
                if body.get("status") == "ok":
                    return True
        except (urllib.error.URLError, TimeoutError, ValueError, ConnectionError):
            pass
        time.sleep(interval_sec)
    return False


def _wait_for_node(node_name_substring, timeout_sec, interval_sec):
    """Same substring-match semantics as wait_for_node.sh — see
    bringup_aruco_pipeline.launch.py's identical helper for the full
    rationale (not shared as an importable module since these bringup
    files must each stay independently launchable from an install space
    with no guaranteed shared Python path between packages)."""
    rclpy.init(args=None)
    node = RclpyNode("bringup_yolo_pipeline_node_wait")

    deadline = time.monotonic() + timeout_sec
    found = False
    while time.monotonic() < deadline:
        names = node.get_node_names()
        if any(node_name_substring in n for n in names):
            found = True
            break
        time.sleep(interval_sec)

    node.destroy_node()
    rclpy.shutdown()
    return found


def _launch_setup(context, *args, **kwargs):
    env = LaunchConfiguration("env").perform(context)

    print(
        f"[bringup_yolo_pipeline] Waiting for inference_server.py at {HEALTH_URL} "
        f"(timeout {INFERENCE_SERVER_WAIT_TIMEOUT_SEC:.0f}s)...", flush=True,
    )
    server_ready = _wait_for_inference_server(
        INFERENCE_SERVER_WAIT_TIMEOUT_SEC, INFERENCE_SERVER_POLL_INTERVAL_SEC)
    if server_ready:
        print("[bringup_yolo_pipeline] inference_server.py is up.", flush=True)
    else:
        print(
            "[bringup_yolo_pipeline] Timed out waiting for inference_server.py — "
            "continuing anyway (same convention as wait_for_inference_server.sh).",
            flush=True,
        )

    print(
        f"[bringup_yolo_pipeline] Waiting for move_group "
        f"(timeout {MOVE_GROUP_WAIT_TIMEOUT_SEC:.0f}s)...", flush=True,
    )
    move_group_ready = _wait_for_node(
        "move_group", MOVE_GROUP_WAIT_TIMEOUT_SEC, MOVE_GROUP_POLL_INTERVAL_SEC)
    if move_group_ready:
        print(
            "[bringup_yolo_pipeline] move_group is up — starting "
            "yolo_marker_bridge_node.", flush=True,
        )
    else:
        print(
            "[bringup_yolo_pipeline] Timed out waiting for move_group — starting "
            "yolo_marker_bridge_node anyway (same 'continuing anyway' convention "
            "as wait_for_node.sh).", flush=True,
        )

    yolo_marker_bridge_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("aruco_perception_yolo_bridge"),
                "launch", "yolo_marker_bridge.launch.py",
            ])
        ),
        launch_arguments={"env": env}.items(),
    )
    return [yolo_marker_bridge_include]


def generate_launch_description():
    env_arg = DeclareLaunchArgument(
        "env",
        default_value="sim",
        description="Which parameter file to load: 'sim' or 'real'",
    )

    inference_server_process = ExecuteProcess(
        cmd=[VENV_PYTHON, "inference_server.py", "--env", LaunchConfiguration("env")],
        cwd=YOLO_PIPELINE_DIR,
        output="screen",
        # Same directories/existence assumptions start_inference_server.sh
        # checks explicitly with a friendly error — ExecuteProcess itself
        # will just fail to spawn (FileNotFoundError-style launch error)
        # if VENV_PYTHON or YOLO_PIPELINE_DIR don't exist; run
        # install_yolo.sh first if this action fails to start.
    )

    return LaunchDescription([
        env_arg,
        inference_server_process,
        # See bringup_moveit_pipeline.launch.py's matching comment: this
        # only guarantees inference_server.py's process is STARTED before
        # the wait begins — the blocking waits inside _launch_setup are
        # what actually enforce both gates.
        OpaqueFunction(function=_launch_setup),
    ])
