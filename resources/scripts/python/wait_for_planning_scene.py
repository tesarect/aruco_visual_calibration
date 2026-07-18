#!/usr/bin/env python3
"""Polls the live planning scene until both "countertop" and "wall"
collision objects are present, or a timeout elapses.

Replaces an earlier bash version that called `ros2 service call
/get_planning_scene ...` in a tight ~1s loop — each call spun up a brand
new CLI client process against move_group, and that repeated connect/
disconnect churn is believed to have crashed move_group once during
testing (RCLError: "failed to send response: client will not receive
response", move_group terminated). This script creates ONE persistent
rclpy node + one PlanningSceneInterface, waits for /get_planning_scene
once via wait_for_service, then polls with a single long-lived client at
a slow interval (default 2s) instead of respawning a process per check.

Why this matters: TrajectoryPlanner's move_to_home_on_startup (see
trajectory_planner_sim.yaml/_real.yaml) plans/executes a move as soon as
its constructor runs — collision checking is on by default, but it can
only avoid obstacles already in the scene at plan time. This script gates
trajectory_planner's launch on the scene actually being populated,
instead of just on move_group being reachable.

Only checks countertop/wall (the always-on core scene bounds) — not
coffee_machine/cupholder (optional, toggled via scene_objects_*.yaml).

Usage:
    python3 wait_for_planning_scene.py [--timeout SECONDS] [--interval SECONDS]
Exit code 0 on success, 1 on timeout (caller decides whether to proceed
anyway — same convention as wait_for_node.sh/wait_for_controllers.sh).
"""

import argparse
import sys
import time

import rclpy


REQUIRED_OBJECTS = {"countertop", "wall"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=30.0, help="Timeout in seconds")
    parser.add_argument("--interval", type=float, default=2.0, help="Poll interval in seconds")
    args = parser.parse_args()

    rclpy.init()
    node = _PlanningSceneWaitNode()
    deadline = time.monotonic() + args.timeout
    print(
        f"Waiting for planning scene to contain {sorted(REQUIRED_OBJECTS)} "
        f"(timeout {args.timeout:.0f}s)...", flush=True)

    while time.monotonic() < deadline:
        known = node.get_known_object_names()
        if REQUIRED_OBJECTS.issubset(known):
            elapsed = args.timeout - (deadline - time.monotonic())
            print(f"Planning scene has {sorted(REQUIRED_OBJECTS)} (waited {elapsed:.0f}s).")
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(0)
        time.sleep(args.interval)

    print(
        f"Timed out waiting for planning scene objects after {args.timeout:.0f}s — "
        "continuing anyway.")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(1)


class _PlanningSceneWaitNode:
    """Thin wrapper around a single, long-lived /get_planning_scene client —
    one rclpy node, one service client, reused across every poll. Calls
    the service directly with rclpy, the same mechanism
    moveit::planning_interface::PlanningSceneInterface::getKnownObjectNames()
    uses under the hood in C++ (both go through GetPlanningScene) —
    avoids depending on moveit_commander's Python bindings."""

    def __init__(self):
        from rclpy.node import Node
        from moveit_msgs.srv import GetPlanningScene
        from moveit_msgs.msg import PlanningSceneComponents

        self._GetPlanningScene = GetPlanningScene
        self._PlanningSceneComponents = PlanningSceneComponents
        self._node = Node("wait_for_planning_scene")
        self._client = self._node.create_client(GetPlanningScene, "/get_planning_scene")
        self._client.wait_for_service()

    def get_known_object_names(self):
        request = self._GetPlanningScene.Request()
        request.components.components = self._PlanningSceneComponents.WORLD_OBJECT_NAMES
        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
        result = future.result()
        if result is None:
            return set()
        return {obj.id for obj in result.scene.world.collision_objects}

    def destroy_node(self):
        self._node.destroy_node()


if __name__ == "__main__":
    main()