#!/bin/bash
# Master launcher, sim: fires sim_win_base.sh, sim_win_trajcal.sh,
# sim_win_yolo.sh (all with essential_logs=on) and win_debug.sh sim (no
# logs) in order, as four WINDOWS in one shared "sim_deploy" tmux session,
# then attaches once — switch between them with prefix+0/1/2/3 (window
# number) or prefix+w (window picker; prefix is whatever tmux.conf sets,
# default Ctrl-b).
#
# Rewritten (2026-07-26) to use the _win_ scripts instead of separate-
# session tmux*.sh scripts — one session end-to-end, easier to run as a
# single deploy step while Jenkins/Grafana are unavailable/disabled. The
# _win_ scripts (sim_win_base.sh/sim_win_trajcal.sh/sim_win_yolo.sh/
# win_debug.sh) deliberately have NO trailing attach-session call (see
# sim_win_base.sh's header) — they add their window and return
# immediately, so no setsid/detach workaround is needed here anymore
# (previously required because sim_tmux_base.sh etc. each end with their
# own blocking attach-session, meant for standalone use). Standalone
# tmuxbasesim/tmuxtrajcalsim/tmuxyolosim/tmuxdebug aliases and their
# underlying scripts are untouched and still create independent sessions.
#
# Sequencing is real, not cosmetic: sim_win_trajcal.sh's trajectory_planner
# pane polls for move_group + a populated planning scene (both created by
# sim_win_base.sh) before it plans its startup home-move — starting base
# and trajcal in the wrong order (or fully in parallel) risks trajcal's
# very first plan racing an empty/absent planning scene. sim_win_yolo.sh
# only needs move_group (also from base). win_debug.sh's tf_debug_markers.py
# pane polls for move_group too. Each child script's own wait_for_node.sh/
# wait_for_planning_scene.sh calls are what actually GATE readiness inside
# each pane — this script's own BASE_DELAY_SEC fixed delay below is an
# additional, explicit head start (not a substitute for those polls): base
# is given BASE_DELAY_SEC to get Gazebo/move_group/planning_scene underway
# before trajcal AND yolo are both fired (yolo doesn't need the planning
# scene, only move_group, but per explicit user choice it waits the same
# fixed delay as trajcal, counted from base's start — not chained after
# trajcal). debug is fired immediately after with no delay (it has no
# meaningful shared-resource race with the others beyond its own
# move_group poll).
BASE_DELAY_SEC=60
#
# Usage: ./orchestrate_tmux_sim.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESSION="sim_deploy"

echo "=== [orchestrate_tmux_sim] Starting base window (Gazebo/move_group/rviz/planning_scene) ==="
bash "$SCRIPT_DIR/sim_win_base.sh" essential_logs=on

echo "=== [orchestrate_tmux_sim] Waiting ${BASE_DELAY_SEC}s before trajcal/yolo (head start for base) ==="
sleep "$BASE_DELAY_SEC"

echo "=== [orchestrate_tmux_sim] Starting trajcal window (trajectory + calibration pipeline) ==="
bash "$SCRIPT_DIR/sim_win_trajcal.sh" essential_logs=on

echo "=== [orchestrate_tmux_sim] Starting yolo window (YOLO/hybrid-detector) ==="
bash "$SCRIPT_DIR/sim_win_yolo.sh" essential_logs=on

echo "=== [orchestrate_tmux_sim] Starting debug window (no logs) ==="
bash "$SCRIPT_DIR/win_debug.sh" sim

echo "=== [orchestrate_tmux_sim] All windows launched in session '$SESSION': base, trajcal, yolo, debug ==="
echo "=== [orchestrate_tmux_sim] Attaching — switch windows with prefix+0/1/2/3 or prefix+w ==="

tmux attach-session -t "$SESSION"
