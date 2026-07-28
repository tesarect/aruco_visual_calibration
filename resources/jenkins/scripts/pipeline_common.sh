#!/bin/bash
# Shared helpers for every stage script under resources/jenkins/scripts/.
# Sourced (not executed) by each stage script — NOT the same mechanism as
# resources/scripts/shell/logging.sh (that's tmux's opt-in per-pane tee
# convention); Jenkins archives each stage's log file itself
# (archiveArtifacts in the Jenkinsfile), so this only needs to (a) put each
# stage's full stdout/stderr somewhere Jenkins can find and archive, and
# (b) track PIDs of anything backgrounded so the pipeline can tear them
# down later (see kill_tracked_pids below and the Jenkinsfile's post block).
#
# All log files land in $WORKSPACE/logs/$BUILD_NUMBER/ (Jenkins' per-build
# workspace, NOT ros2_ws/log/tmux/ — keeps Jenkins-triggered runs and
# tmux-driven dev runs' logs from ever mixing in the same directory).
# Nested under $BUILD_NUMBER specifically (not just $WORKSPACE/logs/) so
# Promtail can extract the build number from the file PATH itself as a
# label (same technique already used for the "component" label — see
# install_grafana.sh's promtail-config.yaml pipeline_stages) — Jenkins
# reuses the same $WORKSPACE path across every build of a job, so without
# this nesting there'd be no way to tell which build a given log line
# came from once multiple builds' logs land in the same flat directory.
# Jenkins' own archiveArtifacts patterns (logs/**/*.log) already glob
# recursively, so this needs no Jenkinsfile change to keep archiving
# working.
#
# PIDs of long-lived background processes started by any stage are appended
# to $LOG_DIR/.pids so a later stage (or the pipeline's post block)
# can find and kill them irrespective of which stage started them — this is
# the answer to "what kills Gazebo/move_group when the pipeline finishes or
# is aborted": every stage that backgrounds something MUST call
# track_pid "$!" right after backgrounding it.

set -uo pipefail

LOG_DIR="${WORKSPACE:?WORKSPACE not set — source this from a Jenkins sh step}/logs/${BUILD_NUMBER:?BUILD_NUMBER not set — source this from a Jenkins sh step}"
mkdir -p "$LOG_DIR"
PID_FILE="$LOG_DIR/.pids"
touch "$PID_FILE"

# Records a PID so it can be torn down later. Also records a short label
# alongside it purely for readability when hand-inspecting .pids — the
# label is not parsed by kill_tracked_pids.
track_pid() {
    local pid="$1"
    local label="${2:-unlabeled}"
    echo "$pid $label" >> "$PID_FILE"
    echo "[pipeline_common] tracked PID $pid ($label)"
}

# Kills every PID recorded in .pids, most-recently-started first (roughly
# reverses dependency order — e.g. RViz/move_group before Gazebo), ignoring
# PIDs that already exited. Safe to call multiple times. Does NOT clear
# .pids itself (left for post-mortem inspection) — the Jenkinsfile's post
# block is expected to call this exactly once, at the very end of the run
# (success, failure, or abort all funnel through Jenkins' `post` stage).
kill_tracked_pids() {
    if [ ! -s "$PID_FILE" ]; then
        echo "[pipeline_common] no tracked PIDs to kill."
        return 0
    fi
    echo "[pipeline_common] killing tracked PIDs (most recent first)..."
    tac "$PID_FILE" | while read -r pid label; do
        [ -z "${pid:-}" ] && continue
        if kill -0 "$pid" 2>/dev/null; then
            echo "[pipeline_common] killing $pid ($label)"
            kill -TERM "$pid" 2>/dev/null || true
            sleep 1
            kill -KILL "$pid" 2>/dev/null || true
        else
            echo "[pipeline_common] $pid ($label) already gone"
        fi
    done
}

# Best-effort cleanup of anything NOT captured by a tracked PID — e.g.
# gzserver/gzclient children Gazebo's launch process spawns, which don't
# share the launched PID directly. Mirrors aliases.sh's customkill/killsim
# intent but scoped to what THIS pipeline run may have started; does not
# touch tmux sessions (tmux stays completely independent, see design doc).
#
# Waits (up to 10s) for gzserver's master port (11345) to actually be free
# before returning — confirmed live root cause of a real failure: pkill
# sends SIGTERM and returns immediately, but a gzserver under load can take
# a moment to actually release its port; the caller (stage_gazebo_sim.sh)
# launches a brand new gzserver immediately after this returns, so without
# this wait the new gzserver can lose the same "Address already in use"
# race the sweep was supposed to prevent.
kill_stray_ros_processes() {
    echo "[pipeline_common] best-effort sweep of stray gzserver/gzclient/ros2 processes..."
    pkill -f "gzserver" 2>/dev/null || true
    pkill -f "gzclient" 2>/dev/null || true
    pkill -f "zenoh-bridge-ros2dds" 2>/dev/null || true

    local waited=0
    while [ "$waited" -lt 10 ]; do
        if ! (echo > /dev/tcp/127.0.0.1/11345) 2>/dev/null; then
            return 0
        fi
        sleep 1
        waited=$((waited + 1))
    done
    echo "[pipeline_common] WARNING: port 11345 still held after ${waited}s wait — proceeding anyway."
}
