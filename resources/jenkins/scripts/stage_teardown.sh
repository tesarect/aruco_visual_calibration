#!/bin/bash
# Jenkins pipeline `post { always {} }` teardown — the answer to "what kills
# Gazebo/move_group/the bringup launch trees when the pipeline finishes or
# is aborted". Called exactly once per build, unconditionally, regardless
# of which stage (if any) failed or whether the build was manually stopped
# — Jenkins guarantees `post { always {} }` runs in all of those cases.
#
# Does two independent things:
#   1. kill_tracked_pids — walks $WORKSPACE/logs/.pids (every PID any stage
#      script backgrounded via track_pid), most-recently-started first.
#   2. kill_stray_ros_processes — best-effort sweep for anything a tracked
#      PID doesn't directly cover (e.g. gzserver/gzclient children Gazebo's
#      launch process spawns under a different PID than the one `&`
#      returned).
#
# Does NOT touch tmux sessions — tmux stays completely independent of
# Jenkins (see design doc); a Jenkins build ending must never kill a
# developer's separate tmux dev session.

set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=./pipeline_common.sh
source "$SCRIPT_DIR/pipeline_common.sh"

echo "=== [stage_teardown] Tearing down this build's tracked/stray processes ==="
kill_tracked_pids
kill_stray_ros_processes
echo "=== [stage_teardown] Teardown complete ==="
