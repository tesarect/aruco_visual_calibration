"""ExecuteProcess-based readiness gate: blocks a launch sequence until a
node matching `node_name` appears in `ros2 node list`, or `timeout_sec`
elapses. Mirrors resources/scripts/shell/wait_for_node.sh's polling
strategy — used here because launch's OnProcessStart event handler can't
attach across an IncludeLaunchDescription boundary (see progress.md),
while this works with any already-included launch file's nodes.
"""

from launch.actions import ExecuteProcess


def wait_for_node_action(node_name: str, timeout_sec: int = 30) -> ExecuteProcess:
    script = (
        f'ELAPSED=0; TIMEOUT={timeout_sec}; '
        f'echo "Waiting for node matching \'{node_name}\' (timeout ${{TIMEOUT}}s)..."; '
        f'while ! ros2 node list 2>/dev/null | grep -q "{node_name}"; do '
        f'sleep 1; ELAPSED=$((ELAPSED + 1)); '
        f'if [ "$ELAPSED" -ge "$TIMEOUT" ]; then '
        f'echo "Timed out waiting for node \'{node_name}\' after ${{TIMEOUT}}s — continuing anyway."; '
        f'exit 0; fi; done; '
        f'echo "Node matching \'{node_name}\' is up (waited ${{ELAPSED}}s)."'
    )
    return ExecuteProcess(
        cmd=["bash", "-c", script],
        output="screen",
    )
