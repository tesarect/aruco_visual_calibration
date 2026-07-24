#!/usr/bin/env python3
"""Lightweight curses TUI showing the live/crashed/hung status of every
tmux pane running a ROS 2 process, across all sessions started by
resources/scripts/tmux/*.sh.

Green  - pane's shell is alive; if it's running `ros2 run`/`ros2 launch`,
         the expected node also appears in `ros2 node list`; and no
         recent [ERROR]/[FATAL] line in the pane's scrollback (see
         "Error in log" below).
Yellow - pane's shell is alive but the expected ROS node hasn't shown up
         in `ros2 node list` yet within the startup grace period (still
         "Starting"), or has gone missing after having been seen once
         while the pane itself has not exited (likely hung/crashed
         internally without killing the process), OR the node process
         is alive/registered but recently logged an [ERROR]/[FATAL] line
         ("Error in log") — e.g. a node whose startup routine (a
         move-to-home, a service call) failed internally without the
         process itself dying; rclcpy/rclpy keeps spinning regardless,
         so pure liveness/node-list checks can't see this on their own.
Red    - tmux reports the pane as dead (process exited); shows the exit
         status via tmux's pane_dead_status.

Panes titled "Scratch" (see sim_tmux_base.sh) are free-form shells with
nothing to monitor and are skipped, as are untitled panes (tmux's
default title is just the running command, indistinguishable from a
pane nobody bothered to set up for this dashboard).

Node name to watch for is derived from each pane's actual running
command, found by walking the pane shell's full descendant process
tree (not just its direct child) and skipping over wait_for_*.sh/.py
helper wrappers (see resources/scripts/shell/wait_for_node.sh and
friends) — several tmux/*.sh launch scripts chain
"wait_for_node.sh ... && ros2 run/launch ..." in one send-keys call, so
while the helper is still blocking, the real ros2 process doesn't
exist as a process yet:
  ros2 run <pkg> <node>              -> watch for "<node>"
  ros2 launch ... (no clean name)    -> no node-list check, liveness only
  still inside a wait_for_*.sh/.py   -> shown as "Waiting (prereq)",
                                         not green/stuck

Usage:
    python3 node_dashboard.py                  # poll every 2s (default)
    python3 node_dashboard.py --interval 1
    python3 node_dashboard.py --grace 15        # seconds before a
                                                 # never-appeared node
                                                 # is flagged yellow

Keys:
    up/k, down/j   move the row selection
    c              send Ctrl-C to the selected pane (SIGINT to its
                   foreground process — lets a ROS node run its
                   rclcpy/rclpy shutdown handlers)
    q              quit the dashboard (does not touch any panes)
'c' asks for a y/n confirmation before acting.
"""

import argparse
import curses
import re
import subprocess
import time

SCRATCH_TITLES = {"scratch", ""}


def sh(cmd, timeout=3.0):
    try:
        out = subprocess.run(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            timeout=timeout, text=True,
        )
        return out.stdout
    except Exception:
        return ""


PANE_FIELD_SEP = "@@FIELD@@"


def list_panes():
    # NOTE: some tmux builds sanitize literal tab/control characters
    # embedded in a -F format string down to "_" before printing (seen on
    # this project's rosject tmux — confirmed via `cat -A` showing no ^I
    # at all), so a tab delimiter silently corrupts every field boundary.
    # Use a distinctive plain-text separator instead, which tmux passes
    # through unmodified.
    fmt = PANE_FIELD_SEP.join([
        "#{session_name}", "#{window_index}", "#{pane_index}", "#{pane_id}",
        "#{pane_title}", "#{pane_pid}", "#{pane_dead}", "#{pane_dead_status}",
        "#{pane_current_command}",
    ])
    out = sh(["tmux", "list-panes", "-a", "-F", fmt])
    panes = []
    for line in out.splitlines():
        parts = line.split(PANE_FIELD_SEP)
        if len(parts) != 9:
            continue
        session, window, pane_idx, pane_id, title, pid, dead, dead_status, cmd = parts
        panes.append({
            "session": session,
            "window": window,
            "pane_idx": pane_idx,
            "pane_id": pane_id,
            "title": title,
            "pid": pid,
            "dead": dead == "1",
            "dead_status": dead_status,
            "command": cmd,
        })
    return panes


ERROR_LOG_RE = re.compile(r"\[(ERROR|FATAL)\]")


def capture_pane_tail(pane_id, lines=200):
    # -p prints to stdout instead of a tmux buffer; -S -N starts N lines
    # back from the end of the pane's history (scrollback), not just the
    # currently visible screen — a resized/scrolled pane could otherwise
    # miss a startup error that already scrolled off.
    out = sh(["tmux", "capture-pane", "-p", "-t", pane_id, "-S", f"-{lines}"])
    return out


def has_recent_error(pane_id, lines=200):
    text = capture_pane_tail(pane_id, lines=lines)
    return bool(ERROR_LOG_RE.search(text))


def send_ctrl_c(pane_id):
    # tmux send-keys C-c delivers SIGINT to the pane's foreground process
    # group, same as pressing Ctrl-C at the keyboard — lets rclpy/rclcpy
    # nodes run their shutdown handlers instead of dying uncleanly. The
    # pane/shell itself is untouched, so it's immediately reusable.
    subprocess.run(["tmux", "send-keys", "-t", pane_id, "C-c"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


WAIT_HELPER_RE = re.compile(r"\bwait_for_\S+\.(sh|py)\b")


def descendants(pid):
    # Direct child of the pane's shell isn't necessarily the real command:
    # sim_tmux_*.sh chains "wait_for_node.sh ... && ros2 run/launch ..." in
    # one send-keys call, so while the wait helper is still blocking, the
    # actual ros2 process doesn't exist as a process yet at all. Walk the
    # descendant tree and return every process's (pid, args), but do NOT
    # descend into a wait_for_*.sh/.py match's own children — those are
    # the helper's internal implementation (e.g. an inner `sleep`), not a
    # preview of the pane's eventual real command, and would otherwise be
    # misread as "the real command has started".
    frontier = [pid]
    found = []
    seen = set()
    while frontier:
        parent = frontier.pop(0)
        out = sh(["ps", "--ppid", parent, "-o", "pid=,args="])
        for line in out.splitlines():
            line = line.strip()
            if not line:
                continue
            child_pid, _, args = line.partition(" ")
            if child_pid in seen:
                continue
            seen.add(child_pid)
            args = args.strip()
            found.append((child_pid, args))
            if not WAIT_HELPER_RE.search(args):
                frontier.append(child_pid)
    return found


def pane_leaf_process(pid):
    """(pid, cmdline) of the process actually doing the work in this pane
    — skips over wait_for_*.sh/.py wrapper processes to find what they'll
    eventually exec, or returns the wrapper itself if still waiting, or
    the pane's own shell if nothing else is running (idle prompt).
    """
    found = descendants(pid)
    for child_pid, args in found:
        if not WAIT_HELPER_RE.search(args):
            return child_pid, args
    for child_pid, args in found:
        if WAIT_HELPER_RE.search(args):
            return child_pid, args
    return pid, ""


def pane_full_cmdline(pid):
    return pane_leaf_process(pid)[1]


def pane_uptime_sec(pid):
    out = sh(["ps", "-o", "etimes=", "-p", pid])
    out = out.strip()
    if out.isdigit():
        return int(out)
    return None


def fmt_duration(sec):
    if sec is None:
        return "?"
    h, rem = divmod(int(sec), 3600)
    m, s = divmod(rem, 60)
    if h:
        return f"{h:02d}:{m:02d}:{s:02d}"
    return f"{m:02d}:{s:02d}"


NODE_RUN_RE = re.compile(r"\bros2\s+run\s+\S+\s+(\S+)")
NODE_LAUNCH_RE = re.compile(r"\bros2\s+launch\s+(\S+)\s+(\S+)")


def expected_node_hint(cmdline):
    """Best-effort node name substring to grep for in `ros2 node list`,
    or None if this pane isn't a ros2 run/launch we can match that way.
    """
    m = NODE_RUN_RE.search(cmdline)
    if m:
        return m.group(1)
    m = NODE_LAUNCH_RE.search(cmdline)
    if m:
        # Launch files start N nodes; no single name to match. Return the
        # launch file stem as a best-effort hint only used for display.
        return None
    return None


def get_ros2_node_list():
    out = sh(["ros2", "node", "list"], timeout=3.0)
    return out


class PaneState:
    """Tracks first-seen time for a node hint, so a pane that starts as
    "Starting" (grace period) can flip to yellow "hung" if the node never
    shows up, rather than silently staying green-adjacent forever.
    """

    def __init__(self):
        self.first_seen_missing = {}  # pane_id -> timestamp
        self.ever_seen_alive = set()  # pane_id where node was seen at least once


def classify(pane, cmdline, node_list_text, state, now, grace_sec):
    if pane["dead"]:
        return "red", f"Exit {pane['dead_status']}", None

    if WAIT_HELPER_RE.search(cmdline):
        # Blocked on a prerequisite (wait_for_node.sh/_controllers.sh/
        # _planning_scene.sh) — the real ros2 process hasn't been execed
        # yet, so there's nothing to check in `ros2 node list` and no
        # point reporting stale state from before this pane's current
        # command started.
        state.first_seen_missing.pop(pane["pane_id"], None)
        return "yellow", "Waiting (prereq)", pane_uptime_sec(pane["pid"])

    hint = expected_node_hint(cmdline)
    if hint is None:
        # No specific node to watch — liveness only, plus the log check.
        if has_recent_error(pane["pane_id"]):
            return "yellow", "Error in log", pane_uptime_sec(pane["pid"])
        return "green", "Running", pane_uptime_sec(pane["pid"])

    node_present = bool(re.search(re.escape(hint), node_list_text))
    pane_id = pane["pane_id"]

    if node_present:
        state.ever_seen_alive.add(pane_id)
        state.first_seen_missing.pop(pane_id, None)
        if has_recent_error(pane_id):
            return "yellow", "Error in log", pane_uptime_sec(pane["pid"])
        return "green", "Running", pane_uptime_sec(pane["pid"])

    if pane_id not in state.ever_seen_alive:
        first = state.first_seen_missing.setdefault(pane_id, now)
        waited = now - first
        if waited < grace_sec:
            return "yellow", "Starting", int(waited)
        return "yellow", "No node (stuck?)", int(waited)

    # Was alive before, missing now, pane not dead -> node process likely
    # crashed internally (e.g. rclcpy exception) without tmux's shell
    # exiting, or is hung inside a callback and dropped off the graph.
    return "yellow", "Hung/Error", pane_uptime_sec(pane["pid"])


def confirm(stdscr, message):
    """Blocking y/n prompt drawn on the last line. Any key other than
    y/Y cancels — so a stray keypress can never accidentally confirm.
    """
    h, w = stdscr.getmaxyx()
    try:
        stdscr.addnstr(h - 1, 0, message.ljust(w), w, curses.A_REVERSE | curses.A_BOLD)
    except curses.error:
        pass
    stdscr.refresh()
    stdscr.nodelay(False)
    ch = stdscr.getch()
    stdscr.nodelay(True)
    return ch in (ord("y"), ord("Y"))


def draw(stdscr, interval, grace_sec):
    try:
        curses.curs_set(0)
    except curses.error:
        pass  # some terminfo entries (common in web/rosject terminals) lack this capability
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_GREEN, -1)
    curses.init_pair(2, curses.COLOR_RED, -1)
    curses.init_pair(3, curses.COLOR_YELLOW, -1)
    curses.init_pair(4, curses.COLOR_CYAN, -1)
    curses.init_pair(5, curses.COLOR_BLACK, curses.COLOR_WHITE)
    stdscr.nodelay(True)
    stdscr.keypad(True)

    state = PaneState()
    dot = {"green": "\U0001F7E2", "red": "\U0001F534", "yellow": "\U0001F7E1"}
    color = {"green": 1, "red": 2, "yellow": 3}
    selected = 0

    while True:
        now = time.time()
        panes = list_panes()
        node_list_text = get_ros2_node_list()

        rows = []
        for p in panes:
            if p["title"].strip().lower() in SCRATCH_TITLES:
                continue
            cmdline = pane_full_cmdline(p["pid"]) if not p["dead"] else ""
            status, label, extra = classify(
                p, cmdline, node_list_text, state, now, grace_sec)
            rows.append((p, status, label, extra))

        rows.sort(key=lambda r: (r[1] != "red", r[1] != "yellow", r[0]["session"]))
        if rows:
            selected = max(0, min(selected, len(rows) - 1))

        stdscr.erase()
        h, w = stdscr.getmaxyx()
        header = f" ROS 2 Node Dashboard  (tmux panes, refresh {interval:.0f}s)  {time.strftime('%H:%M:%S')} "
        stdscr.addnstr(0, 0, header.ljust(w), w, curses.color_pair(4) | curses.A_BOLD)
        stdscr.addnstr(1, 0, f"{'':2} {'NAME':22} {'SESSION':14} {'STATUS':16} {'INFO':16}", w)

        row = 2
        for idx, (p, status, label, extra) in enumerate(rows):
            if row >= h - 1:
                break
            name = p["title"] or p["command"]
            session = f"{p['session']}:{p['window']}.{p['pane_idx']}"
            if status == "red":
                info = f"pid {p['pid']}"
            elif label in ("Starting", "Hung/Error", "No node (stuck?)", "Waiting (prereq)"):
                info = f"{extra}s"
            else:
                info = fmt_duration(extra)
            marker = "▶" if idx == selected else " "
            line = f"{marker}{dot[status]} {name:22.22} {session:14.14} {label:16.16} {info:16.16}"
            attr = curses.color_pair(color[status])
            if idx == selected:
                attr |= curses.A_BOLD | curses.A_UNDERLINE
            try:
                stdscr.addnstr(row, 0, line, w, attr)
            except curses.error:
                pass
            row += 1

        if not rows:
            stdscr.addnstr(row, 0, "(no titled tmux panes found — start a session or set pane titles)", w)

        footer = " ↑/k ↓/j: select   c: Ctrl-C   q: quit "
        try:
            stdscr.addnstr(h - 1, 0, footer.ljust(w), w, curses.A_REVERSE)
        except curses.error:
            pass

        stdscr.refresh()

        deadline = time.time() + interval
        while time.time() < deadline:
            ch = stdscr.getch()
            if ch == -1:
                time.sleep(0.1)
                continue
            if ch in (ord("q"), ord("Q")):
                return
            if ch in (curses.KEY_UP, ord("k")):
                selected = max(0, selected - 1)
            elif ch in (curses.KEY_DOWN, ord("j")):
                selected = min(len(rows) - 1, selected + 1) if rows else 0
            elif ch == ord("c") and rows:
                p = rows[selected][0]
                name = p["title"] or p["command"]
                loc = f"{p['session']}:{p['window']}.{p['pane_idx']}"
                msg = f" Send Ctrl-C to '{name}' ({loc})? y/n "
                if confirm(stdscr, msg):
                    send_ctrl_c(p["pane_id"])
                break  # re-poll immediately so the effect shows right away


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--interval", type=float, default=2.0,
                         help="Poll interval in seconds (default: 2.0)")
    parser.add_argument("--grace", type=float, default=10.0,
                         help="Seconds to allow a node to appear before "
                              "flagging yellow (default: 10.0)")
    args = parser.parse_args()
    curses.wrapper(draw, args.interval, args.grace)


if __name__ == "__main__":
    main()