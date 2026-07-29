#!/bin/bash
# Read-only: dumps every unstaged change (tracked-file diffs + untracked
# file contents) under the visual_calibration repo into one text file, so
# it can be handed off for review elsewhere. Does NOT stage, commit, add,
# reset, or otherwise modify anything — pure `git diff`/`git status`/`cat`
# reads.
#
# Usage: bash dump_diff.sh [output_file]
#   Default output_file: ~/unstaged_diff_<timestamp>.txt

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
OUT_FILE="${1:-$HOME/unstaged_diff_$(date +%Y%m%d_%H%M%S).txt}"

cd "$REPO_DIR"

{
    echo "=================================================================="
    echo "Unstaged diff dump — generated $(date)"
    echo "Repo: $REPO_DIR"
    echo "=================================================================="
    echo
    echo "=== git status ==="
    git status
    echo

    echo "=== git diff (tracked files, unstaged changes) ==="
    git diff
    echo

    echo "=== git diff --stat (summary) ==="
    git diff --stat
    echo

    echo "=== Untracked files (new, never git-added) ==="
    UNTRACKED=$(git ls-files --others --exclude-standard)
    if [ -z "$UNTRACKED" ]; then
        echo "(none)"
    else
        while IFS= read -r f; do
            echo
            echo "--- $f ---"
            if file "$f" 2>/dev/null | grep -qi "text"; then
                cat "$f"
            else
                echo "(binary or unreadable file — skipped content)"
            fi
        done <<< "$UNTRACKED"
    fi
    echo
    echo "=================================================================="
    echo "End of dump"
    echo "=================================================================="
} > "$OUT_FILE"

echo "Wrote diff dump to: $OUT_FILE"
echo "Nothing was staged, committed, or modified — this only reads/writes the output file above."
