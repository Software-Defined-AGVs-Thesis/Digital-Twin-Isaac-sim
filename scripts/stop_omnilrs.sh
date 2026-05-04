#!/usr/bin/env bash
# stop_omnilrs.sh - Tear down everything start_omnilrs.sh brought up.
#
# Usage:
#   ./scripts/stop_omnilrs.sh                # prompts for sudo password if needed
#   ./scripts/stop_omnilrs.sh <password>     # password as arg
#   SUDO_PASSWORD=xxx ./scripts/stop_omnilrs.sh

set -uo pipefail

SESSION="${SESSION:-omnilrs}"
CONTAINER="${CONTAINER:-isaac-sim-omnilrs-container}"

SUDO_PASSWORD="${1:-${SUDO_PASSWORD:-}}"
if [ -z "$SUDO_PASSWORD" ] && ! sudo -n true 2>/dev/null; then
  read -rsp "[stop_omnilrs] sudo password (hidden): " SUDO_PASSWORD
  echo
fi
sudox() {
  if [ -n "${SUDO_PASSWORD:-}" ]; then
    echo "$SUDO_PASSWORD" | sudo -S -p '' "$@"
  else
    sudo "$@"
  fi
}

echo "[stop] killing tmux session '$SESSION'..."
if tmux has-session -t "$SESSION" 2>/dev/null; then
  tmux kill-session -t "$SESSION" && echo "[stop]   tmux session killed."
else
  echo "[stop]   no tmux session found."
fi

echo "[stop] stopping docker container '$CONTAINER'..."
if sudox docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
  sudox docker stop "$CONTAINER" >/dev/null && echo "[stop]   container stopped."
else
  echo "[stop]   container not running."
fi
# run_docker.sh uses --rm, so the container is removed automatically on stop.

echo "[stop] killing leftover host helpers..."
for pat in "alvr_dashboard" "vr_host_sender.py"; do
  pids=$(pgrep -f "$pat" || true)
  if [ -n "$pids" ]; then
    echo "[stop]   killing $pat (pids: $pids)"
    kill $pids 2>/dev/null || true
    sleep 1
    # force any survivors
    pids=$(pgrep -f "$pat" || true)
    [ -n "$pids" ] && kill -9 $pids 2>/dev/null || true
  fi
done

echo "[stop] cleaning up askpass temp files..."
rm -f /tmp/omnilrs-askpass-*.sh /tmp/omnilrs-askpass-*.pw 2>/dev/null

echo "[stop] done."
