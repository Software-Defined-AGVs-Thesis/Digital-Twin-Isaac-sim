#!/usr/bin/env bash
# start_omnilrs.sh - One-shot launcher for the full OmniLRS stack via tmux.
#
# Usage:
#   ./scripts/start_omnilrs.sh                # prompts for sudo password once
#   ./scripts/start_omnilrs.sh <password>     # password as arg (shows in history!)
#   SUDO_PASSWORD=xxx ./scripts/start_omnilrs.sh   # via env var
#
# Layout (tmux session: omnilrs):
#   [0] alvr        - ALVR dashboard (host)
#   [1] vr-bridge   - vr_host_sender.py (host, auto-respawn)
#   [2] container   - run_docker.sh -> Isaac Sim container shell
#   [3] isaac       - waits for container, runs run.py (Isaac Sim)
#   [4] ros-stack   - waits for prerequisites, launches SLAM + Nav2 + RViz + VR receiver
#   [5] cliff-guard - cliff_guard launch on its own (isolated logs, isolated failures)
#   [6] teleop      - interactive teleop_twist_keyboard for manual control
#   [7] explore     - explore_lite frontier exploration + forever_explore wrapper
#                     (random reachable goals when the room is fully mapped, so
#                     the robot keeps roaming forever)
#
# Attach:  tmux attach -t omnilrs
# Kill:    tmux kill-session -t omnilrs

set -euo pipefail

# ---- Sudo password handling ---------------------------------------------------
# Priority: $1 arg > $SUDO_PASSWORD env > interactive prompt.
SUDO_PASSWORD="${1:-${SUDO_PASSWORD:-}}"
if [ -z "$SUDO_PASSWORD" ]; then
  read -rsp "[start_omnilrs] sudo password (hidden): " SUDO_PASSWORD
  echo
fi
# Sanity-check: ensure the password actually works for sudo before we burn through tmux setup.
if ! echo "$SUDO_PASSWORD" | sudo -S -k -p '' true 2>/dev/null; then
  echo "[start_omnilrs] ERROR: sudo password is incorrect."; exit 1
fi
echo "[start_omnilrs] sudo password OK."

SESSION="omnilrs"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ALVR_DIR="${ALVR_DIR:-$HOME/alvr_streamer_linux}"
UDP_BRIDGE_DIR="${UDP_BRIDGE_DIR:-$HOME/UDP_Bridge}"
CONTAINER="${CONTAINER:-isaac-sim-omnilrs-container}"
CLOCK_WAIT_SECS="${CLOCK_WAIT_SECS:-15}"     # initial sleep before first /clock poll
CLOCK_POLL_INTERVAL="${CLOCK_POLL_INTERVAL:-5}"  # seconds between /clock polls
CLOCK_HEARTBEAT_EVERY="${CLOCK_HEARTBEAT_EVERY:-12}" # print waiting message every N polls (~1 minute)

command -v tmux >/dev/null || { echo "tmux not installed: sudo apt install tmux"; exit 1; }

# ---- Apply explore_lite slam_toolbox-compat QoS patch (idempotent) ------------
# explore_lite's costmap subscription uses default (VOLATILE) QoS, but
# slam_toolbox publishes /map with TRANSIENT_LOCAL durability. The QoS
# mismatch silently drops every message and explore_lite blocks forever
# on "Waiting for costmap to become available". Patch the source so the
# subscription requests TRANSIENT_LOCAL.
EXPLORE_SRC="$REPO_DIR/jetauto/m-explore-ros2/explore/src/costmap_client.cpp"
if [ -f "$EXPLORE_SRC" ]; then
  if grep -q 'map_qos.transient_local' "$EXPLORE_SRC"; then
    echo "[start_omnilrs] explore_lite QoS patch already applied."
  else
    echo "[start_omnilrs] applying explore_lite TRANSIENT_LOCAL QoS patch..."
    python3 - "$EXPLORE_SRC" <<'PYPATCH'
import sys
p = sys.argv[1]
src = open(p).read()
old = '  /* initialize costmap */\n  costmap_sub_ = node_.create_subscription<nav_msgs::msg::OccupancyGrid>(\n      costmap_topic, 1000,'
new = (
    '  /* initialize costmap (slam_toolbox compat: TRANSIENT_LOCAL) */\n'
    '  rclcpp::QoS map_qos(rclcpp::KeepLast(1));\n'
    '  map_qos.transient_local();\n'
    '  map_qos.reliable();\n'
    '  costmap_sub_ = node_.create_subscription<nav_msgs::msg::OccupancyGrid>(\n'
    '      costmap_topic, map_qos,'
)
if old not in src:
    sys.stderr.write("WARN: anchor not found in costmap_client.cpp; skipping patch\n")
    sys.exit(0)
open(p, 'w').write(src.replace(old, new))
print("  patched.")
PYPATCH
    # Force a rebuild of explore_lite next time (clears cached object files).
    rm -rf "$REPO_DIR/build/explore_lite" "$REPO_DIR/install/explore_lite" 2>/dev/null || true
  fi
else
  echo "[start_omnilrs] NOTE: explore_lite source not found ($EXPLORE_SRC). Run 'git submodule update --init' if you want autonomous exploration."
fi

if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "Session '$SESSION' already exists. Attach with: tmux attach -t $SESSION"
  echo "Or kill it: tmux kill-session -t $SESSION"
  exit 1
fi

# No session exists yet, so any leftover askpass files are stale (from an
# interrupted previous run). Remove them BEFORE creating new ones.
rm -f /tmp/omnilrs-askpass-*.sh /tmp/omnilrs-askpass-*.pw 2>/dev/null || true

# Set up a SUDO_ASKPASS helper so panes can call `sudo -A` without piping into
# stdin (stdin must stay free for `docker exec -it` and similar). The helper
# script just prints the password; both files are mode 600/700 in /tmp.
ASKPASS_SCRIPT="/tmp/omnilrs-askpass-$$.sh"
ASKPASS_PWFILE="/tmp/omnilrs-askpass-$$.pw"
umask 077
printf '%s' "$SUDO_PASSWORD" > "$ASKPASS_PWFILE"
cat > "$ASKPASS_SCRIPT" <<EOF
#!/usr/bin/env bash
cat "$ASKPASS_PWFILE"
EOF
chmod 700 "$ASKPASS_SCRIPT"
echo "[start_omnilrs] askpass helper at $ASKPASS_SCRIPT (cleanup happens in stop_omnilrs.sh)"

# Helper snippet that each pane's bash sources to define passwordless sudo via -A.
SUDO_HELPER="export SUDO_ASKPASS=${ASKPASS_SCRIPT}; sudox() { sudo -A \"\$@\"; }"

# ---- Window 0: ALVR dashboard --------------------------------------------------
tmux new-session -d -s "$SESSION" -n alvr -c "$ALVR_DIR" \
  "echo '[ALVR] starting dashboard...'; ./bin/alvr_dashboard; exec bash"

# ---- Window 1: VR UDP bridge (host, auto-respawn until SteamVR is reachable) --
tmux new-window -t "$SESSION" -n vr-bridge -c "$UDP_BRIDGE_DIR" "bash -c '
  echo \"[VR-BRIDGE] launching vr_host_sender.py with auto-respawn (waits for SteamVR)...\"
  attempt=0
  while true; do
    attempt=\$((attempt + 1))
    echo \"[VR-BRIDGE] attempt #\$attempt — starting vr_host_sender.py\"
    python3 vr_host_sender.py
    rc=\$?
    echo \"[VR-BRIDGE] exited with code \$rc — retrying in 5s (Ctrl-C to stop)\"
    sleep 5
  done
'"

# ---- Window 2: Container shell (run_docker.sh) --------------------------------
tmux new-window -t "$SESSION" -n container -c "$REPO_DIR" "bash -c '
  ${SUDO_HELPER}
  echo \"[CONTAINER] launching docker (sudo)...\"
  sudox ./omnilrs.docker/run_docker.sh
  exec bash
'"

# ---- Window 3: Isaac Sim (waits for container, then runs run.py) --------------
tmux new-window -t "$SESSION" -n isaac -c "$REPO_DIR" "bash -c '
  ${SUDO_HELPER}
  echo \"[ISAAC] waiting for container ${CONTAINER} ...\"
  for i in \$(seq 1 120); do
    if sudox docker ps --format \"{{.Names}}\" | grep -q \"^${CONTAINER}\$\"; then
      echo \"[ISAAC] container is up.\"; break
    fi
    sleep 2
  done
  echo \"[ISAAC] waiting for container apt-install to finish before starting Isaac Sim...\"
  while sudox docker exec ${CONTAINER} pgrep -f apt-get >/dev/null 2>&1; do
    echo \"[ISAAC]   apt still running, sleeping 10s...\"
    sleep 10
  done
  echo \"[ISAAC] starting Isaac Sim (cwd=/workspace/omnilrs)...\"
  sudox docker exec -it -w /workspace/omnilrs ${CONTAINER} /isaac-sim/python.sh run.py
  exec bash
'"

# ---- Window 4: ROS stack (waits for prerequisites, then launches) -------------
# Stable order: container up -> apt-install done -> /clock publishing -> launch.
# No timeouts: the ROS stack will simply wait until everything is ready.
tmux new-window -t "$SESSION" -n ros-stack -c "$REPO_DIR" "bash -c '
  ${SUDO_HELPER}
  # NOTE: prepend system lib path so rcl_logging_spdlog loads the apt libspdlog,
  # not the incompatible one Isaac Sim ships under /isaac-sim/exts/.../humble/lib.
  ROS_PRELUDE=\"export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:\\\$LD_LIBRARY_PATH && source /opt/ros/humble/setup.bash && export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:\\\$LD_LIBRARY_PATH\"

  # ---- Respawn loop ----
  # If the container is restarted (manually, by Docker daemon, or by an OOM)
  # the docker exec session below dies and the launch exits. Without this
  # loop the bash here would also exit, tmux would close the window, and
  # the user would lose SLAM/Nav2/RViz/VR-receiver until they manually
  # respawn. Looping keeps the window alive across container churn — the
  # wait-for-container + wait-for-/clock blocks below handle the gap, and
  # the colcon-build / symlink-heal blocks are idempotent (no-op once
  # packages are built).
  ATTEMPT=0
  while true; do
  ATTEMPT=\$((ATTEMPT + 1))
  if [ \$ATTEMPT -gt 1 ]; then
    echo \"\"
    echo \"[ROS-STACK] >>> respawn attempt #\$ATTEMPT (previous launch died) <<<\"
  fi

  echo \"[ROS-STACK] waiting for container ${CONTAINER} ...\"
  while ! sudox docker ps --format \"{{.Names}}\" | grep -q \"^${CONTAINER}\$\"; do
    sleep 2
  done
  echo \"[ROS-STACK] container is up.\"

  echo \"[ROS-STACK] waiting for in-container apt-install to finish...\"
  while sudox docker exec ${CONTAINER} pgrep -f apt-get >/dev/null 2>&1 || sudox docker exec ${CONTAINER} pgrep -f dpkg >/dev/null 2>&1; do
    sleep 10
  done
  echo \"[ROS-STACK] apt-install done.\"

  # Auto-build any workspace package whose install/ dir is missing. This makes
  # a fresh checkout / cleared install/ recoverable without manual colcon build.
  echo \"[ROS-STACK] checking workspace build state...\"
  MISSING=\"\"
  for pkg in jetauto_description jetauto_bringup cliff_guard explore_lite_msgs explore_lite forever_explore; do
    if ! sudox docker exec ${CONTAINER} test -d /workspace/omnilrs/install/\$pkg; then
      MISSING=\"\$MISSING \$pkg\"
    fi
  done
  if [ -n \"\$MISSING\" ]; then
    echo \"[ROS-STACK] missing packages: \$MISSING\"

    # rosdep: pull any apt deps the workspace declares (e.g. nav2_costmap_2d-dev
    # for explore_lite). Idempotent — safe to run on every boot. We init only
    # once (rosdep init creates /etc/ros/rosdep/sources.list.d/*).
    echo \"[ROS-STACK] running rosdep for missing packages...\"
    sudox docker exec ${CONTAINER} bash -lc \"
      set -e
      if ! command -v rosdep >/dev/null 2>&1; then
        echo \"[ROS-STACK] installing python3-rosdep...\"
        apt-get update -qq && apt-get install -y python3-rosdep >/dev/null
      fi
      if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        rosdep init >/dev/null 2>&1 || true
      fi
      rosdep update --rosdistro humble >/dev/null
      cd /workspace/omnilrs &&
      rosdep install --from-paths jetauto --ignore-src -r -y --rosdistro humble
    \" || echo \"[ROS-STACK] rosdep step failed — continuing; colcon may still succeed.\"

    # IMPORTANT: prepend system lib path so the linker picks the apt libspdlog/
    # fmt that librcl_logging_spdlog.so was compiled against, not the
    # incompatible ones Isaac Sim ships under /isaac-sim/exts/.../humble/lib.
    # Without this, explore_lite fails to link with an undefined-reference
    # error against fmt::v8.
    echo \"[ROS-STACK] running colcon build (with LD_LIBRARY_PATH fixup)...\"
    sudox docker exec ${CONTAINER} bash -lc \"
      export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:\\\$LD_LIBRARY_PATH &&
      source /opt/ros/humble/setup.bash &&
      export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:\\\$LD_LIBRARY_PATH &&
      cd /workspace/omnilrs &&
      colcon build --packages-select\$MISSING --cmake-args -DCMAKE_BUILD_RPATH='/usr/lib/x86_64-linux-gnu' -DCMAKE_INSTALL_RPATH='/usr/lib/x86_64-linux-gnu'
    \"
    echo \"[ROS-STACK] colcon build finished.\"
  else
    echo \"[ROS-STACK] all packages already built.\"
  fi

  # Heal cliff_guard executable layout: newer setuptools installs console_scripts
  # to install/cliff_guard/bin/, but ROS2 looks in install/cliff_guard/lib/cliff_guard/.
  echo \"[ROS-STACK] verifying cliff_guard executable layout...\"
  sudox docker exec ${CONTAINER} bash -lc '\''
    BIN=/workspace/omnilrs/install/cliff_guard/bin
    LIB=/workspace/omnilrs/install/cliff_guard/lib/cliff_guard
    if [ -d \"\$BIN\" ]; then
      mkdir -p \"\$LIB\"
      for exe in cliff_guard vr_override handover_popup; do
        if [ -f \"\$BIN/\$exe\" ] && [ ! -e \"\$LIB/\$exe\" ]; then
          ln -sf \"\$BIN/\$exe\" \"\$LIB/\$exe\"
          echo \"[ROS-STACK]   linked \$LIB/\$exe\"
        fi
      done
    else
      echo \"[ROS-STACK]   NOTE: \$BIN missing — has cliff_guard been built?\"
    fi
  '\''

  echo \"[ROS-STACK] settling ${CLOCK_WAIT_SECS}s before first /clock poll...\"
  sleep ${CLOCK_WAIT_SECS}

  echo \"[ROS-STACK] waiting for /clock (no timeout) ...\"
  poll=0
  while true; do
    if sudox docker exec ${CONTAINER} bash -lc \"\$ROS_PRELUDE && ros2 topic list 2>/dev/null | grep -q ^/clock\$\"; then
      echo \"[ROS-STACK] /clock detected after \$poll polls.\"
      break
    fi
    poll=\$((poll + 1))
    if [ \$((poll % ${CLOCK_HEARTBEAT_EVERY})) -eq 0 ]; then
      echo \"[ROS-STACK] still waiting for /clock... (\$((poll * ${CLOCK_POLL_INTERVAL}))s elapsed) — is Isaac Sim up in window 3?\"
    fi
    sleep ${CLOCK_POLL_INTERVAL}
  done

  echo \"[ROS-STACK] launching omnilrs_stack.launch.py ...\"
  sudox docker exec -it ${CONTAINER} bash -lc \"
    \$ROS_PRELUDE &&
    source /workspace/omnilrs/install/local_setup.bash &&
    ros2 launch jetauto_bringup omnilrs_stack.launch.py
  \" || true
  echo \"[ROS-STACK] launch exited (container restart or crash). Respawning in 5s — Ctrl-C to stop.\"
  sleep 5
  done
'"

# ---- Window 5: cliff_guard (its own pane so its logs are isolated) ------------
# Waits for /clock + 30s extra so SLAM/Nav2/RViz come up first, then runs
# cliff_guard.launch.py standalone. A crash here does NOT kill the rest.
tmux new-window -t "$SESSION" -n cliff-guard -c "$REPO_DIR" "bash -c '
  ${SUDO_HELPER}
  ROS_PRELUDE=\"export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:\\\$LD_LIBRARY_PATH && source /opt/ros/humble/setup.bash && export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:\\\$LD_LIBRARY_PATH\"

  # Same respawn loop as window 4 — survives container restarts.
  ATTEMPT=0
  while true; do
  ATTEMPT=\$((ATTEMPT + 1))
  if [ \$ATTEMPT -gt 1 ]; then
    echo \"\"
    echo \"[CLIFF-GUARD] >>> respawn attempt #\$ATTEMPT (previous launch died) <<<\"
  fi

  echo \"[CLIFF-GUARD] waiting for /clock ...\"
  while ! sudox docker exec ${CONTAINER} bash -lc \"\$ROS_PRELUDE && ros2 topic list 2>/dev/null | grep -q ^/clock\$\" 2>/dev/null; do
    sleep 5
  done
  echo \"[CLIFF-GUARD] /clock detected.\"

  # Race-guard: wait for the cliff_guard executable to be in place. The
  # ros-stack window builds it (if missing) and heals symlinks; we may
  # arrive here before either has run.
  echo \"[CLIFF-GUARD] waiting for cliff_guard executable to be ready...\"
  while ! sudox docker exec ${CONTAINER} test -e /workspace/omnilrs/install/cliff_guard/lib/cliff_guard/cliff_guard 2>/dev/null; do
    sleep 5
  done
  echo \"[CLIFF-GUARD] executable ready.\"

  echo \"[CLIFF-GUARD] sleeping 30s so SLAM/Nav2/RViz start first...\"
  sleep 30

  echo \"[CLIFF-GUARD] launching cliff_guard.launch.py ...\"
  sudox docker exec -it ${CONTAINER} bash -lc \"
    \$ROS_PRELUDE &&
    source /workspace/omnilrs/install/local_setup.bash &&
    ros2 launch cliff_guard cliff_guard.launch.py
  \" || true
  echo \"[CLIFF-GUARD] launch exited (container restart or crash). Respawning in 5s — Ctrl-C to stop.\"
  sleep 5
  done
'"

# ---- Window 6: teleop keyboard (interactive, manual robot control) ------------
# Drops you into teleop_twist_keyboard. Switch here (Ctrl-b 6) and use the keys
# (i j k l u o m , . to move; q/z to change speed) to drive the robot.
tmux new-window -t "$SESSION" -n teleop -c "$REPO_DIR" "bash -c '
  ${SUDO_HELPER}
  ROS_PRELUDE=\"export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:\\\$LD_LIBRARY_PATH && source /opt/ros/humble/setup.bash && export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:\\\$LD_LIBRARY_PATH\"

  echo \"[TELEOP] waiting for /clock ...\"
  while ! sudox docker exec ${CONTAINER} bash -lc \"\$ROS_PRELUDE && ros2 topic list 2>/dev/null | grep -q ^/clock\$\" 2>/dev/null; do
    sleep 5
  done
  echo \"[TELEOP] sleeping 40s so the rest of the stack starts first...\"
  sleep 40

  echo \"[TELEOP] starting teleop_twist_keyboard. Switch to this window (Ctrl-b 6) and use the keys to drive.\"
  sudox docker exec -it ${CONTAINER} bash -lc \"
    \$ROS_PRELUDE &&
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
  \"
  exec bash
'"

# ---- Window 7: forever exploration (explore_lite + forever_explore) -----------
# Starts smart frontier-based exploration (explore_lite). When the room is
# fully mapped, our forever_explore companion node clears the global costmap
# and dispatches a random reachable goal so the robot never stops moving.
# Both produce loud banner logs you can follow live in this window.
tmux new-window -t "$SESSION" -n explore -c "$REPO_DIR" "bash -c '
  ${SUDO_HELPER}
  ROS_PRELUDE=\"export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:\\\$LD_LIBRARY_PATH && source /opt/ros/humble/setup.bash && export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:\\\$LD_LIBRARY_PATH\"

  # Same respawn loop as window 4 — survives container restarts.
  ATTEMPT=0
  while true; do
  ATTEMPT=\$((ATTEMPT + 1))
  if [ \$ATTEMPT -gt 1 ]; then
    echo \"\"
    echo \"[EXPLORE] >>> respawn attempt #\$ATTEMPT (previous launch died) <<<\"
  fi

  echo \"[EXPLORE] waiting for /clock ...\"
  while ! sudox docker exec ${CONTAINER} bash -lc \"\$ROS_PRELUDE && ros2 topic list 2>/dev/null | grep -q ^/clock\$\" 2>/dev/null; do
    sleep 5
  done
  echo \"[EXPLORE] /clock detected.\"

  echo \"[EXPLORE] waiting for forever_explore executable to be built...\"
  while ! sudox docker exec ${CONTAINER} test -e /workspace/omnilrs/install/forever_explore/lib/forever_explore/forever_explore 2>/dev/null; do
    sleep 5
  done

  echo \"[EXPLORE] waiting for /navigate_to_pose to appear in graph ...\"
  while ! sudox docker exec ${CONTAINER} bash -lc \"\$ROS_PRELUDE && ros2 action list 2>/dev/null | grep -q ^/navigate_to_pose\$\" 2>/dev/null; do
    sleep 5
  done

  # The action being in the graph is NOT enough — Nav2 bt_navigator must
  # also be the action SERVER. If lifecycle_manager_navigation hits its
  # smoother_server configure-timeout (CPU stress at boot), nodes get stuck
  # unconfigured/inactive and the server never registers. Detect this and
  # heal by manually transitioning each node configure then activate.
  # NOTE: literal single-quotes inside this bash -c block are forbidden —
  # they would prematurely close the outer quote. Use \"...\" everywhere.
  echo \"[EXPLORE] verifying Nav2 lifecycle (action server registered)...\"
  HEAL_SCRIPT=\"for n in controller_server smoother_server planner_server behavior_server velocity_smoother bt_navigator waypoint_follower; do
      state=\\\$(timeout 4 ros2 lifecycle get /\\\$n 2>&1 | tail -1)
      case \\\"\\\$state\\\" in *unconfigured*) timeout 10 ros2 lifecycle set /\\\$n configure >/dev/null 2>&1 ;; esac
      state=\\\$(timeout 4 ros2 lifecycle get /\\\$n 2>&1 | tail -1)
      case \\\"\\\$state\\\" in *inactive*) timeout 10 ros2 lifecycle set /\\\$n activate >/dev/null 2>&1 ;; esac
      final=\\\$(timeout 4 ros2 lifecycle get /\\\$n 2>&1 | tail -1)
      printf \\\"[EXPLORE]   %-20s %s\\\\n\\\" \\\"\\\$n\\\" \\\"\\\$final\\\"
    done\"
  for attempt in 1 2 3 4 5 6; do
    RAW=\$(sudo -A docker exec ${CONTAINER} bash -lc \"\$ROS_PRELUDE && ros2 action info /navigate_to_pose -t 2>/dev/null\" 2>/dev/null)
    SERVERS=\$(echo \"\$RAW\" | awk -v key=\"Action servers:\" \"index(\\\$0,key){print \\\$3}\" | tr -d \" \\t\\n\")
    if [ -n \"\$SERVERS\" ] && [ \"\$SERVERS\" -gt 0 ] 2>/dev/null; then
      echo \"[EXPLORE] Nav2 action server registered (\$SERVERS server(s)).\"
      break
    fi
    echo \"[EXPLORE] attempt \$attempt: 0 action servers — running Nav2 lifecycle self-heal...\"
    sudo -A docker exec ${CONTAINER} bash -lc \"\$ROS_PRELUDE && \$HEAL_SCRIPT\"
    sleep 5
  done
  if [ \"\$SERVERS\" = \"\" ] || [ \"\$SERVERS\" = \"0\" ]; then
    echo \"[EXPLORE] WARNING: Nav2 still has no action server after self-heal attempts.\"
    echo \"[EXPLORE]          forever_explore will run anyway — random goals will\"
    echo \"[EXPLORE]          be rejected until you fix Nav2 manually. Inspect window 4.\"
  fi

  echo \"[EXPLORE] sleeping 10s so costmaps fully populate before launching ...\"
  sleep 10

  echo \"[EXPLORE] launching explore_lite + forever_explore ...\"
  sudox docker exec -it ${CONTAINER} bash -lc \"
    \$ROS_PRELUDE &&
    source /workspace/omnilrs/install/local_setup.bash &&
    ros2 launch forever_explore forever_explore.launch.py
  \" || true
  echo \"[EXPLORE] launch exited (container restart or crash). Respawning in 5s — Ctrl-C to stop.\"
  sleep 5
  done
'"

tmux select-window -t "$SESSION":container
echo "Started tmux session '$SESSION'."
echo "Kill: tmux kill-session -t $SESSION  (or ./scripts/stop_omnilrs.sh)"

# Auto-attach when run from a real terminal. Skip via: NO_ATTACH=1 ./start_omnilrs.sh
if [ -z "${NO_ATTACH:-}" ] && [ -t 0 ] && [ -t 1 ]; then
  echo "Attaching to tmux session (Ctrl-b d to detach, NO_ATTACH=1 to skip)..."
  exec tmux attach -t "$SESSION"
else
  echo "Attach manually: tmux attach -t $SESSION"
fi
