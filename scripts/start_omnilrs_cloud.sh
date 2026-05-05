#!/usr/bin/env bash
# start_omnilrs_cloud.sh - Cloud launcher for OmniLRS stack on EC2.
#
# Differences from start_omnilrs.sh (local):
#   - Window 0 (ALVR) and Window 1 (vr-bridge) are REMOVED.
#     Run those on your Earth PC as normal using start_omnilrs.sh.
#   - Window 2 (container) uses EC2-compatible docker run (no X11/Steam/OpenXR mounts).
#     X11 forwarding is handled via ssh -X, not host paths.
#   - Everything else (windows 3-7) is identical to the local launcher.
#
# PRE-REQUISITE: SSH into EC2 with X11 forwarding before running this:
#   ssh -X ubuntu@10.10.8.159
#
# Usage:
#   ./scripts/start_omnilrs_cloud.sh                # prompts for sudo password once
#   ./scripts/start_omnilrs_cloud.sh <password>     # password as arg (shows in history!)
#   SUDO_PASSWORD=xxx ./scripts/start_omnilrs_cloud.sh   # via env var
#
# Layout (tmux session: omnilrs):
#   [0] container   - docker run (EC2 version, no X11/Steam/OpenXR host paths)
#   [1] isaac       - waits for container, runs run.py (Isaac Sim)
#   [2] ros-stack   - waits for prerequisites, launches SLAM + Nav2 + RViz + VR receiver
#   [3] cliff-guard - cliff_guard launch on its own (isolated logs, isolated failures)
#   [4] stuck-guard - stuck_guard launch on its own (wheels-spinning-but-pose-static detector)
#   [5] teleop      - interactive teleop_twist_keyboard for manual control
#   [6] explore     - explore_lite frontier exploration + forever_explore wrapper
#
# Attach:  tmux attach -t omnilrs
# Kill:    tmux kill-session -t omnilrs

set -euo pipefail

# ---- Sudo password handling ---------------------------------------------------
SUDO_PASSWORD="${1:-${SUDO_PASSWORD:-}}"
if [ -z "$SUDO_PASSWORD" ]; then
  read -rsp "[start_omnilrs_cloud] sudo password (hidden): " SUDO_PASSWORD
  echo
fi
if ! echo "$SUDO_PASSWORD" | sudo -S -k -p '' true 2>/dev/null; then
  echo "[start_omnilrs_cloud] ERROR: sudo password is incorrect."; exit 1
fi
echo "[start_omnilrs_cloud] sudo password OK."

SESSION="omnilrs"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER="${CONTAINER:-isaac-sim-omnilrs-container}"
CLOCK_WAIT_SECS="${CLOCK_WAIT_SECS:-15}"
CLOCK_POLL_INTERVAL="${CLOCK_POLL_INTERVAL:-5}"
CLOCK_HEARTBEAT_EVERY="${CLOCK_HEARTBEAT_EVERY:-12}"

command -v tmux >/dev/null || { echo "tmux not installed: sudo apt install tmux"; exit 1; }

# ---- Apply explore_lite slam_toolbox-compat QoS patch (idempotent) ------------
EXPLORE_SRC="$REPO_DIR/jetauto/m-explore-ros2/explore/src/costmap_client.cpp"
if [ -f "$EXPLORE_SRC" ]; then
  if grep -q 'map_qos.transient_local' "$EXPLORE_SRC"; then
    echo "[start_omnilrs_cloud] explore_lite QoS patch already applied."
  else
    echo "[start_omnilrs_cloud] applying explore_lite TRANSIENT_LOCAL QoS patch..."
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
    rm -rf "$REPO_DIR/build/explore_lite" "$REPO_DIR/install/explore_lite" 2>/dev/null || true
  fi
else
  echo "[start_omnilrs_cloud] NOTE: explore_lite source not found ($EXPLORE_SRC). Run 'git submodule update --init' if you want autonomous exploration."
fi

if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "Session '$SESSION' already exists. Attach with: tmux attach -t $SESSION"
  echo "Or kill it: tmux kill-session -t $SESSION"
  exit 1
fi

rm -f /tmp/omnilrs-askpass-*.sh /tmp/omnilrs-askpass-*.pw 2>/dev/null || true

ASKPASS_SCRIPT="/tmp/omnilrs-askpass-$$.sh"
ASKPASS_PWFILE="/tmp/omnilrs-askpass-$$.pw"
umask 077
printf '%s' "$SUDO_PASSWORD" > "$ASKPASS_PWFILE"
cat > "$ASKPASS_SCRIPT" <<EOF
#!/usr/bin/env bash
cat "$ASKPASS_PWFILE"
EOF
chmod 700 "$ASKPASS_SCRIPT"
echo "[start_omnilrs_cloud] askpass helper at $ASKPASS_SCRIPT"

SUDO_HELPER="export SUDO_ASKPASS=${ASKPASS_SCRIPT}; sudox() { sudo -A \"\$@\"; }"

# ---- Virtual display setup (Xvfb) --------------------------------------------
# EC2 has no monitor. We create a virtual display so Isaac Sim can render.
# If Xvfb is not installed: sudo apt-get install -y xvfb
echo "[start_omnilrs_cloud] setting up virtual display..."
if ! command -v Xvfb >/dev/null 2>&1; then
  echo "[start_omnilrs_cloud] installing Xvfb..."
  sudo apt-get install -y xvfb x11-utils >/dev/null
fi
# Kill any existing Xvfb on :1
pkill -f "Xvfb :1" 2>/dev/null || true
sleep 1
Xvfb :1 -screen 0 1920x1080x24 &
export DISPLAY=:1
sleep 2
echo "[start_omnilrs_cloud] virtual display started on :1"

# ---- Window 0: Container (EC2 version — no X11 host paths, no Steam, no OpenXR) ----
# Uses Xvfb virtual display :1 started above.
tmux new-session -d -s "$SESSION" -n container -c "$REPO_DIR" "bash -c '
  ${SUDO_HELPER}
  echo \"[CONTAINER] launching docker (EC2 cloud mode)...\"
  sudox docker run \
    --name ${CONTAINER} \
    -it \
    --gpus all \
    --rm \
    --network=host \
    --ipc=host \
    -e ACCEPT_EULA=Y \
    -e PRIVACY_CONSENT=Y \
    -e DISPLAY=:1 \
    -e ROS_DOMAIN_ID=0 \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v \$HOME/.Xauthority:/root/.Xauthority \
    -v ${REPO_DIR}:/workspace/omnilrs \
    -v \$HOME/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
    -v \$HOME/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
    -v \$HOME/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
    -v \$HOME/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
    -v \$HOME/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
    -v \$HOME/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
    -v \$HOME/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
    -v \$HOME/docker/isaac-sim/documents:/root/Documents:rw \
    -v bash_command_history:/commandhistory \
    isaac-sim-omnilrs:latest
  exec bash
'"

# ---- Window 1: Isaac Sim (waits for container, then runs run.py) --------------
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

# ---- Window 2: ROS stack (waits for prerequisites, then launches) -------------
tmux new-window -t "$SESSION" -n ros-stack -c "$REPO_DIR" "bash -c '
  ${SUDO_HELPER}
  ROS_PRELUDE=\"export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:\\\$LD_LIBRARY_PATH && source /opt/ros/humble/setup.bash && export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:\\\$LD_LIBRARY_PATH\"

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

  echo \"[ROS-STACK] checking workspace build state...\"
  MISSING=\"\"
  for pkg in jetauto_description jetauto_bringup cliff_guard stuck_guard explore_lite_msgs explore_lite forever_explore; do
    if ! sudox docker exec ${CONTAINER} test -d /workspace/omnilrs/install/\$pkg; then
      MISSING=\"\$MISSING \$pkg\"
    fi
  done
  if [ -n \"\$MISSING\" ]; then
    echo \"[ROS-STACK] missing packages: \$MISSING\"
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
    \" || echo \"[ROS-STACK] rosdep step failed — continuing.\"

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
    fi
  '\''

  # stuck_guard ships a setup.cfg so console_scripts go to lib/, but heal anyway.
  echo \"[ROS-STACK] verifying stuck_guard executable layout...\"
  sudox docker exec ${CONTAINER} bash -lc '\''
    BIN=/workspace/omnilrs/install/stuck_guard/bin
    LIB=/workspace/omnilrs/install/stuck_guard/lib/stuck_guard
    if [ -d \"\$BIN\" ]; then
      mkdir -p \"\$LIB\"
      for exe in stuck_guard; do
        if [ -f \"\$BIN/\$exe\" ] && [ ! -e \"\$LIB/\$exe\" ]; then
          ln -sf \"\$BIN/\$exe\" \"\$LIB/\$exe\"
          echo \"[ROS-STACK]   linked \$LIB/\$exe\"
        fi
      done
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
      echo \"[ROS-STACK] still waiting for /clock... (\$((poll * ${CLOCK_POLL_INTERVAL}))s elapsed)\"
    fi
    sleep ${CLOCK_POLL_INTERVAL}
  done

  echo \"[ROS-STACK] launching omnilrs_stack.launch.py ...\"
  sudox docker exec -it ${CONTAINER} bash -lc \"
    \$ROS_PRELUDE &&
    source /workspace/omnilrs/install/local_setup.bash &&
    ros2 launch jetauto_bringup omnilrs_stack.launch.py
  \" || true
  echo \"[ROS-STACK] launch exited. Respawning in 5s — Ctrl-C to stop.\"
  sleep 5
  done
'"

# ---- Window 3: cliff_guard ----------------------------------------------------
tmux new-window -t "$SESSION" -n cliff-guard -c "$REPO_DIR" "bash -c '
  ${SUDO_HELPER}
  ROS_PRELUDE=\"export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:\\\$LD_LIBRARY_PATH && source /opt/ros/humble/setup.bash && export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:\\\$LD_LIBRARY_PATH\"

  ATTEMPT=0
  while true; do
  ATTEMPT=\$((ATTEMPT + 1))
  if [ \$ATTEMPT -gt 1 ]; then
    echo \"[CLIFF-GUARD] >>> respawn attempt #\$ATTEMPT <<<\"
  fi

  echo \"[CLIFF-GUARD] waiting for /clock ...\"
  while ! sudox docker exec ${CONTAINER} bash -lc \"\$ROS_PRELUDE && ros2 topic list 2>/dev/null | grep -q ^/clock\$\" 2>/dev/null; do
    sleep 5
  done

  echo \"[CLIFF-GUARD] waiting for cliff_guard executable...\"
  while ! sudox docker exec ${CONTAINER} test -e /workspace/omnilrs/install/cliff_guard/lib/cliff_guard/cliff_guard 2>/dev/null; do
    sleep 5
  done

  echo \"[CLIFF-GUARD] sleeping 30s so SLAM/Nav2/RViz start first...\"
  sleep 30

  echo \"[CLIFF-GUARD] launching cliff_guard.launch.py ...\"
  sudox docker exec -it ${CONTAINER} bash -lc \"
    \$ROS_PRELUDE &&
    source /workspace/omnilrs/install/local_setup.bash &&
    ros2 launch cliff_guard cliff_guard.launch.py
  \" || true
  echo \"[CLIFF-GUARD] launch exited. Respawning in 5s — Ctrl-C to stop.\"
  sleep 5
  done
'"

# ---- Window 4: stuck_guard ----------------------------------------------------
tmux new-window -t "$SESSION" -n stuck-guard -c "$REPO_DIR" "bash -c '
  ${SUDO_HELPER}
  ROS_PRELUDE=\"export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:\\\$LD_LIBRARY_PATH && source /opt/ros/humble/setup.bash && export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:\\\$LD_LIBRARY_PATH\"

  ATTEMPT=0
  while true; do
  ATTEMPT=\$((ATTEMPT + 1))
  if [ \$ATTEMPT -gt 1 ]; then
    echo \"[STUCK-GUARD] >>> respawn attempt #\$ATTEMPT <<<\"
  fi

  echo \"[STUCK-GUARD] waiting for /clock ...\"
  while ! sudox docker exec ${CONTAINER} bash -lc \"\$ROS_PRELUDE && ros2 topic list 2>/dev/null | grep -q ^/clock\$\" 2>/dev/null; do
    sleep 5
  done

  echo \"[STUCK-GUARD] waiting for stuck_guard executable...\"
  while ! sudox docker exec ${CONTAINER} test -e /workspace/omnilrs/install/stuck_guard/lib/stuck_guard/stuck_guard 2>/dev/null; do
    sleep 5
  done

  echo \"[STUCK-GUARD] sleeping 30s so SLAM/Nav2/RViz start first...\"
  sleep 30

  echo \"[STUCK-GUARD] launching stuck_guard.launch.py ...\"
  sudox docker exec -it ${CONTAINER} bash -lc \"
    \$ROS_PRELUDE &&
    source /workspace/omnilrs/install/local_setup.bash &&
    ros2 launch stuck_guard stuck_guard.launch.py
  \" || true
  echo \"[STUCK-GUARD] launch exited. Respawning in 5s — Ctrl-C to stop.\"
  sleep 5
  done
'"

# ---- Window 5: teleop ---------------------------------------------------------
tmux new-window -t "$SESSION" -n teleop -c "$REPO_DIR" "bash -c '
  ${SUDO_HELPER}
  ROS_PRELUDE=\"export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:\\\$LD_LIBRARY_PATH && source /opt/ros/humble/setup.bash && export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:\\\$LD_LIBRARY_PATH\"

  echo \"[TELEOP] waiting for /clock ...\"
  while ! sudox docker exec ${CONTAINER} bash -lc \"\$ROS_PRELUDE && ros2 topic list 2>/dev/null | grep -q ^/clock\$\" 2>/dev/null; do
    sleep 5
  done
  echo \"[TELEOP] sleeping 40s so the rest of the stack starts first...\"
  sleep 40

  echo \"[TELEOP] starting teleop_twist_keyboard.\"
  sudox docker exec -it ${CONTAINER} bash -lc \"
    \$ROS_PRELUDE &&
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
  \"
  exec bash
'"

# ---- Window 5: explore --------------------------------------------------------
tmux new-window -t "$SESSION" -n explore -c "$REPO_DIR" "bash -c '
  ${SUDO_HELPER}
  ROS_PRELUDE=\"export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:\\\$LD_LIBRARY_PATH && source /opt/ros/humble/setup.bash && export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:\\\$LD_LIBRARY_PATH\"

  ATTEMPT=0
  while true; do
  ATTEMPT=\$((ATTEMPT + 1))
  if [ \$ATTEMPT -gt 1 ]; then
    echo \"[EXPLORE] >>> respawn attempt #\$ATTEMPT <<<\"
  fi

  echo \"[EXPLORE] waiting for /clock ...\"
  while ! sudox docker exec ${CONTAINER} bash -lc \"\$ROS_PRELUDE && ros2 topic list 2>/dev/null | grep -q ^/clock\$\" 2>/dev/null; do
    sleep 5
  done

  echo \"[EXPLORE] waiting for forever_explore executable...\"
  while ! sudox docker exec ${CONTAINER} test -e /workspace/omnilrs/install/forever_explore/lib/forever_explore/forever_explore 2>/dev/null; do
    sleep 5
  done

  echo \"[EXPLORE] waiting for /navigate_to_pose ...\"
  while ! sudox docker exec ${CONTAINER} bash -lc \"\$ROS_PRELUDE && ros2 action list 2>/dev/null | grep -q ^/navigate_to_pose\$\" 2>/dev/null; do
    sleep 5
  done

  echo \"[EXPLORE] verifying Nav2 lifecycle...\"
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

  echo \"[EXPLORE] sleeping 10s so costmaps fully populate...\"
  sleep 10

  echo \"[EXPLORE] launching explore_lite + forever_explore ...\"
  sudox docker exec -it ${CONTAINER} bash -lc \"
    \$ROS_PRELUDE &&
    source /workspace/omnilrs/install/local_setup.bash &&
    ros2 launch forever_explore forever_explore.launch.py
  \" || true
  echo \"[EXPLORE] launch exited. Respawning in 5s — Ctrl-C to stop.\"
  sleep 5
  done
'"

tmux select-window -t "$SESSION":container
echo ""
echo "============================================================"
echo " OmniLRS cloud stack started — tmux session: $SESSION"
echo "============================================================"
echo " Windows:"
echo "   [0] container   — docker run (EC2)"
echo "   [1] isaac        — Isaac Sim (run.py)"
echo "   [2] ros-stack    — SLAM + Nav2 + RViz"
echo "   [3] cliff-guard  — cliff detection"
echo "   [4] teleop       — manual keyboard control"
echo "   [5] explore      — autonomous exploration"
echo ""
echo " Earth PC: run ALVR + vr-bridge there as normal."
echo " Latency:  run ./earth_latency.sh moon on Earth PC"
echo "           run ./ec2_latency.sh moon on EC2 (new terminal)"
echo "============================================================"

if [ -z "${NO_ATTACH:-}" ] && [ -t 0 ] && [ -t 1 ]; then
  echo "Attaching (Ctrl-b d to detach)..."
  exec tmux attach -t "$SESSION"
else
  echo "Attach manually: tmux attach -t $SESSION"
fi