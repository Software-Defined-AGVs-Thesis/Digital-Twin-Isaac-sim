# OmniLRS Lunar Rover — Project Wiki

Software-Defined Autonomous Guided Vehicle (AGV) with Virtual Telepresence.
This wiki is the handoff doc for whoever picks up this project next.

---

## 1. System Overview

The project has three main pieces:

1. **Onboard autonomy** — ROS 2 Humble, Nav2, SLAM Toolbox, SMAC Hybrid-A*,
   MPPI. Runs identically in sim (Isaac Sim) and, partially, on the real robot.
2. **SASL — Safety Assurance and Safeguarding Layer** — a set of independent
   ROS 2 nodes that watch the robot and can halt autonomy if something looks
   dangerous, handing control to a human via VR.
3. **Digital twin / telepresence** — Isaac Sim + OmniLRS (lunar terrain sim)
   in Docker, streamed to a Meta Quest 2 via ALVR, with a UDP bridge carrying
   VR controller input back into ROS 2.

Hardware: HiWonder JetAuto rover (mecanum drive), Jetson Nano onboard compute.

```
                 ┌────────────────────┐
                 │   Isaac Sim + OmniLRS│  (Docker container)
                 │   lunar terrain sim   │
                 └─────────┬────────────┘
                           │ /clock, sensors
                 ┌─────────▼────────────┐
                 │   ROS 2 Humble stack  │
                 │  SLAM Toolbox → Nav2  │
                 └───┬───────────────┬──┘
                     │               │
          ┌──────────▼───┐   ┌───────▼────────┐
          │  SASL         │   │  Teleop         │
          │  cliff_guard  │   │  keyboard OR    │
          │  (stuck_guard │   │  VR joystick    │
          │   - WIP)      │   │  (ALVR + UDP    │
          └───────────────┘   │   bridge)       │
                               └─────────────────┘
```

---

## 2. One-Time Setup

Do this once per machine.

```bash
# 1. Install tmux on the host
sudo apt install -y tmux

# 2. Build the Docker image (only once, or after Dockerfile changes)
cd ~/Desktop/OmniLRS
sudo ./omnilrs.docker/build_docker.sh

# 3. Pull the explore_lite submodule (once, after a fresh clone)
git submodule update --init --recursive
```

Everything else — in-container apt installs, rosdep, `colcon build` of the
ROS workspace, and a source patch to `explore_lite` for slam_toolbox QoS
compatibility — is handled automatically by the launcher on first run.
First run takes ~5 minutes (mostly apt-install); later runs are fast.

**VR prerequisites** (installed separately, not by the launcher):
- ALVR v20.14.0 streamer (`~/alvr_streamer_linux` or wherever `ALVR_DIR` points)
- `UDP_Bridge` folder containing `vr_host_sender.py`
- `sinpin-vr` for optional PC screen mirroring inside the headset

---

## 3. Daily Run (the normal way)

```bash
cd ~/Desktop/OmniLRS
./scripts/start_omnilrs.sh
```

You'll be asked for your sudo password once (hidden input); every internal
`sudo` call reuses it via an askpass helper — no repeat prompts. The script
auto-attaches you to a tmux dashboard called `omnilrs`.

```bash
# Detach (everything keeps running)
Ctrl-b d

# Re-attach later
tmux attach -t omnilrs

# Start without auto-attaching
NO_ATTACH=1 ./scripts/start_omnilrs.sh

# Stop everything
./scripts/stop_omnilrs.sh
```

Password options:
| Method | Command |
|---|---|
| Interactive prompt (recommended) | `./scripts/start_omnilrs.sh` |
| Env var (avoids shell history) | `SUDO_PASSWORD=xxx ./scripts/start_omnilrs.sh` |

### What each tmux window does

| # | Name | Purpose |
|---|------|---------|
| 0 | `alvr` | ALVR dashboard (host) — connects the Quest headset |
| 1 | `vr-bridge` | `vr_host_sender.py`, bridges SteamVR → UDP. Auto-respawns every 5s until SteamVR is reachable |
| 2 | `container` | Boots the Isaac Sim Docker container |
| 3 | `isaac` | Waits for the container, runs `run.py` (Isaac Sim) |
| 4 | `ros-stack` | Waits for prerequisites, auto-builds the workspace if needed, launches SLAM → Nav2 → RViz → VR receiver (staggered 10s apart). Wrapped in a respawn loop |
| 5 | `cliff-guard` | Runs `cliff_guard.launch.py` standalone — isolated logs, isolated crashes. Respawn loop |
| 6 | `stuck-guard` | Runs `stuck_guard.launch.py` standalone  |
| 7 | `teleop` | Interactive `teleop_twist_keyboard` — drive manually with `i j k l u o m , .`, speed with `q`/`z` |
| 8 | `explore` | Autonomous frontier exploration — `explore_lite` + `forever_explore` (see §6) |

Windows 4/5/7(explore) survive container restarts — they log
`respawn attempt #N`, wait for the new container, and relaunch automatically.

**Tmux basics:** prefix is `Ctrl-b`.
`Ctrl-b 0..8` jump to a window · `Ctrl-b n`/`p` next/previous · `Ctrl-b w`
window picker · `Ctrl-b [` scroll mode (`q` to exit) · `Ctrl-b d` detach.

---

## 4. Verifying the Stack Is Up

```bash
sudo docker exec -it isaac-sim-omnilrs-container bash -lc \
  "source /opt/ros/humble/setup.bash && ros2 topic list"
```

A healthy run shows:
```
/clock
/cmd_vel, /cmd_vel_nav, /vr/cmd_vel
/scan, /map, /tf, /odom
/cliff_guard/depth, /cliff_guard/alert, /cliff_guard/status
/explore/status, /explore/frontiers
/navigate_to_pose/_action/feedback
```

Check Nav2's action server is actually registered:

```bash
sudo docker exec -it isaac-sim-omnilrs-container bash -lc \
  "source /opt/ros/humble/setup.bash && ros2 action info /navigate_to_pose -t"
```

Look for `Action servers: 1`. `0` means Nav2's lifecycle nodes are stuck —
window 8's self-heal handles this at launch (see §6); mid-run, fix manually
(see §8).

---

## 5. SASL — Safety Assurance and Safeguarding Layer

SASL is not one node — it's a small set of independent, isolated watchdogs
plus a human handover mechanism.

### cliff_guard 
Monitors depth data for cliff/drop-off hazards. On detection it does three
things, in order:
1. Spams zero `Twist` on **both** `/cmd_vel` and `/cmd_vel_nav` at 20 Hz, so
   `velocity_smoother` has nothing left to forward.
2. Cancels all active goals on `/navigate_to_pose` via the cancel-goal action.
3. Best-effort lifecycle-deactivates the Nav2 nodes (may partially fail —
   steps 1+2 already stop the wheels, so this is belt-and-suspenders).

> Earlier versions only tried step 3 (lifecycle deactivate). Nav2's
> `bt_navigator`/`behavior_server` frequently **reject** a deactivate while a
> goal is in flight, so the robot kept half-moving (BT still ticking,
> recoveries writing to `/cmd_vel_nav`). Steps 1+2 were added to fix this.

**If the robot still moves after a "CLIFF DETECTED" log line:** check
`vr_override_active` — while a human has VR control, cliff_guard
intentionally does *not* zero `/cmd_vel` (the operator keeps authority).
Otherwise look for a "Cancel-all dispatched" log line; if it's missing, the
`/navigate_to_pose` action server isn't registered (Nav2 lifecycle issue,
see §8).

### stuck_guard 
Detects "wheels spinning but pose static." The launch file and node exist
and run standalone in tmux window 6.

### VR override / human handover
This is the actual safety-critical handoff path:

1. SASL detects a critical situation → autonomous driving (Nav2) is halted.
2. A UI alert appears telling the operator there's a critical situation,
   with a **VR override** button.
3. Operator clicks it → they now drive the robot directly via the VR
   joystick.
4. Once clear of the hazard, the operator clicks a **return control** button
   in the UI → authority goes back to Nav2.

`vr_override` and `handover_popup` are the two executables behind this (both
live under `cliff_guard`'s package and get symlink-healed by the launcher).

---

## 6. Autonomous Exploration (core demo feature)

This is the centerpiece of the space-exploration autonomy story: the robot
maps and explores without a human driving.

- **`explore_lite`** — frontier-based exploration; picks unexplored map
  frontiers as goals.
- **`forever_explore`** — companion node. When the room is fully mapped,
  it clears the global costmap and dispatches a random reachable goal so
  the robot keeps roaming indefinitely instead of stopping.

Watch tmux window 8 for banners: `FOREVER MODE - cycle #N`, `RANDOM GOAL`,
`Nav2 action server registered`.

Startup sequence in that window includes a **Nav2 lifecycle self-heal**:
if `bt_navigator` is stuck `unconfigured` (common when
`lifecycle_manager_navigation` hits a configure timeout under CPU load at
boot), the window manually walks each Nav2 node through
`configure → activate`, up to 6 attempts, before giving up and warning.

**Known quirk:** `explore_lite`'s costmap subscription defaults to
`VOLATILE` QoS, but `slam_toolbox` publishes `/map` with `TRANSIENT_LOCAL`
durability — a mismatch that silently drops every map message and blocks
exploration forever. `start_omnilrs.sh` patches this in `explore_lite`'s
source automatically (idempotent, runs before tmux even starts).

---

## 7. Physical Robot (JetAuto hardware)

- The real rover is **powered on manually** — no software bring-up for
  power-on itself.
- Once powered, it runs **Nav2 and SASL for real** (same core autonomy +
  cliff_guard logic as sim).
- **The sim → real digital twin mapping has some bugs, requires future work.** Earlier
  documentation implied the telepresence/digital-twin pipeline was fully
  bridged between simulation and hardware. Treat this as
  **future work**, don't build workflows in this
  wiki that assume live sim↔hardware mirroring exists yet.

---

## 8. VR teleop, manual (no tmux)

```bash
# 1. ALVR dashboard
cd ~/Downloads/alvr_v20.14.0/alvr_streamer_linux
./bin/alvr_dashboard

# 2. UDP bridge
cd ~/UDP_Bridge
python3 vr_host_sender.py

# 3. In-container VR receiver
source /opt/ros/humble/setup.bash
python3 /workspace/omnilrs/vr_container_receiver.py

# 4. Verify
ros2 topic echo /cmd_vel
```

### Screen mirroring (PC desktop inside the headset, optional)

```bash
# Connect Quest 2 via USB
adb kill-server && adb start-server && adb devices
# If "offline": put headset on, accept "Allow USB debugging", re-run adb devices

# Port forwarding
adb reverse --remove-all
adb reverse tcp:9944 tcp:9944 && adb reverse tcp:9943 tcp:9943
# If "Address already in use": adb shell am force-stop alvr.client, then retry

# ALVR dashboard
DISPLAY=:1 /home/g04-f25/Downloads/alvr_v20.14.0/alvr_streamer_linux/bin/alvr_dashboard
# Wait for "SteamVR: Connected" (green, bottom left)

# On the headset: open ALVR app → Stream
# Confirm on PC dashboard: Wired Connection → Quest 2 → Streaming ✅

# Screen mirror (new terminal)
cd ~/Downloads/sinpin-vr
./sinpin_vr
```

---

## 9. Cloud / VPC Variant 

`scripts/start_omnilrs_cloud.sh` runs the stack on AWS VPC instead of a
local machine, for remote-operator scenarios.

---

## 10. Tunable Environment Variables

Pass these before `./scripts/start_omnilrs.sh`:

| Var | Default | Meaning |
|---|---|---|
| `SUDO_PASSWORD` | (prompt) | skip the password prompt |
| `ALVR_DIR` | `$HOME/alvr_streamer_linux` | ALVR install location |
| `UDP_BRIDGE_DIR` | `$HOME/UDP_Bridge` | where `vr_host_sender.py` lives |
| `CONTAINER` | `isaac-sim-omnilrs-container` | Docker container name |
| `CLOCK_WAIT_SECS` | `15` | settle delay before first `/clock` poll |
| `CLOCK_POLL_INTERVAL` | `5` | seconds between `/clock` polls |
| `CLOCK_HEARTBEAT_EVERY` | `12` | heartbeat log every N polls |

Example: `ALVR_DIR=/opt/alvr ./scripts/start_omnilrs.sh`

---

## 11. Troubleshooting

**`vr-bridge` keeps printing "attempt #N"**
Normal until SteamVR is running and the headset is streaming — it retries
every 5s. Start SteamVR/ALVR and the next attempt succeeds.

**Window 4 stuck on "still waiting for /clock..."**
Isaac Sim hasn't finished loading — check window 3. If it crashed, fix and
rerun `run.py`; window 4 detects `/clock` automatically once it's back.

**Window 8 (explore) idle, no goals sent**
- Only seeing `EXPLORATION_STARTED` with no progress → Nav2 may not be
  ready yet (it waits 10s after Nav2 comes up).
- Seeing `EXPLORATION_COMPLETE` but no random goal follows → `/map`
  probably has no free cells yet; drive a bit with teleop to seed SLAM.
- `"0 action servers"` after 6 self-heal attempts → Nav2's lifecycle
  manager is wedged. Manual last resort (inside container):
  ```bash
  for n in controller_server smoother_server planner_server behavior_server \
           velocity_smoother bt_navigator waypoint_follower; do
    ros2 lifecycle set /$n configure
    ros2 lifecycle set /$n activate
  done
  ```

**Cliff detected but the robot keeps moving**
See §5 (`cliff_guard`) — check `vr_override_active` first, then look for
the "Cancel-all dispatched" log line.

**`ros-stack` window died after a container restart**
Shouldn't happen — windows 4/5/8 are wrapped in respawn loops. If you only
see 7 windows instead of 8, the bash inside exited before the loop could
start; inspect that window's last log lines (`Ctrl-b w` to pick it).

**"package 'X' not found" on launch**
A workspace package wasn't built. Next `start_omnilrs.sh` run should
auto-build it; to force it manually:
```bash
sudo docker exec -it isaac-sim-omnilrs-container bash -lc \
  "source /opt/ros/humble/setup.bash && cd /workspace/omnilrs && colcon build"
```

**"the input device is not a TTY"**
Old script version — update to the current one (uses a `SUDO_ASKPASS`
helper that doesn't touch stdin).

**Container won't stop**
```bash
sudo docker rm -f isaac-sim-omnilrs-container
tmux kill-server
```

---

## 12. Known Gaps / Future Work

- **Sim → real digital twin mapping** does not fully work — the real
  robot runs Nav2 + SASL independently, not mirrored from the sim (§7).
- **Cloud/VPC variant** is experimental (§9).
- **Integration with InnexisVSI** a Siemens EDA tool requirement for the project. 
