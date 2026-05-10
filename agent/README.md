# Lite3 agent

Laptop-side LLM agent for the Deep Robotics Jueying Lite3. Connects to the robot over two rosbridge WebSocket connections, runs a multi-agent planning loop powered by AWS Bedrock (Claude), and exposes a typed CLI and push-to-talk voice UI.

## Architecture

```
phone (push-to-talk)
      │ audio
      ▼
laptop (you)                                 Lite3 robot (ysc@192.168.1.103)
┌──────────────────────────────────┐         ┌──────────────────────────────────────┐
│ voice_server.py (Flask :5050)    │         │ rosbridge Noetic   (port 9090)        │
│   Whisper STT → Orchestrator     │         │   /camera/color/image_raw             │
│                                  │         │   /camera/depth/image_rect_raw        │
│ cli.py (REPL)                    │         │                                      │
│   Orchestrator                   │   WS    │ rosbridge Foxy     (port 9091)        │
│     └─ PlannerAgent              │ ◄─────► │   /leg_odom2  /cmd_vel               │
│          └─ 30+ tools            │         │   /agent/*  (YOLO tracker)           │
│               ├─ perception      │         │   Nav2 goal topics / TF              │
│               ├─ motion          │         │                                      │
│               ├─ follow          │         │ realsense_ros2 (systemd service)     │
│               ├─ map / Nav2      │         │ run_tracker.py  (YOLO + TensorRT)    │
│               └─ memory          │         └──────────────────────────────────────┘
│                                  │
│ WorldTickDriver (1 Hz daemon)    │   projects YOLO detections → /leg_odom frame
│ MemoryStore (per-session)        │   objects, pose, events, active goal
└──────────────────────────────────┘

internet via phone hotspot (Bedrock needs it)
robot subnet 192.168.1.x via robot WiFi
```

Two separate ROS bridges are intentional — the camera (Noetic) and motion/nav (Foxy) stacks cannot share a single rosbridge without cross-contaminating the ROS environment.

## Pre-flight checklist

Before starting, confirm:

- Laptop is on **robot WiFi** (gives you `192.168.2.x`)
- Phone is tethered for **internet** (Bedrock needs it)
- Lite3 is powered on, standing, in walk mode, **Auto Mode ON** in the DeepRobotics app
- `agent/.env` has a valid `AWS_BEARER_TOKEN_BEDROCK` (tokens expire ~12 h — rotate if needed)
- **EPFL VPN is disconnected** — it hijacks the route to the robot

## Bringup (cold start, ~5 min)

### 1. Laptop — add route to robot subnet

```bash
sudo route -n add 192.168.1.0/24 192.168.2.1
```

Verify: `ping -c 2 192.168.1.103` → replies. If the route disappears after sleep or WiFi bounce, just re-run.

### 2. Robot terminal A — verify camera

```bash
ssh ysc@192.168.1.103   # password: '
sudo systemctl restart realsense_ros2
sleep 8
source /opt/ros/noetic/setup.bash
timeout 5 rostopic hz /camera/color/image_raw   # expect ~30 Hz
```

If the rate is 0 and `lsusb | grep -i intel` returns nothing, the USB cable is loose — reseat it. Once confirmed, you can `exit` this terminal; the service keeps running.

### 3. Robot terminal B — Noetic rosbridge (camera, port 9090)

```bash
ssh ysc@192.168.1.103
source /opt/ros/noetic/setup.bash
ROS_MASTER_URI=http://localhost:11311 roslaunch rosbridge_server rosbridge_websocket.launch
```

Wait for: `Rosbridge WebSocket server started at ws://0.0.0.0:9090`. Leave open.

### 4. Robot terminal C — Foxy rosbridge (motion + Nav2, port 9091)

Open a **fresh SSH session** — do not reuse a terminal that sourced Noetic.

```bash
ssh ysc@192.168.1.103
source /opt/ros/foxy/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9091
```

Wait for: `port 9091`. Leave open.

### 5. Laptop — verify both bridges

```bash
nc -vz 192.168.1.103 9090   # Noetic camera bridge
nc -vz 192.168.1.103 9091   # Foxy motion bridge
```

Both should say `succeeded`.

### 6. Laptop — run the agent

```bash
cd /path/to/agent
source .venv/bin/activate
python cli.py
```

Smoke-test:
```
> what do you see?
> send a basic goal to x 0.2 y 0.0 theta 0.0
> cancel the basic goal
```

Only one `cli.py` per machine is allowed. If a previous run crashed, remove the stale lock:
```bash
rm agent/.cli.pid.lock
```

### 7. (Optional) Robot terminal D — YOLO tracker

Required for `follow_person`, `go_to_object`, `find_and_go_to`, and the semantic world map. Takes ~40 s to warm up (TensorRT).

**Fresh SSH session — must NOT have Noetic sourced.**

```bash
ssh ysc@192.168.1.103
source /opt/ros/foxy/setup.bash
cd /home/ysc/lite_cog_ros2/track/src
python3 run_tracker.py
```

Wait for: `[tracker] running. window=False. publishing /agent/* topics.`

### 8. (Optional) Voice UI

```bash
python voice_server.py
```

Find your laptop's WiFi IP:
```bash
ipconfig getifaddr en0
```

On the phone, open `http://<laptop-ip>:5050/` in Safari or Chrome. Tap **enable audio** once (iOS requires a user gesture), then hold the button to talk and release to send. Round-trip time is 3–6 s (Whisper + Bedrock + TTS).

## Laptop setup (first time)

```bash
cd agent
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
cp .env.example .env
# fill in: AWS_BEARER_TOKEN_BEDROCK, BEDROCK_MODEL_ID
```

### Environment variables

| Variable | Default | Notes |
|----------|---------|-------|
| `AWS_BEARER_TOKEN_BEDROCK` | — | Required. Expires ~12 h. |
| `BEDROCK_MODEL_ID` | `us.anthropic.claude-sonnet-4-6` | Override to test another model. |
| `ROS_BRIDGE_HOST` | `192.168.1.103` | Robot IP. |
| `ROS2_BRIDGE_PORT` | `9091` | Foxy bridge port. |
| `RGB_MAX_HZ` | `2` | Client-side RGB throttle. `0` = unlimited. |
| `DEPTH_MAX_HZ` | `2` | Client-side depth throttle. |
| `ROS2_MAP_FRAME` | `map` | TF map frame name. |
| `ROS2_BASE_FRAME` | `rslidar` | Robot base frame. |
| `NAV2_GOAL_POSE_TOPIC` | — | Set to `/goal_pose` if rosbridge actions don't work. |
| `WORLD_TICK_PERIOD_S` | `1.0` | World map update interval. |
| `DISABLE_WORLD_TICK` | — | Set to `1` to disable semantic world map. |
| `WHISPER_MODEL` | `small` | `tiny`/`base`/`small`/`medium`. |
| `TTS_VOICE` | `Samantha` | Any macOS `say` voice (`say -v ?` to list). |
| `USER_NAME` | `Maria` | Robot addresses the user by name. |

## Tools

| Tool | What it does |
|------|-------------|
| `describe_scene` | RGB frame → Claude VLM → one-sentence scene description |
| `read_label` | RGB frame → Claude VLM → verbatim text on a named item |
| `get_rgbd_summary` | Depth stats: min, median, center distance, valid fraction |
| `get_depth_at_pixel` | Depth at a specific RGB pixel (u, v) with nearest-valid fallback |
| `list_visible_objects` | YOLO detections + per-object VLM descriptions, cached 8 s |
| `get_visible_objects` | Objects from in-session memory (no camera call) |
| `resolve_reference` | Resolve "it", "that one", "the chair on the left" → yolo_id |
| `find_objects_matching_constraints` | Filter memory by label, position, recency |
| `find_object_in_world` / `list_world_objects` | Objects with world coordinates from the semantic map |
| `go_to_world_object` | Navigate to a remembered object by label |
| `walk_forward` / `walk_backward` | Move by distance (m) or duration (s) |
| `turn_left` / `turn_right` | Turn by angle (deg) or duration (s) |
| `stop_motion` | Halt locomotion |
| `follow_person` | Continuously follow a YOLO-tracked person |
| `go_to_object` | Approach a YOLO object, auto-stop at stop_distance_m |
| `find_and_go_to` | Rotate scanning YOLO until label found, then walk to it |
| `find_object` | Same rotation sweep, but locate only (don't walk) |
| `find_person_and_follow` | Find a person via sweep, then follow continuously |
| `stop_tracking` | Stop follow/go-to tracking |
| `get_robot_pose_in_map` | x, y, yaw in the map frame (from TF) |
| `get_map_summary` | High-level map metadata |
| `get_local_map_context` | Nearby free/occupied space within a radius |
| `get_local_occupancy_grid` | ASCII-style occupancy grid around the robot |
| `save_waypoint` | Save current pose as a named waypoint |
| `list_waypoints` / `get_waypoint` | List or fetch saved waypoints |
| `check_waypoint_reachable` | Path-plan to waypoint, report reachability + distance |
| `get_route_summary_to_waypoint` | Human-readable route description to a waypoint |
| `compare_map_vs_live_scan` | Check whether the stored map matches the current LiDAR scan |
| `go_to_map_pose` | Navigate to absolute (x, y, yaw) via Nav2 |
| `go_to_waypoint` | Navigate to a named waypoint via Nav2 |
| `get_navigation_status` / `cancel_navigation` | Poll or cancel an active Nav2 goal |
| `stop` | Emergency stop (all motion + tracking) |
| `get_robot_status` | Connection health, frame ages, last pose |
| `speak_to_user` | TTS output to the user (only way the agent speaks) |

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| `Failed to connect to ROS` | rosbridge died | Redo step 3 or 4 |
| `Can't assign requested address` | Route gone (sleep/WiFi bounce) | Redo step 1 |
| `Connection refused` on 9090/9091 | Bridge not running | Step 3 / step 4 |
| No RGB (`rgb_age_s: null`) | RealSense color pipe stuck | See "RGB silently drops to 0 Hz" below |
| `Invalid API Key format` | Bedrock bearer token expired | Get new token, update `.env` |
| `explicit deny` on Bedrock | Model not entitled | Use `us.anthropic.claude-sonnet-4-6` |
| Dog doesn't move on `/cmd_vel` | Auto Mode off, or not in walk mode | Enable Auto Mode + stand in DeepRobotics app |
| `Tracker hangs at BLOCKING MODE` | TensorRT still loading | Wait ~40 s |
| `RLException: run_id … does not match` | Stale roslaunch holds the Noetic rosmaster | See "Killing stale rosbridge" below |
| `Another agent appears to be running` | Stale lock from a crash | `rm agent/.cli.pid.lock` |

### RGB silently drops to 0 Hz

Symptom: `realsense_ros2` shows `active (running)`, depth publishes fine at 30 Hz, but `/camera/color/image_raw` is silent. `journalctl -u realsense` shows `control_transfer returned error, Resource temporarily unavailable`.

This is a kernel UVC driver bug — the color pipe's USB state gets stuck. Restarting the ROS node alone doesn't clear it.

```bash
# On the robot:
sudo /sbin/modprobe -r uvcvideo && sudo /sbin/modprobe uvcvideo
sudo systemctl restart realsense_ros2
source /opt/ros/noetic/setup.bash
sleep 10
timeout 5 rostopic hz /camera/color/image_raw   # should show ~30 Hz
```

If still 0 Hz, power-cycle the robot. If that fails, physically reseat the camera USB cable.

### Killing a stale rosbridge

If an SSH terminal disconnected without Ctrl-C, the rosbridge process is still running on the robot and owns the port. A fresh `roslaunch` then fails with `run_id … does not match`.

```bash
# On the robot — find stale processes:
ps -fC python3 | grep -E "roslaunch|rosbridge|rosapi"
ps -fC python3 | grep ros2

# Kill Noetic bridge (port 9090):
pkill -f rosbridge_websocket
pkill -f "roslaunch rosbridge_server"

# Kill Foxy bridge (port 9091):
pkill -f "ros2 launch rosbridge_server"

# Do NOT kill the realsense roslaunch — it owns the Noetic rosmaster.
```

Then redo steps 3/4. Confirm with:
```bash
nc -vz localhost 9090
nc -vz localhost 9091
```

### Rotating the Bedrock token

Bearer tokens expire in ~12 h. When you see `Authentication failed: Please make sure your API Key is valid.`, get a fresh token and update `agent/.env`. Do not commit or paste the token in chat.
