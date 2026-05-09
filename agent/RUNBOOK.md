# Bringup runbook

Cold-start sequence. ~5 min when nothing has gone weird.

## Pre-flight

- Laptop on the **robot's WiFi** (gives you `192.168.2.x`)
- Phone tethered for **internet** (Bedrock needs it)
- The Lite3 powered on, dog standing in walk mode, **Auto Mode ON in the DeepRobotics app** (motion won't work without this)
- `agent/.env` has the `AWS_BEARER_TOKEN_BEDROCK` (rotate if expired — bearer tokens last ~12h)
- **Don't connect the EPFL VPN** — it hijacks the route to the robot

## 1. Laptop — route to the robot subnet

```
sudo route -n add 192.168.1.0/24 192.168.2.1
```

**Verify:** `ping -c 2 192.168.1.103` → replies. If "Can't assign requested address" later, the route was wiped (sleep / WiFi bounce). Just re-run.

## 2. Robot terminal A — start camera

```
ssh ysc@192.168.1.103
sudo systemctl restart realsense
sleep 8
source /opt/ros/noetic/setup.bash
timeout 5 rostopic hz /camera/color/image_raw
```

**Expect** ~30 Hz. If empty → camera failed; check `lsusb | grep -i intel` — if nothing, the USB cable is loose, ask someone to reseat it.

You can `exit` this terminal once it's working — realsense runs as a service.

## 3. Robot terminal B — noetic rosbridge (camera)

```
ssh ysc@192.168.1.103
source /opt/ros/noetic/setup.bash
roslaunch rosbridge_server rosbridge_websocket.launch
```

**Wait for** `Rosbridge WebSocket server started at ws://0.0.0.0:9090`. Leave open.

## 4. Robot terminal C — foxy rosbridge (motion + follow + basic goal)

**Open a fresh SSH session — don't reuse a terminal you sourced noetic in.**

```
ssh ysc@192.168.1.103
source /opt/ros/foxy/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9091
```

**Wait for** `port 9091`. Leave open.

## 5. Laptop — verify both bridges

```
nc -vz 192.168.1.103 9090
nc -vz 192.168.1.103 9091
```

Both should say "succeeded".

## 6. Laptop — run the agent (perception + motion)

```
cd /Users/maria/Desktop/RoboHack/agent
source .venv/bin/activate
python cli.py
```

You should see stderr lines for: camera bridge (9090), **one** ROS 2 bridge (9091) shared by motion, follow, and basic goal, then adapter status. If the tracker isn’t running yet, follow still connects — detection tools return empty until step 8.

Smoke-test:
```
> what do you see?
> send a basic goal to x 0.2 y 0.0 theta 0.0
> cancel the basic goal
```

If the camera works and the dog moves to the short goal, perception + motion are good.

## 6b. Basic goal + low-level command checks (real robot)

Make sure the area is clear and Auto Mode is on.

Basic goal:
```
> send a basic goal to x 0.3 y 0.0 theta 0.0
> what is the basic goal status?
> cancel the basic goal
```

Optional wait-for status:
```
> wait for basic goal status goal_reached (timeout 10s)
```

Low-level commands (use valid command codes for your controller):
```
> send a simple cmd with cmd_code 1 size 0 type 0
> send a complex cmd with cmd_code 2 size 1 type 0 data 0.5
```

## 7. (Optional) Robot terminal D — start the follow tracker

**Only do this if you need the follow-me feature.** The tracker takes ~40s to warm up (TensorRT). It also seems to occasionally make realsense flake — see "Known issues" below.

**Fresh SSH session — must NOT have noetic sourced first.**

```
ssh ysc@192.168.1.103
source /opt/ros/foxy/setup.bash
cd /home/ysc/lite_cog_ros2/track/src
python3 run_tracker.py
```

**Wait for** `[tracker] running. window=False. publishing /agent/* topics.` — that's the new windowless mode. Until you see that line, the tracker is still loading YOLO.

If you want the visual debugging window (XQuartz on Mac):
```
ssh -Y ysc@192.168.1.103
TRACKER_SHOW_WINDOW=1 python3 run_tracker.py
```

## 8. Test the follow flow from the agent

Stand in front of the dog (~1–3 m). Then in the agent prompt:

```
> follow me
```

Watch stderr — you should see:
```
  → tool: list_people({})
  ← {"yolo_ids": [1], "vlm_descriptions_raw": "..."}
  → tool: follow_person({"yolo_id": 1})
```

Then:
```
> stop following
```

## 9. (Optional) Voice + phone web app

A push-to-talk web app on your phone records audio, the laptop runs Whisper STT,
feeds the transcript into the same agent (Orchestrator → DialogueAgent →
PlannerAgent → tools), TTSes the reply with macOS `say`, sends it back. Same
agent the typed CLI uses (no separate code path).

**Pre-flight on the laptop:**
- Whisper is installed in the venv (`pip show openai-whisper` should print)
- `ffmpeg` is on PATH (`brew install ffmpeg`) — Whisper uses it to decode
  the phone's audio
- Phone is on the **same WiFi as the laptop**

Start the server (laptop):

```
cd /Users/maria/Desktop/RoboHack/agent
source .venv/bin/activate
python voice_server.py
```

Wait for these lines in stderr:
- `[voice] preloading whisper model=base …` then `whisper ready in N.Ns`
- `[voice] connecting to camera bridge ws://192.168.1.103:9090 …`
- `[voice] new pipeline ready [bedrock-boto3]` ← confirms the new agent path
  (if you see `falling back to legacy run_once`, paste the error)
- `[voice] serving on http://0.0.0.0:5050/`

Find the laptop's WiFi IP:

```
ipconfig getifaddr en0
```

On the phone, open in Safari/Chrome:

```
http://<laptop-wifi-ip>:5050/
```

First time only: tap **enable audio** (iOS Safari needs a user gesture before
playback). Then **hold the big button to talk** → release → wait ~3–6 s
(Whisper + Bedrock + TTS) → the reply plays automatically.

Sanity-check audio without the agent: open `http://<laptop-ip>:5050/tts_test`
on the phone — should download/play "hello, this is a test of the audio path."

Tweaks (env vars):
- `WHISPER_MODEL=tiny|base|small|medium` — default `base`. `tiny` is faster but
  less accurate; `small` is the sweet spot if the laptop has the headroom.
- `TTS_VOICE=Samantha|Daniel|Karen|Allison|...` — any macOS `say` voice
  (`say -v ?` to list)
- `VOICE_PORT=5050` — change if 5050 is taken
- `LEGACY_LOOP=1` — bypass the Orchestrator and use the old single-agent loop
  (useful if the new pipeline is broken)

## Known issues

| Symptom | Likely cause | Fix |
|---|---|---|
| Agent launch breaks other robot clients | rosbridge overloaded (burst subscriptions from multiple tools / agents) | Run only one `python cli.py`; remove stale `.cli.pid.lock` if needed; lower `RGB_MAX_HZ` / `DEPTH_MAX_HZ` in `.env` (defaults 2 Hz) |
| `Another agent appears to be running` | Second CLI or crashed exit left lock | Stop the other process or delete `agent/.cli.pid.lock` if stale (Windows: delete manually; Unix: lock clears if PID is dead) |
| Camera dies (`No RealSense devices were found`) | RealSense USB flake. Sometimes correlated with tracker running. | `sudo systemctl restart realsense`; if persistent, reseat USB cable |
| `Failed to connect to ROS` | rosbridge died (terminal closed) | redo step 3 or 4 |
| `Can't assign requested address` | route gone | redo step 1 |
| `Connection refused` on 9090/9091 | rosbridge not running | step 3 / step 4 |
| `Invalid API Key format` | Bedrock bearer token expired | get new token, update `.env` |
| `explicit deny` on Bedrock | Model not entitled | use `us.anthropic.claude-sonnet-4-6` |
| `_TYPE_SUPPORT` error launching tracker | mixed noetic + foxy in shell | open fresh SSH, source only foxy |
| Dog doesn't move on `/cmd_vel` | Auto Mode not on, or not in walk mode | use the DeepRobotics app to enable Auto Mode + stand |
| Tracker hangs at `BLOCKING MODE` | Just warming up | wait 40s for TensorRT |
| `RLException: run_id on parameter server does not match declared run_id` | A previous `roslaunch` is still alive owning the noetic rosmaster, and a new launch tried to spin up a competing one | See "Killing stale rosbridge / roslaunch" below |

## Killing stale rosbridge / roslaunch

If a previous SSH terminal disconnected without `Ctrl+C`-ing the rosbridge, the
process is still running on the robot, holding port 9090 (or 9091) and the
rosmaster `run_id`. A fresh `roslaunch rosbridge_server …` then crashes with
`RLException: run_id on parameter server does not match declared run_id`.

Find and kill it. On the robot:

```
ps -fC python3 | grep -E "roslaunch|rosbridge|rosapi" | grep -v grep
ps -fC python3 | grep ros2 | grep -v grep
```

Note the PIDs in column 2. Then:

```
# noetic side (port 9090) — kill rosbridge AND any stale roslaunch parent
pkill -f rosbridge_websocket
pkill -f "roslaunch rosbridge_server"

# foxy side (port 9091)
pkill -f "ros2 launch rosbridge_server"
```

DO NOT kill the realsense roslaunch — it's the one that owns the noetic
rosmaster you actually want to keep:

```
ps -ef | grep "roslaunch realsense2_camera" | grep -v grep   # leave this alone
```

After the kills, retry the launch from the runbook step. The rosmaster from the
realsense service stays up, your new rosbridge attaches to it cleanly with:

```
ROS_MASTER_URI=http://localhost:11311 roslaunch rosbridge_server rosbridge_websocket.launch
```

Confirm it's listening:

```
nc -vz localhost 9090   # for noetic
nc -vz localhost 9091   # for foxy
```

## RGB camera silently goes to 0 Hz (depth still works)

Symptom — the realsense systemd service shows `active (running)`, depth is publishing
fine at 30 Hz, but `/camera/color/image_raw` has zero messages and `get_status` returns
`rgb_age_s: null`. `journalctl -u realsense` shows lines like:

```
messenger-libusb.cpp: control_transfer returned error, Resource temporarily unavailable
```

What's happening — the Intel D435i has two parallel USB pipes: one for depth (lower
bandwidth), one for color (higher bandwidth). Under sustained load — or sometimes just
randomly — the color pipe's USB control transfer fails and the camera firmware leaves
that channel in a stuck state. Restarting the realsense ROS node alone doesn't fix it,
because the bug isn't in the ROS node. It's in the **kernel UVC driver** holding stale
state, plus the camera firmware refusing further control transfers on the bad pipe.

The fix that works — kick the UVC kernel driver out and reload it, *then* restart the
realsense node:

```
sudo /sbin/modprobe -r uvcvideo && sudo /sbin/modprobe uvcvideo
sudo systemctl restart realsense
source /opt/ros/noetic/setup.bash
sleep 10
timeout 5 rostopic hz /camera/color/image_raw   # should now show ~30 Hz
```

Why each step matters:
- `modprobe -r uvcvideo` unloads the kernel module that owns the USB camera. This forces
  the kernel to drop its stuck state.
- `modprobe uvcvideo` reloads it cleanly, which re-enumerates the camera and re-negotiates
  the USB pipes from scratch.
- `systemctl restart realsense` then re-launches the ROS node so it can attach to the
  freshly-clean device.
- `sleep 10` gives the realsense node time to enumerate, configure streams, and start
  publishing before we test.

If after this the rate is still 0, the camera USB is stuck deeper than the kernel driver
can fix. Power-cycle the dog. If even that fails, physically reseat the camera's USB
cable on the robot.

## Rotating the Bedrock token

Bearer tokens expire in ~12h. When you see `Authentication failed: Please make sure your API Key is valid.`, get a fresh one and update [agent/.env](.env). Don't paste the token in chat / commits.
