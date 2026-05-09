# Bringup runbook

Step-by-step to get the agent talking to the Lite3 from a cold start.
Follow in order — each step has a one-line "did it work?" check.

## What you need next to you

- Laptop on the **robot's WiFi** (gives you `192.168.2.x`)
- Phone tethered for **internet** (so Bedrock works)
- The Lite3 powered on
- `agent/.env` already has the Bedrock token + model from yesterday

---

## 1. Laptop: route the robot subnet through the WiFi gateway

The robot's perception host is on `192.168.1.103`, but your laptop is on `192.168.2.x`. Add a route:

```
sudo route -n add 192.168.1.0/24 192.168.2.1
```

**Check:** `ping -c 2 192.168.1.103` → should get replies.

If you see "Can't assign requested address" later, the route was wiped (sleep / VPN / WiFi bounce). Just re-run the command.

**Do NOT connect the EPFL VPN** — it hijacks the route and breaks robot access. Bedrock is on the public internet, you don't need the VPN.

---

## 2. Robot: SSH in

```
ssh ysc@192.168.1.103
```

Password is `'` (a single quote).

**Check:** you see `ysc@lite:~$`.

---

## 3. Robot: confirm the camera is publishing

```
source /opt/ros/noetic/setup.bash
rostopic list | grep image_raw
```

**Check:** you see `/camera/color/image_raw` and `/camera/depth/image_rect_raw`.

If empty, the realsense service isn't running. Start it:

```
sudo systemctl restart realsense
sleep 5
rostopic list | grep image_raw
```

---

## 4. Robot: launch rosbridge

In the same SSH terminal (must already be sourced from step 3):

```
roslaunch rosbridge_server rosbridge_websocket.launch
```

**Check:** the last log line says `Rosbridge WebSocket server started at ws://0.0.0.0:9090`.

**Leave this terminal open.** If you close it, rosbridge dies and the laptop can't talk to the robot.

---

## 5. Laptop: confirm the bridge is reachable

In a *new* laptop terminal:

```
nc -vz 192.168.1.103 9090
```

**Check:** "succeeded".

If "Connection refused" → step 4 didn't actually start (look at the SSH terminal for errors).
If "Can't assign requested address" → the route from step 1 is gone; re-add it.

---

## 6. Laptop: smoke-test camera (optional but recommended)

```
cd /Users/maria/Desktop/RoboHack/agent
source .venv/bin/activate
python scripts/grab_frame.py
```

**Check:** prints `saved frame_rgb.jpg` and a depth summary, no traceback. Open `frame_rgb.jpg` to confirm it's a real photo from the robot.

---

## 7. Laptop: smoke-test Bedrock (optional)

```
python3 -c "
import os
from dotenv import load_dotenv
load_dotenv()
import boto3
c = boto3.client('bedrock-runtime', region_name=os.environ['AWS_REGION'])
r = c.converse(
    modelId=os.environ['BEDROCK_MODEL_ID'],
    messages=[{'role':'user','content':[{'text':'say hello'}]}],
    inferenceConfig={'maxTokens':20},
)
print(r['output']['message']['content'][0]['text'])
"
```

**Check:** prints a hello.

If "Invalid API Key format" → the Bedrock bearer token in `.env` is missing or malformed.
If "AccessDeniedException ... explicit deny" → the model in `BEDROCK_MODEL_ID` isn't entitled to your role. Currently working: `us.anthropic.claude-sonnet-4-6`.
If DNS error → phone tether is off, no internet.

---

## 8. Laptop: run the agent

```
python cli.py
```

**Check:** `connecting to rosbridge ws://192.168.1.103:9090 …` then `connected. type a question`.

Try a real perception question (must mention seeing/looking, otherwise the model just chats):

```
> what do you see in front of you?
```

You should see in stderr:
```
  → tool: describe_scene({})
  ← <Claude's description of the actual frame>
```

Then the answer prints to stdout.

---

## Quick reference: what's in the project

- [cli.py](cli.py) — agent loop
- [robot/lite3.py](robot/lite3.py) — rosbridge subscriber for camera, depth, pose
- [tools/perception.py](tools/perception.py) — tools the LLM can call
- [vlm.py](vlm.py) — Bedrock client wrapper
- [scripts/grab_frame.py](scripts/grab_frame.py) — direct frame test, no agent

## Tomorrow's open work

- **`/leg_odom` and `/cmd_vel`** are on ROS 2 (foxy), not ROS 1 (noetic). Current rosbridge is noetic, so `get_pose` returns null and we can't drive yet.
- Fix: install `ros-foxy-rosbridge-server` on the robot (needs the robot to have internet — quickest path is putting the robot on your phone's hotspot for 2 minutes), launch it on a different port (9091), add a second `Lite3Robot` connection in the code for the foxy graph.
- After that: add `walk_forward` / `turn` / `stop` tools that publish to `/cmd_vel`.

## When things go wrong mid-session

| Symptom | Likely cause | Fix |
|---|---|---|
| `Failed to connect to ROS` | rosbridge died | SSH in, redo step 4 |
| `Can't assign requested address` | route gone | redo step 1 |
| `Connection refused` on 9090 | rosbridge not running | step 4 |
| `nodename ... not known` | no internet | check phone tether |
| `Invalid API Key format` | bad bearer token | check `.env` |
| `explicit deny` | model not entitled | use `us.anthropic.claude-sonnet-4-6` |
| Agent answers but never calls a tool | question too generic | mention "see"/"look"/"in front" so the model invokes `describe_scene` |
