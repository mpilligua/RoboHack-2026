# Lite3 perception agent (Test 1)

Laptop-side LLM agent that talks to a Qwen multimodal endpoint and pulls live RGB / depth / pose from the Jueying Lite3 over a ROS websocket bridge.

## Architecture

```
laptop (you)                                Lite3 perception host (ysc@192.168.1.103)
┌────────────────────────────────┐          ┌──────────────────────────────────┐
│ agent.py (REPL)                │          │ rosbridge_websocket   (port 9090) │
│   ↓ OpenAI SDK                 │          │ realsense node        (/camera/*) │
│ Qwen-VL endpoint  ◄── HTTPS ── │          │ message_transformer   (/leg_odom, │
│   ↑ tool calls                 │          │                       /imu/data,  │
│ tools/perception.py            │          │                       /cmd_vel)   │
│   ↑ roslibpy                   │   WS     │                                   │
│ robot/lite3.py  ◄────────────► │ ◄──────► │                                   │
└────────────────────────────────┘          └──────────────────────────────────┘
   internet (phone hotspot)                    robot WiFi (192.168.1.x)
```

## One-time setup on the robot

SSH into the perception host (password is a single quote `'`):

```bash
ssh ysc@192.168.1.103
```

Confirm RealSense and message_transformer are running:

```bash
sudo systemctl status realsense
sudo systemctl status message_transformer
# start whichever is inactive
```

Install rosbridge if needed, then launch it:

```bash
sudo apt install ros-$ROS_DISTRO-rosbridge-server   # one-time
roslaunch rosbridge_server rosbridge_websocket.launch
```

Leave that terminal running. Confirm with `rostopic list` from another shell that you see `/camera/color/image_raw`, `/camera/aligned_depth_to_color/image_raw`, `/leg_odom`, `/cmd_vel`.

## Laptop setup

```bash
cd /Users/maria/Desktop/RoboHack/agent
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
cp .env.example .env
# edit .env: QWEN_BASE_URL, QWEN_API_KEY, QWEN_MODEL
```

## Run

```bash
python -m agent.agent
```

You'll get a `>` prompt. Try:

- `what do you see?`
- `is there anything in front of the robot? how far?`
- `read the label on the can`
- `where are you right now?`

Tool calls and results are echoed to stderr so you can see the agent reasoning.

## Tools the agent has

| Tool                | What it does                                                          |
|---------------------|-----------------------------------------------------------------------|
| `describe_scene`    | Captures one RGB frame, sends to Qwen-VL with a focus hint            |
| `read_label`        | Captures one RGB frame, asks Qwen-VL to read text on a named item     |
| `get_rgbd_summary`  | Local depth stats: closest, center distance, valid fraction           |
| `get_pose`          | x, y, yaw from `/leg_odom`                                            |
| `get_status`        | Connection + frame ages — diagnostics                                 |

Locomotion (`/cmd_vel`) is wired via `Lite3Robot.set_velocity()` / `.stop()` but not exposed as a tool yet — that's Person A's slot in the plan.

## Troubleshooting

- **Connect fails**: `nc -vz 192.168.1.103 9090` from the laptop. If that fails, rosbridge isn't running on the robot.
- **No RGB**: from the robot, `rostopic hz /camera/color/image_raw`. If 0, RealSense isn't running — `sudo systemctl restart realsense`.
- **VLM 401**: re-check `QWEN_API_KEY` and `QWEN_BASE_URL` in `.env`.
- **VLM model rejects images**: confirm `QWEN_MODEL` is the multimodal variant (e.g. `qwen2.5-vl-…`), not the text-only one.
