## Inspiration

Guide dogs take years to train and reach only a fraction of people who need them. We wanted to ask: what if you could give a robot dog the ability to understand natural speech, see the world, and navigate it — making mobility assistance more accessible? The Jueying Lite3 was sitting in the lab, and Bedrock was one API call away. That was enough to start.

## What it does

Lite3 is a voice-controlled AI agent running on a quadruped robot. You talk to it through a push-to-talk phone app — no commands, no app UI, just conversation.

- Say **"what's in front of me?"** → it narrates the scene using its RGB-D camera and Claude's vision
- Say **"follow me"** → it locks onto you with YOLO and trails you through the room
- Say **"find the bottle and bring me to it"** → it rotates in place scanning its live YOLO feed until it spots one, then walks over and stops at a safe distance
- Say **"go back to the desk"** → it navigates there from a persistent semantic map, even after walking away minutes ago

The robot speaks back in natural language, not robot-speak. "Found the door, about two meters ahead — walking over now." Not "Goal status: reached. Object ID: 42." The persona and voice output were designed from the start for visually impaired users who only hear the robot, never see a screen.

## How we built it

**Hardware:** Deep Robotics Jueying Lite3 quadruped, Intel RealSense D435i (RGB-D), laptop as compute host.

**AI stack:** AWS Bedrock with Claude Sonnet (claude-sonnet-4-6) is the reasoning engine and vision model — every decision the robot makes goes through Claude via the Bedrock Converse streaming API. YOLO (TensorRT, on the robot) handles real-time object and person detection. OpenAI Whisper transcribes voice on the laptop; macOS `say` handles TTS back to the phone.

**ROS:** The robot runs two separate stacks — ROS Noetic for the camera, ROS 2 Foxy for motion and Nav2 navigation. We expose both over independent rosbridge WebSocket connections (ports 9090 and 9091), so the laptop agent can subscribe and publish without a ROS install.

**Agent architecture:** An Orchestrator feeds memory snapshots to a PlannerAgent, which calls tools in a streaming loop via Bedrock Converse. The planner has 30+ tools across perception, motion, object following, map-based navigation, and memory. Read-only tools run in parallel via a thread pool.

**Semantic world map:** A background daemon runs at 1 Hz. It takes every YOLO detection, back-projects its depth into 3D using calibrated RealSense intrinsics, and transforms it into the odometry frame using the robot's current pose. The result is a persistent object map — when you say "go to the bottle", the planner checks this map first before searching.

**Voice UI:** A Flask server on the laptop hosts a push-to-talk page. The phone records audio, Whisper transcribes it, the full agent pipeline runs, and the TTS audio streams back to the phone. End-to-end round-trip: 3–6 s.

## Challenges we ran into

**Dual ROS version coexistence.** The camera runs under Noetic, motion under Foxy. Sourcing both in the same shell silently corrupts the environment. We ended up with strict SSH session discipline and two fully separate rosbridge sockets.

**RealSense USB pipe failures.** Under sustained load, the D435i's color stream would drop silently to 0 Hz while depth kept working. Restarting the ROS node didn't fix it — the bug was in the kernel's UVC driver. The fix was `modprobe -r uvcvideo && modprobe uvcvideo` followed by a service restart.

**WiFi bandwidth.** Raw RGB at 30 Hz is ~35 MB/s. We built a subscribe-one-unsubscribe-one fetch pattern with client-side rate limiting so the rosbridge doesn't saturate when multiple tools hit the camera at once.

**Building a world map without SLAM.** Getting the full Nav2 TF tree and costmaps working reliably across two bridges in hackathon time wasn't realistic. We built our own laptop-side projection pipeline: depth → 3D point → rotate by camera pitch and robot yaw → translate by odometry pose. It drifts over long runs, but it's accurate enough for room-scale tasks.

## Accomplishments that we're proud of

The `find_and_go_to` sweep feels fluid: the dog rotates continuously while a background thread polls YOLO at ~6 Hz, stops the rotation the instant it spots the target, and starts walking — no jittery stop-check-step pattern.

The voice persona. One line in the system prompt — *"the user is visually impaired and only hears you via `speak_to_user`"* — completely changed the output quality. The robot stopped reading out sensor data and started talking like a guide dog companion.

The semantic world map working over plain odometry with no SLAM. Saying *"go back to the chair"* 90 seconds and 5 meters later, and watching it navigate there correctly, was the most satisfying moment of the hackathon.

## What we learned

How to bridge ROS 1 and ROS 2 without a full migration. That RealSense depth + YOLO + odometry is enough for a usable semantic map even without SLAM. How the Bedrock Converse streaming API works for multi-step tool-use loops. And that good voice output is 90% system prompt, 10% model.

## What's next for Lite3

- Integrate the onboard LiDAR for SLAM-corrected mapping and real obstacle avoidance (we have the driver, just didn't get to it)
- Persist waypoints across sessions to a local file store
- Run the YOLO tracker continuously in the background for a tighter world map update rate
- Replace the depth advisory in `walk_forward` with actual costmap-based collision avoidance from Nav2
