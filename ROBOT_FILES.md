# Robot-side files we've modified

These live on the perception host (`ysc@192.168.1.103`). The local copy lives
under [src/](src/) and is the source of truth — we edit there and `scp` to the
robot. **Never edit on the robot directly except for emergency one-liners.**

When you reset / reboot the robot, sync these files first.

| Local path | Robot path | What we changed |
|---|---|---|
| [src/run_tracker.py](src/run_tracker.py) | `/home/ysc/lite_cog_ros2/track/src/run_tracker.py` | Replaced the OpenCV/keyboard loop with a windowless ROS bridge. Publishes `/agent/yolo_detections` (JSON with id, cls, label, bbox, conf), subscribes to `/agent/follow_target` accepting `<id>` (open-ended follow), `goto:<id>` (auto-stop when close), or `stop`. Optional `TRACKER_SHOW_WINDOW=1` env to bring the OpenCV window back. |
| [src/RobotController/RobotController.py](src/RobotController/RobotController.py) | `/home/ysc/lite_cog_ros2/track/src/RobotController/RobotController.py` | Added `self.last_results` (so the tracker bridge can read the latest YOLO output). Added `goto_mode` flag + `SetGotoMode` / `GetGotoMode`; when set and the target's bbox covers ≥ 20% of the frame area, the controller publishes a zero-velocity Twist and clears its tracking state. Also stubbed `cv2.waitKey` to `key = -1` so the controller runs without an OpenCV window. |
| [src/RobotController/YoloWrapper/YoloWrapper.py](src/RobotController/YoloWrapper/YoloWrapper.py) | `/home/ysc/lite_cog_ros2/track/src/RobotController/YoloWrapper/YoloWrapper.py` | Removed `classes=[CocoTypeId.kPerson]` filter so YOLO returns all 80 COCO classes (chair, door, bottle, cup, etc.), not just persons. The `CocoTypeId` import is also gone. |
| [src/restart_camera.sh](src/restart_camera.sh) | `~/restart_camera.sh` (in ysc home) | One-shot RealSense recovery script. When the camera dies (`/camera/color/image_raw` 0 Hz, libusb errors), `sudo bash ~/restart_camera.sh` reloads the UVC driver, restarts the realsense service, and verifies the topic rate. |

## Sync command (run from `/Users/maria/Desktop/RoboHack/` on the laptop)

```
scp src/run_tracker.py                         ysc@192.168.1.103:/home/ysc/lite_cog_ros2/track/src/run_tracker.py
scp src/RobotController/RobotController.py     ysc@192.168.1.103:/home/ysc/lite_cog_ros2/track/src/RobotController/RobotController.py
scp src/RobotController/YoloWrapper/YoloWrapper.py  ysc@192.168.1.103:/home/ysc/lite_cog_ros2/track/src/RobotController/YoloWrapper/YoloWrapper.py
```

(Password is `'` — a single quote.)

After scp, restart the tracker:

```
ssh ysc@192.168.1.103
source /opt/ros/foxy/setup.bash
cd /home/ysc/lite_cog_ros2/track/src
python3 run_tracker.py
```

Wait ~40 s for the YOLO/TensorRT warmup, then look for `[tracker] running. window=False. publishing /agent/yolo_detections.`.

## Backups

Each file has a `.bak` next to it on the robot from when we first edited:

```
/home/ysc/lite_cog_ros2/track/src/run_tracker.py.bak
```

If something breaks badly, you can recover the originals from there.
