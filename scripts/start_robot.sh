#!/usr/bin/env bash
# Start the 5 services that must run ON THE ROBOT for the demo:
#   0. RealSense camera (foxy)  -- starts first, others wait CAMERA_WARMUP_S
#   1. Online SLAM + nav (foxy headless)
#   2. ROS Noetic rosbridge (port 9090)
#   3. ROS Foxy rosbridge (port 9091)
#   4. run_tracker.py
#
# Uses tmux so it works headlessly over plain ssh. After running:
#   tmux attach -t robohack
# to see all panes. Detach with Ctrl-b d. Kill the whole session with:
#   tmux kill-session -t robohack

set -e

SESSION="${SESSION:-robohack}"
TRACKER_DIR="${TRACKER_DIR:-/home/ysc/lite_cog_ros2/track/src}"
ROS_NOETIC_SETUP="${ROS_NOETIC_SETUP:-/opt/ros/noetic/setup.bash}"
ROS_FOXY_SETUP="${ROS_FOXY_SETUP:-/opt/ros/foxy/setup.bash}"
REALSENSE_SETUP="${REALSENSE_SETUP:-/home/ysc/lite_cog_ros2/driver/realsense_ws/install/setup.bash}"
SLAM_SCRIPT="${SLAM_SCRIPT:-/home/ysc/lite_cog_ros2/system/scripts/online_slam/start_online_slam_nav_coarse_headless.sh}"
CAMERA_WARMUP_S="${CAMERA_WARMUP_S:-20}"

if ! command -v tmux >/dev/null 2>&1; then
    echo "tmux not installed. Run: sudo apt install tmux" >&2
    exit 1
fi

# Refuse to clobber an existing session — make the operator decide.
if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "tmux session '$SESSION' already exists." >&2
    echo "  attach:  tmux attach -t $SESSION" >&2
    echo "  kill:    tmux kill-session -t $SESSION" >&2
    exit 1
fi

# Build each command as a single string. tmux send-keys runs it in the pane's
# shell. We don't `exec bash` at the end — if a service exits, the pane shows
# the error and stays open as part of the tmux session.
CAMERA_CMD="source ${ROS_FOXY_SETUP} && source ${REALSENSE_SETUP} && \
ros2 launch realsense2_camera dr_camera_launch.py \
  enable_color:=true \
  enable_depth:=true \
  enable_infra1:=false \
  enable_infra2:=false \
  align_depth:=false \
  enable_sync:=false \
  enable_pointcloud:=false \
  initial_reset:=false \
  color_width:=640 \
  color_height:=480 \
  color_fps:=15.0 \
  depth_width:=640 \
  depth_height:=480 \
  depth_fps:=15.0"
SLAM_CMD="source ${ROS_FOXY_SETUP} && bash ${SLAM_SCRIPT}"
NOETIC_CMD="source ${ROS_NOETIC_SETUP} && roslaunch rosbridge_server rosbridge_websocket.launch"
FOXY_CMD="source ${ROS_FOXY_SETUP} && ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9091"
TRACKER_CMD="source ${ROS_FOXY_SETUP} && cd ${TRACKER_DIR} && python3 run_tracker.py"

# 5 panes stacked vertically:
#   pane 0: realsense camera (starts first, others wait CAMERA_WARMUP_S)
#   pane 1: online slam + nav
#   pane 2: rosbridge noetic
#   pane 3: rosbridge foxy
#   pane 4: run_tracker
#
# In panes 1-4 we lead with `sleep ${CAMERA_WARMUP_S}` so the camera node has
# time to initialize before anyone subscribes to its topics.
tmux new-session -d -s "$SESSION" -n services
tmux send-keys -t "$SESSION:services.0" "$CAMERA_CMD" C-m

tmux split-window -v -t "$SESSION:services"
tmux send-keys -t "$SESSION:services.1" "echo '[slam] waiting ${CAMERA_WARMUP_S}s for camera...' && sleep ${CAMERA_WARMUP_S} && ${SLAM_CMD}" C-m

tmux split-window -v -t "$SESSION:services"
tmux send-keys -t "$SESSION:services.2" "echo '[noetic] waiting ${CAMERA_WARMUP_S}s for camera...' && sleep ${CAMERA_WARMUP_S} && ${NOETIC_CMD}" C-m

tmux split-window -v -t "$SESSION:services"
tmux send-keys -t "$SESSION:services.3" "echo '[foxy] waiting ${CAMERA_WARMUP_S}s for camera...' && sleep ${CAMERA_WARMUP_S} && ${FOXY_CMD}" C-m

tmux split-window -v -t "$SESSION:services"
tmux send-keys -t "$SESSION:services.4" "echo '[tracker] waiting ${CAMERA_WARMUP_S}s for camera...' && sleep ${CAMERA_WARMUP_S} && ${TRACKER_CMD}" C-m

# Even spacing.
tmux select-layout -t "$SESSION:services" even-vertical

echo
echo "[robot] tmux session '$SESSION' started with 5 panes:"
echo "          pane 0: realsense camera   (running now)"
echo "          pane 1: online slam + nav  (starts in ${CAMERA_WARMUP_S}s)"
echo "          pane 2: rosbridge noetic   (starts in ${CAMERA_WARMUP_S}s)"
echo "          pane 3: rosbridge foxy     (starts in ${CAMERA_WARMUP_S}s)"
echo "          pane 4: run_tracker        (starts in ${CAMERA_WARMUP_S}s)"
echo
echo "[robot] attach with:  tmux attach -t $SESSION"
echo "[robot] detach with:  Ctrl-b then d"
echo "[robot] kill all:     tmux kill-session -t $SESSION"
