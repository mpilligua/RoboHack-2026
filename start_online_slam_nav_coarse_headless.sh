#!/bin/bash
set -e

echo "Stopping old SLAM/Nav2 processes..."

pkill -f slam_toolbox || true
pkill -f pointcloud_to_laserscan || true
pkill -f restamp_cloud.py || true
pkill -f leg_odom_to_tf.py || true
pkill -f livox_lidar_publisher || true
pkill -f start_livox.sh || true
pkill -f controller_server || true
pkill -f planner_server || true
pkill -f bt_navigator || true
pkill -f behavior_server || true
pkill -f recoveries_server || true
pkill -f lifecycle_manager || true
pkill -f waypoint_follower || true
pkill -f nav2 || true

sleep 2

CONFIG_DIR="/home/ysc/lite_cog_ros2/system/config"
TOOLS_DIR="/home/ysc/lite_cog_ros2/system/scripts/tools"
LOG_DIR="/home/ysc/lite_cog_ros2/system/log/online_slam_nav_coarse_$(date +%Y%m%d_%H%M%S)"
PID_FILE="/home/ysc/lite_cog_ros2/system/log/online_slam_nav_coarse_pids.txt"

SLAM_CONFIG="$CONFIG_DIR/slam_toolbox_lite3.yaml"
NAV2_CONFIG="$CONFIG_DIR/nav2_slam_toolbox_lite3.yaml"

mkdir -p "$CONFIG_DIR" "$LOG_DIR"
: > "$PID_FILE"

echo "Starting online SLAM + Nav2 with coarser navigation costmaps..."
echo "Logs: $LOG_DIR"
echo "PID file: $PID_FILE"

if [ ! -f "$TOOLS_DIR/restamp_cloud.py" ]; then
  echo "ERROR: Missing $TOOLS_DIR/restamp_cloud.py"
  exit 1
fi

if [ ! -f "$TOOLS_DIR/leg_odom_to_tf.py" ]; then
  echo "ERROR: Missing $TOOLS_DIR/leg_odom_to_tf.py"
  exit 1
fi

if [ ! -f "$NAV2_CONFIG" ]; then
  echo "ERROR: Missing $NAV2_CONFIG"
  echo "Create it first:"
  echo "  cp /home/ysc/lite_cog_ros2/nav/src/dr_nav2/config/lite_nav2.yaml $NAV2_CONFIG"
  exit 1
fi

cp "$NAV2_CONFIG" "$NAV2_CONFIG.backup_$(date +%Y%m%d_%H%M%S)"

# SLAM config: moderate quality, less artifact-prone.
cat > "$SLAM_CONFIG" <<'YAML'
slam_toolbox:
  ros__parameters:
    use_sim_time: false
    odom_frame: odom
    map_frame: map
    base_frame: rslidar
    scan_topic: /scan
    mode: mapping

    resolution: 0.10
    transform_timeout: 0.5
    transform_publish_period: 0.05
    map_update_interval: 2.0
    minimum_time_interval: 0.15
    throttle_scans: 1
    max_laser_range: 8.0

async_slam_toolbox_node:
  ros__parameters:
    use_sim_time: false
    odom_frame: odom
    map_frame: map
    base_frame: rslidar
    scan_topic: /scan
    mode: mapping

    resolution: 0.10
    transform_timeout: 0.5
    transform_publish_period: 0.05
    map_update_interval: 2.0
    minimum_time_interval: 0.15
    throttle_scans: 1
    max_laser_range: 8.0
YAML

# Patch Nav2 config:
# - rslidar as proof-of-concept base frame
# - local costmap 0.15 m
# - global costmap 0.20 m
# - /scan obstacle layer
# - no active STVL
# - slower global planning/costmap rates
python3 - <<'PY'
from pathlib import Path
import yaml

p = Path("/home/ysc/lite_cog_ros2/system/config/nav2_slam_toolbox_lite3.yaml")
text = p.read_text().replace("\t", "  ")
data = yaml.safe_load(text) or {}

def ensure_node(name):
    data.setdefault(name, {})
    data[name].setdefault("ros__parameters", {})
    return data[name]["ros__parameters"]

def set_use_sim_time_false(obj):
    if isinstance(obj, dict):
        for k, v in obj.items():
            if k == "use_sim_time":
                obj[k] = False
            else:
                set_use_sim_time_false(v)
    elif isinstance(obj, list):
        for item in obj:
            set_use_sim_time_false(item)

set_use_sim_time_false(data)

# BT navigator
bt = ensure_node("bt_navigator")
bt["use_sim_time"] = False
bt["global_frame"] = "map"
bt["robot_base_frame"] = "rslidar"
bt["odom_topic"] = "/leg_odom2"

# Planner
planner = ensure_node("planner_server")
planner["use_sim_time"] = False
planner["expected_planner_frequency"] = 1.0
planner["planner_plugins"] = ["GridBased"]
planner["GridBased"] = {
    "plugin": "nav2_navfn_planner/NavfnPlanner",
    "tolerance": 1.0,
    "use_astar": True,
    "allow_unknown": False,
}

# Local costmap: moderate detail for obstacle avoidance
data.setdefault("local_costmap", {})
data["local_costmap"].setdefault("local_costmap", {})
local = data["local_costmap"]["local_costmap"].setdefault("ros__parameters", {})
local.update({
    "use_sim_time": False,
    "always_send_full_costmap": True,
    "global_frame": "odom",
    "robot_base_frame": "rslidar",
    "rolling_window": True,
    "track_unknown_space": False,
    "width": 4,
    "height": 4,
    "resolution": 0.15,
    "update_frequency": 3.0,
    "publish_frequency": 1.0,
    "transform_tolerance": 0.5,
    "footprint_padding": 0.10,
    "footprint": "[[0.45, 0.30], [0.45, -0.30], [-0.45, -0.30], [-0.45, 0.30]]",
    "plugins": ["obstacle_layer", "inflation_layer"],
})
local["obstacle_layer"] = {
    "plugin": "nav2_costmap_2d::ObstacleLayer",
    "enabled": True,
    "observation_sources": "scan",
    "scan": {
        "topic": "/scan",
        "sensor_frame": "rslidar",
        "data_type": "LaserScan",
        "marking": True,
        "clearing": True,
        "obstacle_range": 4.0,
        "raytrace_range": 5.0,
        "inf_is_valid": True,
    },
}
local["inflation_layer"] = {
    "plugin": "nav2_costmap_2d::InflationLayer",
    "inflation_radius": 0.60,
    "cost_scaling_factor": 3.0,
    "inflate_unknown": False,
    "inflate_around_unknown": False,
}

# Global costmap: coarser for faster path planning
data.setdefault("global_costmap", {})
data["global_costmap"].setdefault("global_costmap", {})
global_cm = data["global_costmap"]["global_costmap"].setdefault("ros__parameters", {})
global_cm.update({
    "use_sim_time": False,
    "always_send_full_costmap": True,
    "global_frame": "map",
    "robot_base_frame": "rslidar",
    "rolling_window": False,
    "track_unknown_space": True,
    "width": 30,
    "height": 30,
    "resolution": 0.20,
    "update_frequency": 0.5,
    "publish_frequency": 0.25,
    "transform_tolerance": 0.5,
    "footprint_padding": 0.10,
    "footprint": "[[0.45, 0.30], [0.45, -0.30], [-0.45, -0.30], [-0.45, 0.30]]",
    "plugins": ["static_layer", "obstacle_layer", "inflation_layer"],
})
global_cm["static_layer"] = {
    "plugin": "nav2_costmap_2d::StaticLayer",
    "enabled": True,
    "map_topic": "/map",
    "subscribe_to_updates": True,
    "map_subscribe_transient_local": True,
    "track_unknown_space": True,
    "use_maximum": False,
    "trinary_costmap": True,
    "transform_tolerance": 0.5,
}
global_cm["obstacle_layer"] = {
    "plugin": "nav2_costmap_2d::ObstacleLayer",
    "enabled": True,
    "observation_sources": "scan",
    "scan": {
        "topic": "/scan",
        "sensor_frame": "rslidar",
        "data_type": "LaserScan",
        "marking": True,
        "clearing": True,
        "obstacle_range": 4.0,
        "raytrace_range": 5.0,
        "inf_is_valid": True,
    },
}
global_cm["inflation_layer"] = {
    "plugin": "nav2_costmap_2d::InflationLayer",
    "inflation_radius": 0.60,
    "cost_scaling_factor": 3.0,
    "inflate_unknown": False,
    "inflate_around_unknown": False,
}

# Controller
controller = ensure_node("controller_server")
controller["use_sim_time"] = False
controller["odom_topic"] = "leg_odom2"
controller["controller_frequency"] = 5.0
controller["controller_plugins"] = ["FollowPath"]
controller["min_x_velocity_threshold"] = 0.001
controller["min_y_velocity_threshold"] = 0.001
controller["min_theta_velocity_threshold"] = 0.001

controller.setdefault("progress_checker", {
    "plugin": "nav2_controller::SimpleProgressChecker",
    "required_movement_radius": 0.5,
    "movement_time_allowance": 10.0,
})
controller.setdefault("goal_checker", {
    "plugin": "nav2_controller::SimpleGoalChecker",
    "xy_goal_tolerance": 0.4,
    "yaw_goal_tolerance": 0.4,
    "stateful": True,
})

follow = controller.setdefault("FollowPath", {})
follow["plugin"] = follow.get("plugin", "dwb_core::DWBLocalPlanner")

# First tests: no sideways velocity.
follow["min_vel_y"] = 0.0
follow["max_vel_y"] = 0.0
follow["acc_lim_y"] = 0.0
follow["decel_lim_y"] = 0.0
follow["vy_samples"] = 1

# Conservative motion.
follow["min_vel_x"] = 0.0
follow["max_vel_x"] = 0.35
follow["max_vel_theta"] = 0.5
follow["min_speed_xy"] = 0.0
follow["max_speed_xy"] = 0.35
follow["min_speed_theta"] = 0.0
follow["acc_lim_x"] = 0.4
follow["acc_lim_theta"] = 0.5
follow["decel_lim_x"] = -0.4
follow["decel_lim_theta"] = -0.5

follow["vx_samples"] = 12
follow["vtheta_samples"] = 20
follow["sim_time"] = 2.0
follow["linear_granularity"] = 0.10
follow["angular_granularity"] = 0.10
follow["transform_tolerance"] = 0.3
follow["short_circuit_trajectory_evaluation"] = True

critics = ["PreferForward", "GoalDist", "PathDist", "RotateToGoal", "BaseObstacle"]
follow["critics"] = critics
follow["PreferForward.scale"] = 40.0
follow["GoalDist.scale"] = 60.0
follow["PathDist.scale"] = 20.0
follow["RotateToGoal.scale"] = 32.0
follow["BaseObstacle.scale"] = 0.5
follow["xy_goal_tolerance"] = 0.4
follow["trans_stopped_velocity"] = 0.05

# Disable heavy debug publishing.
for k in [
    "publish_evaluation",
    "publish_global_plan",
    "publish_transformed_plan",
    "publish_local_plan",
    "publish_trajectories",
    "publish_cost_grid_pc",
]:
    follow[k] = False

# recoveries_server / behavior_server compatibility
for node_name in ["recoveries_server", "behavior_server"]:
    if node_name in data:
        params = data[node_name].setdefault("ros__parameters", {})
        params["use_sim_time"] = False
        params["global_frame"] = "map"
        params["robot_base_frame"] = "rslidar"
        params["transform_tolerance"] = 0.5

# Leave map_server block if present; navigation_launch.py should not start it.
# Remove active base_link references where possible.
def replace_base_link(obj):
    if isinstance(obj, dict):
        for k, v in obj.items():
            if isinstance(v, str) and v == "base_link":
                obj[k] = "rslidar"
            else:
                replace_base_link(v)
    elif isinstance(obj, list):
        for item in obj:
            replace_base_link(item)

replace_base_link(data)

p.write_text(yaml.safe_dump(data, sort_keys=False))
yaml.safe_load(open(p))
print("Patched and validated Nav2 config:", p)
PY

python3 - <<'PY'
import yaml
for f in [
    "/home/ysc/lite_cog_ros2/system/config/slam_toolbox_lite3.yaml",
    "/home/ysc/lite_cog_ros2/system/config/nav2_slam_toolbox_lite3.yaml",
]:
    yaml.safe_load(open(f))
    print("YAML OK:", f)
PY

start_bg() {
  NAME="$1"
  CMD="$2"
  LOG="$LOG_DIR/$NAME.log"

  echo "Starting $NAME..."
  bash -lc "$CMD" > "$LOG" 2>&1 &
  PID=$!
  echo "$PID $NAME" >> "$PID_FILE"
  echo "  PID=$PID log=$LOG"
}

# 1. LiDAR
start_bg "lidar" "
source /opt/ros/foxy/setup.bash
source /home/ysc/lite_cog_ros2/driver/mid360_ws/install/setup.bash
cd /home/ysc/lite_cog_ros2/system/scripts/lidar
./start_livox.sh
"

sleep 4

# 2. Restamp point cloud
start_bg "restamp_cloud" "
source /opt/ros/foxy/setup.bash
python3 $TOOLS_DIR/restamp_cloud.py
"

sleep 2

# 3. PointCloud2 -> LaserScan.
# Cleaner navigation mapping:
# - 1 degree scan
# - 8 m range
# - height band that avoids floor noise but keeps obstacles
start_bg "pointcloud_to_laserscan" "
source /opt/ros/foxy/setup.bash
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
  --ros-args \
  -r cloud_in:=/rslidar_points_restamped \
  -r scan:=/scan \
  -p target_frame:=rslidar \
  -p min_height:=0.05 \
  -p max_height:=1.0 \
  -p angle_min:=-3.14159 \
  -p angle_max:=3.14159 \
  -p angle_increment:=0.01745 \
  -p scan_time:=0.15 \
  -p range_min:=0.2 \
  -p range_max:=8.0 \
  -p use_inf:=true
"

sleep 2

# 4. leg_odom -> odom->rslidar TF
start_bg "leg_odom_to_tf" "
source /opt/ros/foxy/setup.bash
python3 $TOOLS_DIR/leg_odom_to_tf.py
"

sleep 2

# 5. SLAM Toolbox
start_bg "slam_toolbox" "
source /opt/ros/foxy/setup.bash
source /home/ysc/lite_cog_ros2/slam/install/setup.bash 2>/dev/null || true
ros2 launch slam_toolbox online_async_launch.py \
  params_file:=$SLAM_CONFIG
"

echo "Waiting for SLAM to initialize and publish /map..."
sleep 12

# 6. Nav2 navigation-only
start_bg "nav2_navigation" "
source /opt/ros/foxy/setup.bash
source /home/ysc/lite_cog_ros2/navigation2-foxy/install/setup.bash
source /home/ysc/lite_cog_ros2/nav/install/setup.bash
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=$NAV2_CONFIG
"

echo
echo "Started online SLAM + Nav2 with coarser navigation costmaps."
echo
echo "Verify SLAM:"
echo "  ros2 topic hz /scan"
echo "  ros2 topic hz /map"
echo "  ros2 run tf2_ros tf2_echo odom rslidar"
echo "  ros2 run tf2_ros tf2_echo map odom"
echo "  ros2 topic echo --once /map | grep -E 'resolution|width|height' -A 2"
echo
echo "Verify costmap resolutions:"
echo "  ros2 topic echo --once /local_costmap/costmap | grep -E 'frame_id|resolution|width|height' -A 6"
echo "  ros2 topic echo --once /global_costmap/costmap | grep -E 'frame_id|resolution|width|height' -A 6"
echo
echo "Verify obstacle subscriptions:"
echo "  ros2 node info /local_costmap/local_costmap | grep /scan -A 2 -B 2"
echo "  ros2 node info /global_costmap/global_costmap | grep /scan -A 2 -B 2"
echo
echo "Verify Nav2:"
echo "  ros2 lifecycle nodes"
echo "  ros2 topic list | grep -Ei 'cmd|vel|twist|goal|navigate|costmap|plan'"
echo "  ros2 action list | grep navigate"
echo
echo "Watch command output:"
echo "  ros2 topic echo /cmd_vel"
echo
echo "Tiny manual goal:"
echo "  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \"{pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.5, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}\""
echo
echo "Logs:"
echo "  tail -f $LOG_DIR/slam_toolbox.log"
echo "  tail -f $LOG_DIR/nav2_navigation.log"
