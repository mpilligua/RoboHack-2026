# ROS2 Topic Message Types

Generated on this machine using `ros2 topic type`, `ros2 topic list -t`, and `ros2 interface show`.

## Key control topics

### Found live at runtime

- `/cmd_vel` -> `geometry_msgs/msg/Twist`
- `/simple_cmd` -> `transfer_interfaces/msg/MotionSimpleCMD`
- `/complex_cmd` -> `transfer_interfaces/msg/MotionComplexCMD`

### Not currently active in graph, but implemented in ROS2 code

`basic_goal_controller` in `lite_cog_ros2/track/src/basic_goal_controller.py` defines:

- `/basic_goal` -> `geometry_msgs/msg/Pose2D`
- `/basic_goal_cancel` -> `std_msgs/msg/String`
- `/basic_goal_status` -> `std_msgs/msg/String`
- `/odom` -> `nav_msgs/msg/Odometry` (subscription input)
- `/cmd_vel` -> `geometry_msgs/msg/Twist` (publisher output)

## Custom transfer interfaces

### `transfer_interfaces/msg/MotionSimpleCMD`

```text
int32 cmd_code
int32 size
int32 type
```

### `transfer_interfaces/msg/MotionComplexCMD`

```text
int32 cmd_code
int32 size
int32 type
float64 data
```

## Standard message definitions used by above topics

### `geometry_msgs/msg/Pose2D`

```text
float64 x
float64 y
float64 theta
```

### `std_msgs/msg/String`

```text
string data
```

### `geometry_msgs/msg/Twist`

```text
Vector3  linear
Vector3  angular
```

### `nav_msgs/msg/Odometry`

```text
std_msgs/Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
```

## Notes

- `ros2 topic type` returns empty for topics that are not currently being published/subscribed in the active ROS graph.
- In this session, `/basic_goal*` and `/odom` were not active, but their types are fixed by the ROS2 node implementation above.
