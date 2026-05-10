We are working on a Deep Robotics Lite3 ROS2 Foxy system.

High-level goal:
Explore how to integrate the online SLAM Toolbox map with Nav2 so the robot can eventually accept navigation goals in the live ⁠ map ⁠ frame. Do not assume the correct Nav2 launch/config yet. First inspect the existing Deep Robotics navigation stack and determine the safest integration path.

Current known working online SLAM pipeline:
•⁠  ⁠⁠ /rslidar_points ⁠ is published by the Livox driver, but its timestamps are sensor/uptime timestamps.
•⁠  ⁠⁠ /home/ysc/lite_cog_ros2/system/scripts/tools/restamp_cloud.py ⁠ republishes ⁠ /rslidar_points ⁠ as ⁠ /rslidar_points_restamped ⁠ using current ROS time.
•⁠  ⁠⁠ pointcloud_to_laserscan ⁠ converts ⁠ /rslidar_points_restamped ⁠ to ⁠ /scan ⁠.
•⁠  ⁠⁠ /leg_odom ⁠ is available and is used by ⁠ /home/ysc/lite_cog_ros2/system/scripts/tools/leg_odom_to_tf.py ⁠ to publish ⁠ odom -> rslidar ⁠ using current ROS time.
•⁠  ⁠SLAM Toolbox works with:
  - ⁠ map_frame: map ⁠
  - ⁠ odom_frame: odom ⁠
  - ⁠ base_frame: rslidar ⁠
  - ⁠ scan_topic: /scan ⁠
•⁠  ⁠⁠ /map ⁠ publishes correctly and RViz can display it.

Important context:
•⁠  ⁠The official Deep Robotics saved-map navigation flow appears to use ⁠ hdl_localization ⁠, a saved ⁠ lite3.pcd ⁠, and a saved 2D map.
•⁠  ⁠In this online SLAM experiment, do not run ⁠ hdl_localization ⁠, AMCL, or map_server unless there is a specific reason. SLAM Toolbox should provide ⁠ /map ⁠ and ⁠ map -> odom ⁠.
•⁠  ⁠Do not modify the original Deep Robotics scripts or params in-place. If needed, copy files and create clearly named experimental versions.

Please explore these files/directories:
•⁠  ⁠⁠ /home/ysc/lite_cog_ros2/system/scripts/nav/start_nav.sh ⁠
•⁠  ⁠⁠ /home/ysc/lite_cog_ros2/system/scripts/online_slam/ ⁠
•⁠  ⁠⁠ /home/ysc/lite_cog_ros2/system/scripts/tools/ ⁠
•⁠  ⁠⁠ /home/ysc/lite_cog_ros2/nav ⁠
•⁠  ⁠⁠ /home/ysc/lite_cog_ros2/navigation2-foxy ⁠
•⁠  ⁠Any launch files or params files related to Nav2, ⁠ dr_nav2 ⁠, ⁠ hdl_localization ⁠, ⁠ bt_navigator ⁠, ⁠ controller_server ⁠, ⁠ planner_server ⁠, costmaps, velocity command topics, or robot command bridges.

Exploration tasks:
1.⁠ ⁠Identify what the existing Deep Robotics ⁠ start_nav.sh ⁠ launches.
   - Does it launch ⁠ hdl_localization ⁠?
   - Does it launch Nav2?
   - Does it launch map_server or AMCL?
   - Does it launch RViz?
   - What params files does it use?

2.⁠ ⁠Identify the existing Nav2 params file used by the robot, if any.
   Look for:
   - ⁠ controller_server ⁠
   - ⁠ planner_server ⁠
   - ⁠ bt_navigator ⁠
   - ⁠ behavior_server ⁠
   - ⁠ local_costmap ⁠
   - ⁠ global_costmap ⁠
   - ⁠ robot_base_frame ⁠
   - ⁠ global_frame ⁠
   - ⁠ odom_frame ⁠
   - ⁠ cmd_vel ⁠
   - obstacle sources
   - ⁠ /scan ⁠, ⁠ /rslidar_points ⁠, ⁠ /camera/depth/color/points ⁠, or other sensor topics.

3.⁠ ⁠Identify how velocity commands reach the robot.
   Search for:
   - ⁠ /cmd_vel ⁠
   - ⁠ cmd_vel ⁠
   - ⁠ Twist ⁠
   - velocity command remaps
   - controller bridge nodes
   - Deep Robotics-specific command topics.
   Determine whether Nav2 publishes directly to ⁠ /cmd_vel ⁠ or whether a bridge/remap is required.

4.⁠ ⁠Determine whether the existing Nav2 stack can be launched in “navigation only” mode, without:
   - AMCL
   - map_server
   - hdl_localization
   - saved-map localization.
   The desired online-SLAM architecture is:
   ⁠ /scan -> slam_toolbox -> /map + map->odom -> Nav2 -> cmd_vel ⁠.

5.⁠ ⁠Propose, then implement if straightforward, a copied experimental Nav2 params file:
   ⁠ /home/ysc/lite_cog_ros2/system/config/nav2_slam_toolbox_lite3.yaml ⁠

   It should be based on the existing robot Nav2 params where possible, but adapted for online SLAM:
   - ⁠ use_sim_time: false ⁠
   - ⁠ global_frame: map ⁠
   - ⁠ odom_frame: odom ⁠
   - ⁠ robot_base_frame: rslidar ⁠ for the current proof of concept
   - local costmap global frame: ⁠ odom ⁠
   - global costmap global frame: ⁠ map ⁠
   - obstacle source: ⁠ /scan ⁠
   - data type: ⁠ LaserScan ⁠
   - avoid AMCL/map_server-specific assumptions.

6.⁠ ⁠Create an experimental headless launcher only if the required Nav2 command can be determined:
   ⁠ /home/ysc/lite_cog_ros2/system/scripts/online_slam/start_online_slam_nav_headless.sh ⁠

   It should start:
   - LiDAR
   - ⁠ restamp_cloud.py ⁠
   - ⁠ pointcloud_to_laserscan ⁠
   - ⁠ leg_odom_to_tf.py ⁠
   - SLAM Toolbox with ⁠ /home/ysc/lite_cog_ros2/system/config/slam_toolbox_lite3.yaml ⁠
   - Nav2 navigation-only bringup with the experimental params file.

   Requirements:
   - Headless: no ⁠ gnome-terminal ⁠, no RViz.
   - Background processes with logs.
   - Timestamped log directory under ⁠ /home/ysc/lite_cog_ros2/system/log/ ⁠.
   - PID file for cleanup.
   - Do not automatically send any goal or command motion.

7.⁠ ⁠Create a matching stop script:
   ⁠ /home/ysc/lite_cog_ros2/system/scripts/online_slam/stop_online_slam_nav_headless.sh ⁠

8.⁠ ⁠Add or print a verification checklist:
   - ⁠ ros2 topic hz /scan ⁠
   - ⁠ ros2 topic hz /map ⁠
   - ⁠ ros2 run tf2_ros tf2_echo odom rslidar ⁠
   - ⁠ ros2 run tf2_ros tf2_echo map odom ⁠
   - ⁠ ros2 lifecycle nodes ⁠
   - ⁠ ros2 topic list | grep -E "cmd_vel|goal_pose|navigate|costmap|plan" ⁠
   - ⁠ ros2 topic echo /cmd_vel ⁠

9.⁠ ⁠Add manual test commands for tiny goals, but do not run them automatically:
   - ⁠ /goal_pose ⁠ example in ⁠ map ⁠ frame
   - ⁠ /navigate_to_pose ⁠ action example in ⁠ map ⁠ frame
   Use a tiny goal such as x=0.5, y=0.0, yaw=0.

Safety:
•⁠  ⁠Do not command robot motion automatically.
•⁠  ⁠Preserve original Deep Robotics navigation files.
•⁠  ⁠Clearly mark all new files as experimental online-SLAM/Nav2 integration.
•⁠  ⁠Mention that using ⁠ rslidar ⁠ as ⁠ robot_base_frame ⁠ is acceptable only for proof-of-concept; the correct long-term solution is to identify the real robot body frame and static transform between robot body and LiDAR.

Deliverable:
At the end, summarize:
•⁠  ⁠What files were inspected.
•⁠  ⁠What launch files/params are used by the original navigation stack.
•⁠  ⁠What command topic the robot appears to use.
•⁠  ⁠Whether Nav2 can be launched navigation-only.
•⁠  ⁠What new experimental files were created.
•⁠  ⁠Exact commands to start, verify, stop, and manually send a tiny goal.