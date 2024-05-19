#!/bin/bash

#gnome-terminal -t "ros2" -x bash -c "#cd ~/ws_sensor_combined/;
cd ~/ardupilot_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 run px4_ros_com base_local
#ros2 run px4_ros_com offboard_control_srv
#ros2 run px4_ros_com offboard_control_drone;
#ros2 run px4_ros_com offboard_control_srv_simple;
#ros2 run px4_ros_com test_offboard;
#ros2 run px4_ros_com offboard_control
#ros2 run px4_ros_com pose_subscriber;
#ros2 run px4_ros_com camera_image_subscriber;
#ros2 run px4_ros_com vehicle_global_position_listener;
#ros2 run px4_ros_com vehicle_local_position_listener ;
#ros2 run px4_ros_com vehicle_local_position_publisher;
#ros2 launch px4_ros_com vehicle_local_position_demo.launch.py;
#exec bash;"

#ros2 run px4_ros_com offboard_control_1

#ros2 run px4_ros_com vehicle_gps_position_listener 
#ros2 run px4_ros_com vehicle_gps_global_listener
#ros2 run px4_ros_com vehicle_gps_local_listener 
#ros2 run px4_ros_com sensor_combined_listener
#ros2 launch px4_ros_com sensor_combined_listener.launch.py
#ros2 launch px4_ros_com vehicle_local_position_demo.launch.py

#ros2 run rviz2 rviz2 -d $(ros2 pkg prefix --share turtle_tf2_py)/rviz/turtle_rviz.rviz

