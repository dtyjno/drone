#!/bin/bash

gnome-terminal -t "ros2" -x bash -c "#cd ~/ws_sensor_combined/;
#cd ~/ros2_ws;
cd ~/ardupilot_ws;
source /opt/ros/humble/setup.bash;
#colcon build;
colcon build --packages-select px4_ros_com; 
#cd ~/ros2_ws;
#colcon build --packages-up-to ardupilot_gz_bringup;
exec bash;"

cd ~/ardupilot_ws;
source /opt/ros/humble/setup.bash;
MAKEFLAGS="-j1 " colcon build --executor sequential --event-handlers console_direct+ --cmake-args -DBUILD_TESTING=ON;

