#!/bin/bash

gnome-terminal -t "mavros" -x bash -c "#cd ~/ws_sensor_combined/;
#cd ~/ardupilot_ws;
source /opt/ros/humble/setup.bash;
#source install/local_setup.bash;
#ros2 launch ard-mavros apm.launch.xml;
ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@14555;
exec bash;"

#colcon build --allow-overriding mavros

