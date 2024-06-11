#!/bin/bash

gnome-terminal -t "ros2" -x bash -c "
cd ~/ros2CV/cv_ws;
source /opt/ros/humble/setup.bash;
source install/local_setup.bash;
ros2 run topic2 image_sub_gazebo;
exec bash;"

