#!/bin/bash
#gnome-terminal -t "gazebo" -x bash -c "ros2 launch ardupilot_gz_bringup iris_runway.launch.py;exec bash;"
sleep 1s
#UDP（推荐用于 SITL）
#gnome-terminal -t "SITL" -x bash -c " cd ~/ros2_ws/;ros2 launch ardupilot_sitl sitl_dds_udp.launch.py transport:=udp4 refs:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/dds_xrce_profile.xml synthetic_clock:=True wipe:=False model:=quad speedup:=1 slave:=0 instance:=0 defaults:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/copter.parm,$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/dds_udp.parm sim_address:=127.0.0.1 master:=tcp:127.0.0.1:5760 sitl:=127.0.0.1:5501"

#串行
gnome-terminal -t "SITL" -x bash -c " cd ~/ros2_ws/;
source /opt/ros/humble/setup.bash;
source install/local_setup.bash;
ros2 launch ardupilot_sitl sitl_dds_serial.launch.py \
\
tty0:=./dev/ttyROS0 \
tty1:=./dev/ttyROS1 \
\
transport:=serial \
refs:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/dds_xrce_profile.xml \
baudrate:=115200 \
device:=./dev/ttyROS0 \
\
synthetic_clock:=True \
wipe:=True \
model:=quad \
speedup:=1 \
slave:=0 \
instance:=0 \
serial1:=uart:./dev/ttyROS1 \
defaults:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/copter.parm,$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/dds_serial.parm \
sim_address:=127.0.0.1 \
\
master:=tcp:127.0.0.1:5760 \
sitl:=127.0.0.1:5501;
"
sleep 1s
gnome-terminal -t "mavproxy" -x bash -c "mavproxy.py --console --map --aircraft test --master=:14550"
