
ardupilot 通过 mavros 与 ros2 节点通信
```sh
#!/bin/bash
gnome-terminal -t "gazebo" -x bash -c "export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/gz_ws/src/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH;
export GZ_SIM_RESOURCE_PATH=$HOME/gz_ws/src/ardupilot_gazebo/models:$HOME/gz_ws/src/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH; 
gz sim -v4 -r iris_runway.sdf;
#gz sim -v4 -r gimbal.sdf;"

sleep 1s
gnome-terminal -t "SITL" -x bash -c "sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console;
"
sleep 10s
gnome-terminal -t "mavros" -x bash -c "
source /opt/ros/humble/setup.bash;
ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@14555;
"
#gnome-terminal -t "mavproxy" -x bash -c "mavproxy.py --console --map --aircraft test --master=:14550"
```
# drone
