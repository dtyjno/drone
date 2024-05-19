#!/bin/bash

#安装mavros-humble
#添加清华源
sudo apt install curl gnupg2
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
#发行版Ubuntu 22.04 LTS (jammy)
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu jammy main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update

sudo apt-get install ros-humble-mavros 
sudo apt-get install ros-humble-mavros-extras

#clone git仓库
cd ~/
mkdir ardupilot_ws
cd ardupilot_ws
git clone https://github.com/dtyjno/drone.git
#git clone git@github.com:dtyjno/drone.git
mv drone src


#GeographicLib 用于gps
cd ~/Downloads
wget https://github.com/geographiclib/geographiclib/archive/refs/tags/v2.3.tar.gz
tar xfpz v2.3.tar.gz 
cd geographiclib-2.3 
#创建一个单独的构建目录并输入它，例如，
mkdir BUILD
cd BUILD 
#配置软件，指定源目录的路径，使用
#../configure 
#make
#make install
 cmake ..  
  make 

# colcon无法找到  geographiclib-config.cmake
  sudo ln -s /usr/share/cmake/geographiclib/FindGeographicLib.cmake /usr/share/cmake-3.22/Modules/
  
  
cd ~/ardupilot_ws
rosdep install --from-paths src --ignore-src -r -y

