#!/bin/bash
sudo apt update
sudo apt install -y curl gnupg lsb-release
sudo cp ros-archive-keyring.gpg /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-humble-desktop
sudo apt install -y ros-dev-tools ros-humble-geographic-msgs ros-humble-topic-tools ros-humble-topic-tools-interfaces ros-humble-mavros ros-humble-mavros-msgs
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
pip install --user -U empy==3.3.4 pyros-genmsg setuptools
ros2 run turtlesim turtlesim_node &
ros2 run turtlesim turtle_teleop_key
