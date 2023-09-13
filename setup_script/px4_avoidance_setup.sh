#!/bin/bash

sudo apt install curl
mkdir ~/tmp
cd ~/tmp
curl -O https://repo.anaconda.com/archive/Anaconda3-2023.07-2-Linux-x86_64.sh
bash Anaconda3-2023.07-2-Linux-x86_64.sh -b -u
source ~/anaconda3/bin/activate
conda init bash
conda create --name Aerialist python=3.9 -y
source ~/anaconda3/bin/activate Aerialist
pip3 install future
pip3 install kconfiglib
pip3 install empy
pip3 install future
pip3 install packaging
pip3 install toml
pip3 install numpy
pip3 install jinja2
pip3 install cython
pip3 install pyyaml
pip3 install future
pip3 install --user pyros-genmsg
pip3 install --user jsonschema
cd ~
wget https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_ros_melodic.sh

source ~/anaconda3/bin/activate Aerialist
cd ~
source ubuntu_sim_ros_melodic.sh
sudo apt-get install ros-melodic-geographic-msgs -y
cd ~
source ubuntu_sim_ros_melodic.sh

source ~/anaconda3/bin/activate Aerialist


sudo apt install openjdk-8-jdk
sudo update-alternatives --config java -2
file_path="/etc/java-8-openjdk/accessibility.properties"
sed -i '$s/^/#/' "$file_path"
sudo apt install ros-melodic-mavros ros-melodic-mavros-extras -y
sudo apt install libpcl1 ros-melodic-octomap-* -y
sudo apt-get install libgstreamer-plugins-base1.0-dev -y

cd ~
git clone https://github.com/PX4/Firmware.git --recursive
cd ~/Firmware

git fetch origin release/1.13
git checkout release/1.13
make submodulesclean

cd ~/catkin_ws/src
git clone https://github.com/PX4/avoidance.git

catkin build -w ~/catkin_ws

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
