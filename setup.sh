#!/usr/bin/env bash

#Check if user has  configured Ubuntu repositories(ROS ARM Installation Section 2.1"
echo ""
echo "Have you configured your Ubuntu repositories?"
echo "You must allow \"restricted\", \"universe\" , \"multiverse\"."
echo "https://help.ubuntu.com/community/Repositories/Ubuntu"
echo ""
echo "PRESS [ENTER] TO CONTINUE"
read

#Setup the sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

#Setup the keys
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

#ROS Installation
sudo apt-get -y  update
sudo apt-get -y install ros-indigo-desktop-full

#Initialize rosdep
sudo rosdep init
rosdep update

#Setup Environment
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc

#Install rosinstall
sudo apt-get -y install python-rosinstall

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

#make sure your system is able to compile using CMake
sudo apt-get -y install build-essential

echo ""
echo "ROS Installation Done!"
#echo "You can create your catkin workspace now. https://wiki.ros.org/catkin/Tutorials/create_a_workspace"

sudo apt-get install ros-indigo-moveit
sudo apt-get install ros-indigo-abb
sudo apt-get install ros-indigo-industrial-moveit
mkdir -p ws/src
cd ws/src
catkin_init_workspace 
git clone https://github.com/sunbuny/descartes_demo.git
git clone https://github.com/ros-industrial-consortium/descartes.git
cd ..
catkin_make
source devel/setup.bash 
