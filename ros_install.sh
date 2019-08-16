#!/bin/bash

echo "This script will install ROS on ubuntu. It will check to ensure that you have the correct version of ROS for your os version. Currently it is only set for 16.04"


if [[ $(basename $(dirname $(pwd))) == "src" ]]; then
	echo "IMPORTANT for this file to work correctly this directory should be inside of a catkin workspace. Look at the README about how to do this."

	# Exit out of the script because the repo has to be inside of a catkin workspace
	exit -1

fi



# Check which version we are using
. /etc/lsb-release
ROS_VERSION="UNKNOWN"
if [ "${DISTRIB_RELEASE}" = "18.04" ]; then
  ROS_VERSION="melodic"
	echo "The current OS is 18.04. This will install ROS Melodic"
fi
if [ "${DISTRIB_RELEASE}" = "16.04" ]; then
  ROS_VERSION="kinetic"
	echo "The current OS is 16.04. This will install ROS kinetic"


fi

# Add ROS Kinetic Repo
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Add the key to make sure we are connecting securely to the ROS repo
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

# Make sure we are up to date before we install more things
sudo apt-get -y update
sudo apt-get -y upgrade


# Install git, ros, and a few dependancies
sudo apt-get -y install git htop python-catkin-tools libncurses5-dev libmodbus-dev python-rosinstall python-rosinstall-generator python-wstool build-essential ros-${ROS_VERSION}-desktop-full ros-${ROS_VERSION}-usb-cam ros-${ROS_VERSION}-gazebo-dev ros-${ROS_VERSION}-controller-manager ros-${ROS_VERSION}-gazebo-ros-control ros-${ROS_VERSION}-effort-controllers ros-${ROS_VERSION}-joint-state-controller

if [ "${DISTRIB_RELEASE}" = "16.04" ]; then
  sudo apt-get -y install ros-${ROS_VERSION}-qt-ros ros-${ROS_VERSION}-rosserial ros-${ROS_VERSION}-rosserial-server ros-${ROS_VERSION}-rosserial-arduino ros-${ROS_VERSION}-turtlebot-gazebo
fi

# Add the ROS kinetic setup script to the bashrc so that it gets
# loaded every time you open a bash terminal
echo "source /opt/ros/${ROS_VERSION}/setup.bash" >> ~/.bashrc
source /opt/ros/${ROS_VERSION}/setup.bash

# Source the devel of the account
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/catkin_ws/devel/setup.bash

# Run the ros program
rosrun ROS_tutorials ROS_tutorials_node massage="Install successful!"
