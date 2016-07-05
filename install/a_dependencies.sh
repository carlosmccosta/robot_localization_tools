#!/usr/bin/env sh

ros_version=${1:-"$(rosversion -d)"}
install_args=${2:-"-y --allow-unauthenticated"}


echo "####################################################################################################"
echo "##### Checking and installing dependencies for robot_localization_tools ros package"
echo "####################################################################################################"

sudo apt-get update
sudo apt-get upgrade ${install_args}
sudo apt-get dist-upgrade ${install_args}

# required system dependencies
sudo apt-get install collectl ${install_args}
sudo apt-get install coreutils ${install_args}
sudo apt-get install git ${install_args}
sudo apt-get install libav-tools ${install_args}
sudo apt-get install linux-tools ${install_args}
sudo apt-get install python-matplotlib ${install_args}
sudo apt-get install python-numpy ${install_args}
sudo apt-get install python-scipy ${install_args}
sudo apt-get install python-yaml ${install_args}
sudo apt-get install python-wstool ${install_args}
pip install --user git+git://github.com/moble/quaternion

# required ros packages
sudo apt-get install ros-${ros_version}-angles ${install_args}
sudo apt-get install ros-${ros_version}-geometry-msgs ${install_args}
sudo apt-get install ros-${ros_version}-message-generation ${install_args}
sudo apt-get install ros-${ros_version}-message-runtime ${install_args}
sudo apt-get install ros-${ros_version}-rosbag ${install_args}
sudo apt-get install ros-${ros_version}-roscpp ${install_args}
sudo apt-get install ros-${ros_version}-rosconsole ${install_args}
sudo apt-get install ros-${ros_version}-roslib ${install_args}
sudo apt-get install ros-${ros_version}-rospy ${install_args}
sudo apt-get install ros-${ros_version}-sensor-msgs ${install_args}
sudo apt-get install ros-${ros_version}-std-msgs ${install_args}
sudo apt-get install ros-${ros_version}-std-srvs ${install_args}
sudo apt-get install ros-${ros_version}-tf ${install_args}
sudo apt-get install ros-${ros_version}-tf2 ${install_args}
sudo apt-get install ros-${ros_version}-tf2-msgs ${install_args}


sudo apt-get upgrade ${install_args}
sudo apt-get dist-upgrade ${install_args}


echo "\n\n"
echo "----------------------------------------------------------------------------------------------------"
echo ">>>>> Installation of dependencies finished"
echo "----------------------------------------------------------------------------------------------------"
