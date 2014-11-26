robot_localization_tools
========================

ROS package with nodes to retrieve the localization error of a robot using the ROS TF and publish geometry_msgs::Twist from yaml files (allows a robot to follow a predefined path in Gazebo).
It also has tools to edit rosbags (filter topics, modify tf transforms), plot graphs, plot robot paths and fit probability distributions (normal distribution, lognormal distribution, generalized extreme value distribution) from csv files using vector graphics (relies on scipy and matplotlib).
