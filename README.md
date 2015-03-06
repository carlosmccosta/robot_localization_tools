robot_localization_tools
========================

ROS package with several tools useful to evaluate a localization system.

It has tools to:
- [Retrieve the localization error of a robot using the ROS TF](launch/robot_localization_error.launch)
- [Extract csv data from rosbags](tools/bag2csv.sh)
- [Edit TF frames](tools/change_tf.py)
- [Remove TF frames](tools/remove_tf.py)
- [Publish geometry_msgs::Twist from yaml files (allows a robot to follow a predefined path in Gazebo)](launch/twist_publisher.launch).

It also has tools to create graphs in vector graphics formats (pdf, eps, svgz) using [matplotlib](http://matplotlib.org/):
- [Plot graphs from csv files](tools/graph_plotter.py)
- [Plot robot paths from pose files](tools/path_plotter.py)
- [Plot robot velocity and acceleration graphs from pose files (with Savitzky-Golay filtering)](tools/path_velocity_and_acceleration_plotter.py)
- [Fit probability distributions (normal distribution, lognormal distribution, generalized extreme value distribution) to csv data](tools/probability_distribution_plotter.py)

