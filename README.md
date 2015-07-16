robot_localization_tools
========================

ROS package with several tools useful to evaluate a localization system.

It has tools to:
- [Retrieve the localization error of a robot using the ROS TF](launch/robot_localization_error.launch)
- [Extract csv data from rosbags](tools/bag2csv.bash)
- [Add TF frames](tools/add_tf.py)
- [Edit TF frames](tools/change_tf.py)
- [Remove TF frames](tools/remove_tf.py)
- [Edit CameraInfo messages](tools/change_camera_info.py)
- [Publish geometry_msgs::Twist from yaml files (allows a robot to follow a predefined path in Gazebo)](launch/twist_publisher.launch).
- [Monitor ros topics and execute a system command when no msgs are received during a given period.](scripts/topic_supervisor.py)

It also has tools to create graphs in vector graphics formats (pdf, eps, svgz) using [matplotlib](http://matplotlib.org/):
- [Plot graphs from csv files](tools/graph_plotter.py)
- [Plot 2D robot paths from pose files](tools/path_plotter.py)
- [Plot 3D robot paths from pose files](tools/path_plotter_3d.py)
- [Plot robot velocity and acceleration graphs from pose files (with Savitzky-Golay filtering)](tools/path_velocity_and_acceleration_plotter.py)
- [Fit probability distributions (normal distribution, lognormal distribution, generalized extreme value distribution) to csv data](tools/probability_distribution_plotter.py)
- [Save process CPU and memory consumptions to file](tools/process_monitor.sh)
- [Plot graphs for CPU and memory consumptions of processes](tools/generate_process_monitor_graphs.bash)

