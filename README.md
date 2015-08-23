robot_localization_tools
========================

ROS package with several tools useful to evaluate a localization system and edit rosbag files.


Localization tools:
- [Retrieve the localization error of a robot using the ROS TF](launch/robot_localization_error.launch)


Rosbag tools:
- [Extract csv data from rosbags](tools/bag2csv.bash)
- [Add TF transforms to rosbags](scripts/add_tf.py)
- [Edit TF transforms in rosbags](scripts/change_tf.py)
- [Remove TF transforms from rosbags](scripts/remove_tf.py)
- [Remove headers frame_ids leading slash from rosbags](scripts/remove_frame_ids_leading_slash.py)
- [Add time offset to messages in rosbags and replace the record time with the header time](scripts/add_time_offset.py)
- [Edit CameraInfo messages in rosbags](scripts/change_camera_info.py)
- [Reset IMU rotation in rosbags](scripts/reset_imu_rotation.py)


Tools to create plots in vector graphics formats (pdf, eps, svgz) using [matplotlib](http://matplotlib.org/):
- [Plot graphs from csv files](scripts/graph_plotter.py)
- [Plot 2D robot paths from pose files](scripts/path_plotter.py)
- [Plot 3D robot paths from pose files](scripts/path_plotter_3d.py)
- [Plot robot velocity and acceleration graphs from pose files (with Savitzky-Golay filtering)](scripts/path_velocity_and_acceleration_plotter.py)
- [Fit probability distributions (normal distribution, lognormal distribution, generalized extreme value distribution) to csv data](scripts/probability_distribution_plotter.py)


Tools to monitor and plot CPU and memory consumptions:
- [Save process CPU and memory consumptions to file](tools/process_monitor.sh)
- [Plot graphs for CPU and memory consumptions of processes](tools/generate_process_monitor_graphs.bash)


Other tools:
- [Publish geometry_msgs::Twist from yaml files (allows a robot to follow a predefined path in Gazebo)](launch/twist_publisher.launch).
- [Monitor ros topics and execute a system command when no msgs are received during a given period.](scripts/topic_supervisor.py)
