/**\file twist_publisher_node.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <ros/ros.h>
#include <robot_localization_tools/twist_publisher.h>
#include <cstdlib>
#include <cstring>
#include <sstream>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<



// ###################################################################################   <main>   ##############################################################################
int main(int argc, char** argv) {
	ros::init(argc, argv, "rlt_twist_publisher");

	ros::NodeHandlePtr node_handle(new ros::NodeHandle());
	ros::NodeHandlePtr private_node_handle(new ros::NodeHandle("~"));

	robot_localization_tools::TwistPublisher twist_publisher;
	twist_publisher.setupConfigurationFromParameterServer(node_handle, private_node_handle);
	twist_publisher.publishTwistFromParameterServer(private_node_handle);

	if (argc > 1) {
		std::stringstream ss;
		ss << argv[1];
		for (int i = 2; i < argc; ++i) {
			if (strcmp(argv[i], "AND") == 0) {
				ss << " &&";
			} else {
				ss << " " << argv[i];
			}
		}

		ROS_INFO_STREAM("Calling system command: " << ss.str());
		std::system(ss.str().c_str());
	}

	return 0;
}
// ###################################################################################   </main>   #############################################################################
