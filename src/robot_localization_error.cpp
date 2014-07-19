/**\file robot_localization_error.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <robot_localization_error/robot_localization_error.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace robot_localization_tools {

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
RobotLocalizationError::RobotLocalizationError() :
		use_roll_pitch_yaw_angles_(false),
		use_degrees_in_angles_(false),
		use_millimeters_in_distances_(false),
		publish_rate_(10.0) {}

RobotLocalizationError::~RobotLocalizationError() {}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <LocalizationError-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void RobotLocalizationError::readConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle) {
	// configuration fields
	std::string gazebo_link_state_service_name;
	private_node_handle->param("gazebo_link_state_service", gazebo_link_state_service_name, std::string("gazebo/get_link_state"));
	private_node_handle->param("gazebo_ground_truth_link", get_link_state_.request.link_name, std::string(""));
	private_node_handle->param("map_frame_id", map_frame_id_, std::string("map"));
	private_node_handle->param("base_link_frame_id", base_link_frame_id_, std::string("base_footprint"));
	private_node_handle->param("publish_rate", publish_rate_, 10.0);

	if (!gazebo_link_state_service_name.empty() && !get_link_state_.request.link_name.empty()) {
		ros::service::waitForService(gazebo_link_state_service_name);
		gazebo_link_state_service_ = node_handle->serviceClient<gazebo_msgs::GetLinkState>(gazebo_link_state_service_name);

		std::string pose_topic_name;
		private_node_handle->param("pose_topic", pose_topic_name, std::string("initialpose"));
		if (!pose_topic_name.empty() && gazebo_link_state_service_.exists()) {
			pose_subscriber_ = node_handle->subscribe(pose_topic_name, 10, &robot_localization_tools::RobotLocalizationError::processPoseWithCovarianceStamped, this);

			std::string pose_error_publish_topic_name;
			private_node_handle->param("pose_error_publish_topic", pose_error_publish_topic_name, std::string("localization_error"));
			pose_error_publisher_ = node_handle->advertise<geometry_msgs::PoseStamped>(pose_error_publish_topic_name, 10);

			private_node_handle->param("use_roll_pitch_yaw_angles", use_roll_pitch_yaw_angles_, false);
			private_node_handle->param("use_degrees_in_angles", use_degrees_in_angles_, false);
			private_node_handle->param("use_millimeters_in_distances", use_millimeters_in_distances_, false);
		}
	}
}


void RobotLocalizationError::processPoseWithCovarianceStamped(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose) {
	gazebo_link_state_service_.call(get_link_state_);

	if (get_link_state_.response.success) {
		geometry_msgs::PoseStamped pose_errors;
		pose_errors.header = pose->header;

		pose_errors.pose.position.x = pose->pose.pose.position.x - get_link_state_.response.link_state.pose.position.x;
		pose_errors.pose.position.y = pose->pose.pose.position.y - get_link_state_.response.link_state.pose.position.y;
		pose_errors.pose.position.z = pose->pose.pose.position.z - get_link_state_.response.link_state.pose.position.z;

		if (use_millimeters_in_distances_) {
			pose_errors.pose.position.x *= 1000.0;
			pose_errors.pose.position.y *= 1000.0;
			pose_errors.pose.position.z *= 1000.0;
		}

		if (use_roll_pitch_yaw_angles_) {
			tf2Scalar roll_pose, pitch_pose, yaw_pose;
			tf2Scalar roll_pose_ground_truth, pitch_pose_ground_truth, yaw_pose_ground_truth;

			getRollPitchYaw(pose->pose.pose.orientation, roll_pose, pitch_pose, yaw_pose);
			getRollPitchYaw(get_link_state_.response.link_state.pose.orientation, roll_pose_ground_truth, pitch_pose_ground_truth, yaw_pose_ground_truth);

			pose_errors.pose.orientation.x = roll_pose - roll_pose_ground_truth;
			pose_errors.pose.orientation.y = pitch_pose - pitch_pose_ground_truth;
			pose_errors.pose.orientation.z = yaw_pose - yaw_pose_ground_truth;

			if (use_degrees_in_angles_) {
				pose_errors.pose.orientation.x = angles::to_degrees(pose_errors.pose.orientation.x);
				pose_errors.pose.orientation.y = angles::to_degrees(pose_errors.pose.orientation.y);
				pose_errors.pose.orientation.z = angles::to_degrees(pose_errors.pose.orientation.z);
			}

			pose_errors.pose.orientation.w = pose_errors.pose.orientation.x + pose_errors.pose.orientation.y + pose_errors.pose.orientation.z;
		} else {
			pose_errors.pose.orientation.x = pose->pose.pose.orientation.x - get_link_state_.response.link_state.pose.orientation.x;
			pose_errors.pose.orientation.y = pose->pose.pose.orientation.y - get_link_state_.response.link_state.pose.orientation.y;
			pose_errors.pose.orientation.z = pose->pose.pose.orientation.z - get_link_state_.response.link_state.pose.orientation.z;
			pose_errors.pose.orientation.w = pose->pose.pose.orientation.w - get_link_state_.response.link_state.pose.orientation.w;
		}

		pose_error_publisher_.publish(pose_errors);
	}
}


void RobotLocalizationError::getRollPitchYaw(const geometry_msgs::Quaternion& orientation, tf2Scalar& roll, tf2Scalar& pitch, tf2Scalar& yaw) {
	tf2::Matrix3x3 pose_matrix(tf2::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
	pose_matrix.getRPY(roll, pitch, yaw);
}


void RobotLocalizationError::RobotLocalizationError::start() {
	if (publish_rate_ > 0) {
		ros::Rate publish_rate(publish_rate_);
		while (ros::ok()) {
			updateLocalizationError();
			publish_rate.sleep();
			ros::spinOnce();
		}
	} else {
		ros::spin();
	}
}


void RobotLocalizationError::updateLocalizationError() {
	tf2::Transform localization_tf;
	geometry_msgs::PoseWithCovarianceStampedPtr pose(new geometry_msgs::PoseWithCovarianceStamped());
	pose->header.frame_id = map_frame_id_;
	pose->header.stamp = ros::Time::now();

	tf_collector_.lookForTransform(localization_tf, map_frame_id_, base_link_frame_id_, pose->header.stamp);

	pose->pose.pose.position.x = localization_tf.getOrigin().getX();
	pose->pose.pose.position.y = localization_tf.getOrigin().getY();
	pose->pose.pose.position.z = localization_tf.getOrigin().getZ();
	pose->pose.pose.orientation.x = localization_tf.getRotation().getX();
	pose->pose.pose.orientation.y = localization_tf.getRotation().getY();
	pose->pose.pose.orientation.z = localization_tf.getRotation().getZ();
	pose->pose.pose.orientation.w = localization_tf.getRotation().getW();

	processPoseWithCovarianceStamped(pose);
}

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </LocalizationError-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// =============================================================================   <private-section>   =========================================================================
// =============================================================================   </private-section>  =========================================================================

} /* namespace robot_localization_error */

