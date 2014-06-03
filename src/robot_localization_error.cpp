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

namespace robot_localization_error {

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
RobotLocalizationError::RobotLocalizationError() {}
RobotLocalizationError::~RobotLocalizationError() {}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <LocalizationError-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void RobotLocalizationError::readConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle) {
	// configuration fields
	std::string gazebo_link_state_service_name;
	private_node_handle->param("gazebo_link_state_service", gazebo_link_state_service_name, std::string("gazebo/get_link_state"));
	private_node_handle->param("gazebo_ground_truth_link", get_link_state_.request.link_name, std::string(""));

	if (!gazebo_link_state_service_name.empty() && !get_link_state_.request.link_name.empty()) {
		ros::service::waitForService(gazebo_link_state_service_name);
		gazebo_link_state_service_ = node_handle->serviceClient<gazebo_msgs::GetLinkState>(gazebo_link_state_service_name);

		std::string pose_topic_name;
		private_node_handle->param("pose_topic", pose_topic_name, std::string("initialpose"));
		if (!pose_topic_name.empty() && gazebo_link_state_service_.exists()) {
			pose_subscriber_ = node_handle->subscribe(pose_topic_name, 10, &robot_localization_error::RobotLocalizationError::processPoseWithCovarianceStamped, this);

			std::string pose_error_publish_topic_name;
			private_node_handle->param("pose_error_publish_topic", pose_error_publish_topic_name, std::string("localization_error"));
			pose_error_publisher_ = node_handle->advertise<geometry_msgs::PoseStamped>(pose_error_publish_topic_name, 10);

			private_node_handle->param("use_roll_pitch_yaw_angles", use_roll_pitch_yaw_angles_, false);
			private_node_handle->param("use_degrees_in_angles", use_degrees_in_angles_, false);
		}
	}
}


void RobotLocalizationError::processPoseWithCovarianceStamped(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose) {
	gazebo_link_state_service_.call(get_link_state_);

	if (get_link_state_.response.success) {
		geometry_msgs::PoseStamped pose_errors;
		pose_errors.header = pose->header;

		pose_errors.pose.position.x = std::abs(pose->pose.pose.position.x - get_link_state_.response.link_state.pose.position.x);
		pose_errors.pose.position.y = std::abs(pose->pose.pose.position.y - get_link_state_.response.link_state.pose.position.y);
		pose_errors.pose.position.z = std::abs(pose->pose.pose.position.z - get_link_state_.response.link_state.pose.position.z);

		if (use_roll_pitch_yaw_angles_) {
			tf2Scalar roll_pose, pitch_pose, yaw_pose;
			tf2Scalar roll_pose_ground_truth, pitch_pose_ground_truth, yaw_pose_ground_truth;

			getRollPitchYaw(pose->pose.pose.orientation, roll_pose, pitch_pose, yaw_pose);
			getRollPitchYaw(get_link_state_.response.link_state.pose.orientation, roll_pose_ground_truth, pitch_pose_ground_truth, yaw_pose_ground_truth);

			pose_errors.pose.orientation.x = std::abs(roll_pose - roll_pose_ground_truth);
			pose_errors.pose.orientation.y = std::abs(pitch_pose - pitch_pose_ground_truth);
			pose_errors.pose.orientation.z = std::abs(yaw_pose - yaw_pose_ground_truth);

			if (use_degrees_in_angles_) {
				pose_errors.pose.orientation.x = angles::to_degrees(pose_errors.pose.orientation.x);
				pose_errors.pose.orientation.y = angles::to_degrees(pose_errors.pose.orientation.y);
				pose_errors.pose.orientation.z = angles::to_degrees(pose_errors.pose.orientation.z);
			}

			pose_errors.pose.orientation.w = pose_errors.pose.orientation.x + pose_errors.pose.orientation.y + pose_errors.pose.orientation.z;
		} else {
			pose_errors.pose.orientation.x = std::abs(pose->pose.pose.orientation.x - get_link_state_.response.link_state.pose.orientation.x);
			pose_errors.pose.orientation.y = std::abs(pose->pose.pose.orientation.y - get_link_state_.response.link_state.pose.orientation.y);
			pose_errors.pose.orientation.z = std::abs(pose->pose.pose.orientation.z - get_link_state_.response.link_state.pose.orientation.z);
			pose_errors.pose.orientation.w = std::abs(pose->pose.pose.orientation.w - get_link_state_.response.link_state.pose.orientation.w);
		}

		pose_error_publisher_.publish(pose_errors);
	}
}


void RobotLocalizationError::getRollPitchYaw(const geometry_msgs::Quaternion& orientation, tf2Scalar& roll, tf2Scalar& pitch, tf2Scalar& yaw) {
	tf2::Matrix3x3 pose_matrix(tf2::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
	pose_matrix.getRPY(roll, pitch, yaw);
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </LocalizationError-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// =============================================================================   <private-section>   =========================================================================
// =============================================================================   </private-section>  =========================================================================

} /* namespace robot_localization_error */
