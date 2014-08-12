/**\file robot_localization_error.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <robot_localization_tools/robot_localization_error.h>
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
		publish_rate_(100.0),
		invert_tf_from_map_ground_truth_frame_id_(false),
		pose_publishers_sampling_rate_(10),
		last_update_time_(ros::Time::now()),
		number_poses_received_since_last_publish_(0) {}

RobotLocalizationError::~RobotLocalizationError() {}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <LocalizationError-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void RobotLocalizationError::readConfigurationFromParameterServer(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle) {
	// configuration fields
	std::string gazebo_link_state_service_name;
	private_node_handle->param("gazebo_link_state_service", gazebo_link_state_service_name, std::string("/gazebo/get_link_state"));
	private_node_handle->param("gazebo_ground_truth_link", get_link_state_.request.link_name, std::string(""));
	private_node_handle->param("invert_tf_from_map_ground_truth_frame_id", invert_tf_from_map_ground_truth_frame_id_, false);
	private_node_handle->param("map_ground_truth_frame_id", map_ground_truth_frame_id_, std::string("map_ground_truth"));
	private_node_handle->param("map_frame_id", map_frame_id_, std::string("map"));
	private_node_handle->param("odom_frame_id", odom_frame_id_, std::string("odom"));
	private_node_handle->param("base_link_frame_id", base_link_frame_id_, std::string("base_footprint"));
	private_node_handle->param("publish_rate", publish_rate_, 100.0);
	private_node_handle->param("pose_publishers_sampling_rate", pose_publishers_sampling_rate_, 10);

	if (map_ground_truth_frame_id_.empty() && !gazebo_link_state_service_name.empty() && !get_link_state_.request.link_name.empty()) {
		ros::service::waitForService(gazebo_link_state_service_name);
		gazebo_link_state_service_ = node_handle->serviceClient<gazebo_msgs::GetLinkState>(gazebo_link_state_service_name);
	}

	std::string pose_topic_name, pose_with_covariance_topic_name;
	private_node_handle->param("pose_stamped_topic", pose_topic_name, std::string(""));
	private_node_handle->param("pose_stamped_with_covariance_pose_topic", pose_with_covariance_topic_name, std::string("/initialpose"));

	if (!pose_topic_name.empty())
		pose_subscriber_ = node_handle->subscribe(pose_topic_name, 10, &robot_localization_tools::RobotLocalizationError::processPoseStamped, this);

	if (!pose_with_covariance_topic_name.empty())
		pose_with_covariance_subscriber_ = node_handle->subscribe(pose_with_covariance_topic_name, 10, &robot_localization_tools::RobotLocalizationError::processPoseWithCovarianceStamped, this);

	std::string pose_error_publish_topic_name;
	private_node_handle->param("pose_error_publish_topic", pose_error_publish_topic_name, std::string("localization_error"));
	pose_error_publisher_ = node_handle->advertise<geometry_msgs::PoseStamped>(pose_error_publish_topic_name, 10, true);

	std::string localization_poses_publisher_topic;
	private_node_handle->param("localization_poses_publisher_topic", localization_poses_publisher_topic, std::string("localization_poses"));
	localization_poses_publisher_ = node_handle->advertise<geometry_msgs::PoseArray>(localization_poses_publisher_topic, 10, true);

	std::string simulation_poses_publisher_topic;
	private_node_handle->param("simulation_poses_publisher_topic", simulation_poses_publisher_topic, std::string("simulation_poses"));
	simulation_poses_publisher_ = node_handle->advertise<geometry_msgs::PoseArray>(simulation_poses_publisher_topic, 10, true);

	private_node_handle->param("use_roll_pitch_yaw_angles", use_roll_pitch_yaw_angles_, false);
	private_node_handle->param("use_degrees_in_angles", use_degrees_in_angles_, false);
	private_node_handle->param("use_millimeters_in_distances", use_millimeters_in_distances_, false);
}


void RobotLocalizationError::processPoseStamped(const geometry_msgs::PoseStampedConstPtr& pose) {
	if (pose->header.stamp < last_update_time_) return;
	geometry_msgs::Pose simulation_pose;

	if (getSimulationPose(pose->header.stamp, simulation_pose)) {
		geometry_msgs::Pose localization_pose = pose->pose;
		geometry_msgs::PoseStamped pose_errors;
		pose_errors.header = pose->header;

		pose_errors.pose.position.x = localization_pose.position.x - simulation_pose.position.x;
		pose_errors.pose.position.y = localization_pose.position.y - simulation_pose.position.y;
		pose_errors.pose.position.z = localization_pose.position.z - simulation_pose.position.z;

		if (use_millimeters_in_distances_) {
			pose_errors.pose.position.x *= 1000.0;
			pose_errors.pose.position.y *= 1000.0;
			pose_errors.pose.position.z *= 1000.0;
		}

		if (use_roll_pitch_yaw_angles_) {
			tf2Scalar roll_pose, pitch_pose, yaw_pose;
			tf2Scalar roll_pose_ground_truth, pitch_pose_ground_truth, yaw_pose_ground_truth;

			getRollPitchYaw(localization_pose.orientation, roll_pose, pitch_pose, yaw_pose);
			getRollPitchYaw(simulation_pose.orientation, roll_pose_ground_truth, pitch_pose_ground_truth, yaw_pose_ground_truth);

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
			pose_errors.pose.orientation.x = localization_pose.orientation.x - simulation_pose.orientation.x;
			pose_errors.pose.orientation.y = localization_pose.orientation.y - simulation_pose.orientation.y;
			pose_errors.pose.orientation.z = localization_pose.orientation.z - simulation_pose.orientation.z;
			pose_errors.pose.orientation.w = localization_pose.orientation.w - simulation_pose.orientation.w;
		}

		if (!pose_error_publisher_.getTopic().empty()) pose_error_publisher_.publish(pose_errors);

		if (number_poses_received_since_last_publish_ == pose_publishers_sampling_rate_) {
			localization_poses_.poses.push_back(localization_pose);
			if (!localization_poses_publisher_.getTopic().empty()) {
				localization_poses_.header = pose->header;
				localization_poses_publisher_.publish(localization_poses_);
			}

			simulation_poses_.poses.push_back(simulation_pose);
			if (!simulation_poses_publisher_.getTopic().empty()) {
				simulation_poses_.header = pose->header;
				simulation_poses_publisher_.publish(simulation_poses_);
			}

			number_poses_received_since_last_publish_ = 0;
		}

		last_update_time_ = pose->header.stamp;
		++number_poses_received_since_last_publish_;
	}
}


void RobotLocalizationError::processPoseWithCovarianceStamped(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose) {
	geometry_msgs::PoseStampedPtr poseStamped(new geometry_msgs::PoseStamped());
	poseStamped->header = pose->header;
	poseStamped->pose = pose->pose.pose;

	processPoseStamped(poseStamped);
}


bool RobotLocalizationError::getSimulationPose(const ros::Time& time_stamp, geometry_msgs::Pose& simulation_pose_out) {
	if (map_ground_truth_frame_id_.empty() && gazebo_link_state_service_.isValid()) {
		gazebo_link_state_service_.call(get_link_state_);
		if (get_link_state_.response.success) {
			simulation_pose_out = get_link_state_.response.link_state.pose;
			return true;
		}
	} else {
		if (!odom_frame_id_.empty() && invert_tf_from_map_ground_truth_frame_id_) {
			tf2::Transform localization_tf_odom_to_map;
			if (tf_collector_.lookForTransform(localization_tf_odom_to_map, map_ground_truth_frame_id_, odom_frame_id_, time_stamp)) {
				tf2::Transform localization_tf_base_to_odom;
				if (tf_collector_.lookForTransform(localization_tf_base_to_odom, odom_frame_id_, base_link_frame_id_, time_stamp)) {
					tf2::Transform localization_tf = localization_tf_odom_to_map.inverse() * localization_tf_base_to_odom;
					laserscan_to_pointcloud::tf_rosmsg_eigen_conversions::transformTF2ToMsg(localization_tf, simulation_pose_out);
					return true;
				}
			}
		} else {
			tf2::Transform localization_tf;
			if (tf_collector_.lookForTransform(localization_tf, map_ground_truth_frame_id_, base_link_frame_id_, time_stamp)) {
				laserscan_to_pointcloud::tf_rosmsg_eigen_conversions::transformTF2ToMsg(localization_tf, simulation_pose_out);
				return true;
			}
		}
	}

	return false;
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
	ros::Time time_stamp = ros::Time::now();

	if (tf_collector_.lookForTransform(localization_tf, map_frame_id_, base_link_frame_id_, time_stamp)) {
		geometry_msgs::PoseStampedPtr pose(new geometry_msgs::PoseStamped());
		pose->header.stamp = time_stamp;
		pose->header.frame_id = map_frame_id_;
		laserscan_to_pointcloud::tf_rosmsg_eigen_conversions::transformTF2ToMsg(localization_tf, pose->pose);
		processPoseStamped(pose);
	}
}

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </LocalizationError-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// =============================================================================   <private-section>   =========================================================================
// =============================================================================   </private-section>  =========================================================================

} /* namespace robot_localization_error */
