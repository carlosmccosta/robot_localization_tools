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
		use_degrees_in_angles_(false),
		use_millimeters_in_distances_(false),
		publish_rate_(100.0),
		invert_tf_from_map_ground_truth_frame_id_(false),
		pose_publishers_sampling_rate_(10),
		save_poses_timestamp_(true),
		save_poses_orientation_quaternion_(true),
		save_poses_orientation_vector_(true),
		tf_lookup_timeout_(0),
		last_update_time_(ros::Time::now()),
		number_poses_received_since_last_publish_(0) {}

RobotLocalizationError::~RobotLocalizationError() {}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <LocalizationError-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
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

	if (pose_publishers_sampling_rate_ < 1) pose_publishers_sampling_rate_ = 1;

	double tf_lookup_timeout;
	private_node_handle->param("tf_lookup_timeout", tf_lookup_timeout, 0.1);
	tf_lookup_timeout_.fromSec(tf_lookup_timeout);

	std::string localization_poses_output_filename, ground_truth_poses_output_filename;
	private_node_handle->param("localization_poses_output_filename", localization_poses_output_filename, std::string(""));
	private_node_handle->param("ground_truth_poses_output_filename", ground_truth_poses_output_filename, std::string(""));
	private_node_handle->param("save_poses_timestamp", save_poses_timestamp_, true);
	private_node_handle->param("save_poses_orientation_quaternion", save_poses_orientation_quaternion_, true);
	private_node_handle->param("save_poses_orientation_vector", save_poses_orientation_vector_, true);

	if (!localization_poses_output_filename.empty()) {
		localization_poses_output_stream_.open(localization_poses_output_filename.c_str());
		addPoseFileHeader(localization_poses_output_stream_, "# poses from localization system");
		ROS_INFO_STREAM("Saving localization poses to file " << localization_poses_output_filename);
	}

	if (!ground_truth_poses_output_filename.empty()) {
		ground_truth_poses_output_stream_.open(ground_truth_poses_output_filename.c_str());
		addPoseFileHeader(ground_truth_poses_output_stream_, "# poses from ground truth");
		ROS_INFO_STREAM("Saving ground truth poses associated with the localization poses to file " << ground_truth_poses_output_filename);
	}


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
	pose_error_publisher_ = node_handle->advertise<robot_localization_tools::LocalizationError>(pose_error_publish_topic_name, 10, true);

	std::string localization_poses_publisher_topic;
	private_node_handle->param("localization_poses_publisher_topic", localization_poses_publisher_topic, std::string("localization_poses"));
	localization_poses_publisher_ = node_handle->advertise<geometry_msgs::PoseArray>(localization_poses_publisher_topic, 10, true);

	std::string ground_truth_poses_publisher_topic;
	private_node_handle->param("ground_truth_poses_publisher_topic", ground_truth_poses_publisher_topic, std::string("ground_truth_poses"));
	ground_truth_poses_publisher_ = node_handle->advertise<geometry_msgs::PoseArray>(ground_truth_poses_publisher_topic, 10, true);

	private_node_handle->param("use_degrees_in_angles", use_degrees_in_angles_, false);
	private_node_handle->param("use_millimeters_in_distances", use_millimeters_in_distances_, false);
}


void RobotLocalizationError::processPoseStamped(const geometry_msgs::PoseStampedConstPtr& pose) {
	if (pose->header.stamp < last_update_time_) return;
	geometry_msgs::Pose ground_truth;

	if (getGroundTruthPose(pose->header.stamp, ground_truth)) {
		geometry_msgs::Pose localization_pose = pose->pose;

		savePoseToFile(localization_poses_output_stream_, localization_pose, pose->header.stamp);
		savePoseToFile(ground_truth_poses_output_stream_, ground_truth, pose->header.stamp);

		robot_localization_tools::LocalizationError pose_errors;
		pose_errors.header = pose->header;


		// translation errors
		pose_errors.translation_errors.x = ground_truth.position.x - localization_pose.position.x;
		pose_errors.translation_errors.y = ground_truth.position.y - localization_pose.position.y;
		pose_errors.translation_errors.z = ground_truth.position.z - localization_pose.position.z;

		if (use_millimeters_in_distances_) {
			pose_errors.translation_errors.x *= 1000.0;
			pose_errors.translation_errors.y *= 1000.0;
			pose_errors.translation_errors.z *= 1000.0;
		}

		pose_errors.translation_error = std::sqrt(
				pose_errors.translation_errors.x * pose_errors.translation_errors.x +
				pose_errors.translation_errors.y * pose_errors.translation_errors.y +
				pose_errors.translation_errors.z * pose_errors.translation_errors.z);


		// rotation errors
		tf2::Quaternion pose_localization_q(pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z, pose->pose.orientation.w);
		tf2::Quaternion pose_grund_truth_q(ground_truth.orientation.x, ground_truth.orientation.y, ground_truth.orientation.z, ground_truth.orientation.w);
		tf2::Quaternion rotation_error_q = pose_grund_truth_q * pose_localization_q.inverse();
		rotation_error_q.normalize();
		tf2::Vector3 rotation_error_axis = rotation_error_q.getAxis();
		pose_errors.rotation_error_angle = pose_grund_truth_q.angleShortestPath(pose_localization_q);
//		pose_errors.rotation_error_angle = rotation_error_q.getAngleShortestPath();
		pose_errors.rotation_error_axis.x = rotation_error_axis.getX();
		pose_errors.rotation_error_axis.y = rotation_error_axis.getY();
		pose_errors.rotation_error_axis.z = rotation_error_axis.getZ();

		if (std::abs(rotation_error_q.getAngleShortestPath() - pose_errors.rotation_error_angle) > 0.025) {
			pose_errors.rotation_error_axis.x *= -1;
			pose_errors.rotation_error_axis.y *= -1;
			pose_errors.rotation_error_axis.z *= -1;
		}

		if (use_degrees_in_angles_) {
			pose_errors.rotation_error_angle = angles::to_degrees(pose_errors.rotation_error_angle);
		}


		if (!pose_error_publisher_.getTopic().empty()) pose_error_publisher_.publish(pose_errors);

		if (number_poses_received_since_last_publish_ == pose_publishers_sampling_rate_) {
			localization_poses_.poses.push_back(localization_pose);
			if (!localization_poses_publisher_.getTopic().empty()) {
				localization_poses_.header = pose->header;
				localization_poses_publisher_.publish(localization_poses_);
			}

			ground_truth_poses_.poses.push_back(ground_truth);
			if (!ground_truth_poses_publisher_.getTopic().empty()) {
				ground_truth_poses_.header = pose->header;
				ground_truth_poses_publisher_.publish(ground_truth_poses_);
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


bool RobotLocalizationError::getGroundTruthPose(const ros::Time& time_stamp, geometry_msgs::Pose& ground_truth_pose_out) {
	if (map_ground_truth_frame_id_.empty() && gazebo_link_state_service_.isValid()) {
		gazebo_link_state_service_.call(get_link_state_);
		if (get_link_state_.response.success) {
			ground_truth_pose_out = get_link_state_.response.link_state.pose;
			return true;
		}
	} else {
		if (!odom_frame_id_.empty() && invert_tf_from_map_ground_truth_frame_id_) {
			tf2::Transform localization_tf_odom_to_map;
			if (tf_collector_.lookForTransform(localization_tf_odom_to_map, map_ground_truth_frame_id_, odom_frame_id_, time_stamp)) {
				tf2::Transform localization_tf_base_to_odom;
				if (tf_collector_.lookForTransform(localization_tf_base_to_odom, odom_frame_id_, base_link_frame_id_, time_stamp)) {
					tf2::Transform localization_tf = localization_tf_odom_to_map.inverse() * localization_tf_base_to_odom;
					laserscan_to_pointcloud::tf_rosmsg_eigen_conversions::transformTF2ToMsg(localization_tf, ground_truth_pose_out);
					return true;
				}
			}
		} else {
			tf2::Transform localization_tf;
			if (tf_collector_.lookForTransform(localization_tf, map_ground_truth_frame_id_, base_link_frame_id_, time_stamp)) {
				laserscan_to_pointcloud::tf_rosmsg_eigen_conversions::transformTF2ToMsg(localization_tf, ground_truth_pose_out);
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
		ROS_INFO_STREAM("Computing localization error by regular sampling tf at " << publish_rate_ << " Hz");
		ros::Rate publish_rate(publish_rate_);
		while (ros::ok()) {
			updateLocalizationError();
			publish_rate.sleep();
			ros::spinOnce();
		}
	} else {
		ROS_INFO("Computing localization error from pose topics");
		ros::spin();
	}

	if (localization_poses_output_stream_.is_open()) { localization_poses_output_stream_.close(); }
	if (ground_truth_poses_output_stream_.is_open()) { ground_truth_poses_output_stream_.close(); }
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


bool RobotLocalizationError::savePoseToFile(std::ofstream& output_stream, geometry_msgs::Pose& pose, ros::Time timestamp) {
	if (output_stream.is_open()) {
		if (save_poses_timestamp_) { output_stream << timestamp.sec << "." << timestamp.nsec << " "; }
		output_stream << pose.position.x << " " <<pose.position.y << " " << pose.position.z;
		if (save_poses_orientation_quaternion_) { output_stream << " " << pose.orientation.x << " " << pose.orientation.y << " " << pose.orientation.z << " " << pose.orientation.w; }
		if (save_poses_orientation_vector_) {
			tf2::Vector3 orientation_vector = tf2::quatRotate(tf2::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w).normalize(), tf2::Vector3(1,0,0)).normalize();
			output_stream << " " << orientation_vector.x() << " " << orientation_vector.y() << " " << orientation_vector.z();
		}
		output_stream << std::endl;

		return true;
	}

	return false;
}


bool RobotLocalizationError::addPoseFileHeader(std::ofstream& output_stream, std::string first_line) {
	if (output_stream.is_open()) {
		output_stream << first_line << std::endl;
		output_stream << "#";

		if (save_poses_timestamp_) { output_stream << " timestamp_seconds_posix_time"; }
		output_stream << " translation_x_meters translation_y_meters translation_z_meters";
		if (save_poses_orientation_quaternion_) { output_stream << " orientation_quaternion_x orientation_quaternion_y orientation_quaternion_z orientation_quaternion_w"; }
		if (save_poses_orientation_vector_) { output_stream << " orientation_vector_x orientation_vector_y orientation_vector_z"; }
		output_stream << std::endl;

		return true;
	}

	return false;
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </LocalizationError-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// =============================================================================   <private-section>   =========================================================================
// =============================================================================   </private-section>  =========================================================================

} /* namespace robot_localization_error */
