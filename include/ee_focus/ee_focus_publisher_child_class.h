#pragma once

#include <ee_focus/ee_focus_publisher_base_class.h>
#include <look_at_pose/LookAtPose.h>

// TODO does posecalc need to be public so we can inherit it easily?

namespace ee_focus {

class UnconstrainedCameraPointer : public EEFPublisherBase {
 public:
  UnconstrainedCameraPointer() {}

  void initialize_child() {
    // Load child-specific parameters
    if (!nh_.getParam("ee_frame_name", ee_frame_)) {
      throw std::invalid_argument("Could not load parameter: 'ee_frame_name'");
    }
    if (!nh_.getParam("gravity_frame_name", z_axis_up_frame_)) {
      throw std::invalid_argument(
          "Could not load parameter: 'gravity_frame_name'");
    }
    if (!nh_.getParam("target_frame_name", target_frame_)) {
      throw std::invalid_argument(
          "Could not load parameter: 'target_frame_name'");
    }

    // Set up look_at_pose service client
    std::string look_at_pose_server_name;
    nh_.param<std::string>(
        "look_at_pose_server_name", look_at_pose_server_name, "/look_at_pose");
    look_pose_client_ =
        nh_.serviceClient<look_at_pose::LookAtPose>(look_at_pose_server_name);

    // The initial EE pose is always identity in the EE frame
    init_ee_pose_.header.frame_id = ee_frame_;
    init_ee_pose_.pose.orientation.w = 1;
  }

  /* \brief Where all the work to calculate unconstrained camera pose happens */
  const bool poseCalculation(geometry_msgs::PoseStamped& target_pose) {
    // Look up the current transforms
    try {
      ee_to_gravity_tf_ = getTFBuffer().lookupTransform(
          ee_frame_, z_axis_up_frame_, ros::Time(0), ros::Duration(1));
      ee_to_target_tf_ = getTFBuffer().lookupTransform(
          ee_frame_, target_frame_, ros::Time(0), ros::Duration(1));
    } catch (tf2::TransformException& ex) {
      ROS_ERROR_THROTTLE(1, "%s", ex.what());
      return false;
    }

    // Convert the transform for the gravity frame to a rotation matrix
    q_gravity_.w() = ee_to_gravity_tf_.transform.rotation.w;
    q_gravity_.x() = ee_to_gravity_tf_.transform.rotation.x;
    q_gravity_.y() = ee_to_gravity_tf_.transform.rotation.y;
    q_gravity_.z() = ee_to_gravity_tf_.transform.rotation.z;
    R_gravity_ = q_gravity_.normalized().toRotationMatrix();

    // Populate gravity vector as the z-axis of rotation matrix
    gravity_.header.frame_id = ee_to_gravity_tf_.header.frame_id;
    gravity_.vector.x = R_gravity_(0, 2);
    gravity_.vector.y = R_gravity_(1, 2);
    gravity_.vector.z = R_gravity_(2, 2);

    // Set the target pose in the EE frame using the found transformation
    target_look_pose_.header.frame_id = ee_to_target_tf_.header.frame_id;
    target_look_pose_.header.stamp = ee_to_target_tf_.header.stamp;
    target_look_pose_.pose.position.x =
        ee_to_target_tf_.transform.translation.x;
    target_look_pose_.pose.position.y =
        ee_to_target_tf_.transform.translation.y;
    target_look_pose_.pose.position.z =
        ee_to_target_tf_.transform.translation.z;
    target_look_pose_.pose.orientation = ee_to_target_tf_.transform.rotation;

    // Populate the look_at_pose service request
    look_at_pose_service_.request.target_pose = target_look_pose_;
    look_at_pose_service_.request.up = gravity_;

    // We need to update the time for the initial EE pose (identity)
    init_ee_pose_.header.stamp = ros::Time::now();
    // Populate the rest of the look_at_pose service request
    look_at_pose_service_.request.initial_cam_pose = init_ee_pose_;

    // Call the look_at_pose service
    if (!look_pose_client_.call(look_at_pose_service_)) {
      ROS_ERROR_STREAM_THROTTLE(1, "NO RESPONSE FROM: look_at_pose");
      return false;
    }

    target_pose = look_at_pose_service_.response.new_cam_pose;

    return true;
  }

 protected:
  // Server Client to use look at pose
  ros::ServiceClient look_pose_client_;

  // frame names for the frame to move and default "Up" frame
  std::string ee_frame_;
  std::string z_axis_up_frame_;
  std::string target_frame_;

  // Some objects used in the main loop
  // Declare here to avoid constant allocation while running
  look_at_pose::LookAtPose look_at_pose_service_;
  geometry_msgs::PoseStamped init_ee_pose_;
  geometry_msgs::TransformStamped ee_to_gravity_tf_;
  geometry_msgs::TransformStamped ee_to_target_tf_;
  geometry_msgs::Vector3Stamped gravity_;
  geometry_msgs::PoseStamped target_look_pose_;
  Eigen::Quaterniond q_gravity_;
  Eigen::Matrix3d R_gravity_;
};

}  // namespace ee_focus
