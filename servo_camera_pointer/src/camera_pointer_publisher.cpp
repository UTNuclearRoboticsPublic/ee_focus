///////////////////////////////////////////////////////////////////////////////
//      Title     : camera_pointer_publisher.cpp
//      Project   : servo_camera_pointer
//      Created   : 12/15/2020
//      Author    : Adam Pettinger
//      Copyright : Copyright© The University of Texas at Austin, 2014-2021. All
//      rights reserved.
//
//          All files within this directory are subject to the following, unless
//          an alternative license is explicitly included within the text of
//          each file.
//
//          This software and documentation constitute an unpublished work
//          and contain valuable trade secrets and proprietary information
//          belonging to the University. None of the foregoing material may be
//          copied or duplicated or disclosed without the express, written
//          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
//          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
//          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
//          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
//          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
//          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
//          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
//          University be liable for incidental, special, indirect, direct or
//          consequential damages or loss of profits, interruption of business,
//          or related expenses which may arise from use of software or
//          documentation, including but not limited to those resulting from
//          defects in software and/or documentation, or loss or inaccuracy of
//          data of any kind.
//
///////////////////////////////////////////////////////////////////////////////

#include <servo_camera_pointer/camera_pointer_publisher.h>

namespace servo_camera_pointer {
CameraPointerPublisher::CameraPointerPublisher(
    ros::NodeHandle& nh, std::string camera_frame, std::string z_axis_up_frame,
    std::string target_frame, double loop_rate,
    std::string look_pose_server_name, std::string publish_topic_name)
    : nh_(nh),
      tf_buffer_(),
      tf_listener_(tf_buffer_),
      camera_frame_(camera_frame),
      z_axis_up_frame_(z_axis_up_frame),
      target_frame_(target_frame),
      loop_rate_(loop_rate) {
  // Set up look_at_pose client
  look_pose_client_ =
      nh_.serviceClient<look_at_pose::LookAtPose>(look_pose_server_name);

  // Set up the publisher. This publishes to Servo Pose Tracking
  target_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
      publish_topic_name, 1 /* queue */, true /* latch */);
}

void CameraPointerPublisher::stop() {
  continue_publishing_ = false;

  if (thread_.joinable()) thread_.join();
}

void CameraPointerPublisher::start() {
  continue_publishing_ = true;

  thread_ = std::thread([this] { mainPubLoop(); });
}

void CameraPointerPublisher::mainPubLoop() {
  geometry_msgs::TransformStamped cam_to_gravity_tf, cam_to_target_tf;
  geometry_msgs::Vector3Stamped gravity;
  geometry_msgs::PoseStamped init_cam_pose;
  geometry_msgs::PoseStamped target_look_pose;
  look_at_pose::LookAtPose look_at_pose_service;

  // The initial camera pose is always identity in the camera frame
  init_cam_pose.header.frame_id = camera_frame_;
  init_cam_pose.pose.orientation.w = 1;

  // Keep going until ROS dies or stop requested
  while (ros::ok() && continue_publishing_) {
    // Look up the current transforms
    try {
      cam_to_gravity_tf = tf_buffer_.lookupTransform(
          camera_frame_, z_axis_up_frame_, ros::Time(0), ros::Duration(1));
      cam_to_target_tf = tf_buffer_.lookupTransform(
          camera_frame_, target_frame_, ros::Time(0), ros::Duration(1));
    } catch (tf2::TransformException& ex) {
      ROS_ERROR_THROTTLE(1, "%s", ex.what());
      loop_rate_.sleep();
      continue;
    }

    // Convert the transform for the gravity frame to a rotation matrix
    Eigen::Quaterniond q_gravity(cam_to_gravity_tf.transform.rotation.w,
                                 cam_to_gravity_tf.transform.rotation.x,
                                 cam_to_gravity_tf.transform.rotation.y,
                                 cam_to_gravity_tf.transform.rotation.z);
    Eigen::Matrix3d R_gravity = q_gravity.normalized().toRotationMatrix();

    // Populate gravity vector as the z-axis of rotation matrix
    gravity.header.frame_id = cam_to_gravity_tf.header.frame_id;
    gravity.vector.x = R_gravity(0, 2);
    gravity.vector.y = R_gravity(1, 2);
    gravity.vector.z = R_gravity(2, 2);

    // We need to update the time for the initial camera pose (identity)
    init_cam_pose.header.stamp = ros::Time::now();

    // Set the target pose in the camera frame using the found transformation
    target_look_pose.header.frame_id = cam_to_target_tf.header.frame_id;
    target_look_pose.header.stamp = cam_to_target_tf.header.stamp;
    target_look_pose.pose.position.x = cam_to_target_tf.transform.translation.x;
    target_look_pose.pose.position.y = cam_to_target_tf.transform.translation.y;
    target_look_pose.pose.position.z = cam_to_target_tf.transform.translation.z;
    target_look_pose.pose.orientation = cam_to_target_tf.transform.rotation;

    // Populate the look_at_pose service request
    look_at_pose_service.request.initial_cam_pose = init_cam_pose;
    look_at_pose_service.request.target_pose = target_look_pose;
    look_at_pose_service.request.up = gravity;

    // Call the look_at_pose service
    if (!look_pose_client_.call(look_at_pose_service)) {
      ROS_ERROR_STREAM_THROTTLE(1, "NO RESPONSE FROM: look_at_pose");
      loop_rate_.sleep();
      continue;
    }

    // Publish the pose
    look_at_pose_service.response.new_cam_pose.header.stamp = ros::Time::now();
    target_pose_pub_.publish(look_at_pose_service.response.new_cam_pose);

    // Sleep at loop rate
    loop_rate_.sleep();
  }
  return;
}
}  // namespace servo_camera_pointer
