///////////////////////////////////////////////////////////////////////////////
//      Title     : camera_pointer.cpp
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

#include <servo_camera_pointer/camera_pointer.h>

#include <stdexcept>

namespace servo_camera_pointer {
CameraPointer::CameraPointer(
    ros::NodeHandle& nh,
    std::unique_ptr<moveit_servo::PoseTracking> pose_tracking_object)
    : nh_(nh), loop_rate_(0), rotational_tolerance_(0.0) {
  // Set up drift_dims_client_ client
  drift_dims_client_ = nh_.serviceClient<moveit_msgs::ChangeDriftDimensions>(
      "change_drift_dimensions");

  // Set up the servers for starting/stopping the camera pointing
  start_pointing_server_ = nh_.advertiseService(
      ros::names::append(nh_.getNamespace(), "start_camera_pointing"),
      &CameraPointer::startPointingCB, this);
  stop_pointing_server_ = nh_.advertiseService(
      ros::names::append(nh_.getNamespace(), "stop_camera_pointing"),
      &CameraPointer::stopPointingCB, this);

  // Read instance specific parameters
  std::string camera_frame_name, z_axis_up_frame, target_frame,
      look_at_pose_server_name, target_pose_publish_topic;
  double loop_rate;

  if (!nh_.getParam("camera_frame_name", camera_frame_name))
    throw std::invalid_argument(
        "Could not load parameter: 'camera_frame_name'");
  if (!nh_.getParam("gravity_frame_name", z_axis_up_frame))
    throw std::invalid_argument(
        "Could not load parameter: 'gravity_frame_name'");
  if (!nh_.getParam("target_frame_name", target_frame))
    throw std::invalid_argument(
        "Could not load parameter: 'target_frame_name'");
  if (!nh_.getParam("loop_rate", loop_rate))
    throw std::invalid_argument("Could not load parameter: 'loop_rate'");
  if (!nh_.getParam("look_at_pose_server_name", look_at_pose_server_name))
    throw std::invalid_argument(
        "Could not load parameter: 'look_at_pose_server_name'");
  if (!nh_.getParam("target_pose_publish_topic", target_pose_publish_topic))
    throw std::invalid_argument(
        "Could not load parameter: 'target_pose_publish_topic'");

  // 0.0 default = tracking doesn't stop until manually told to
  nh_.param<double>("rotational_tolerance", rotational_tolerance_, 0.0);

  // Set up the target pose publisher
  target_pose_publisher_ =
      std::make_unique<servo_camera_pointer::CameraPointerPublisher>(
          nh_, camera_frame_name, z_axis_up_frame, target_frame, loop_rate,
          look_at_pose_server_name, target_pose_publish_topic);

  // Set loop rate
  loop_rate_ = ros::Rate(loop_rate);

  // Steal the Pose Tracking instance
  pose_tracking_ = std::move(pose_tracking_object);
}

bool CameraPointer::startPointingCB(std_srvs::Trigger::Request& req,
                                    std_srvs::Trigger::Response& res) {
  continue_pointing_ = true;
  state_change_handled_ = false;
  return true;
}

bool CameraPointer::stopPointingCB(std_srvs::Trigger::Request& req,
                                   std_srvs::Trigger::Response& res) {
  pose_tracking_->stopMotion();
  continue_pointing_ = false;
  state_change_handled_ = false;
  return true;
}

void CameraPointer::spin() {
  while (ros::ok()) {
    // Check if we need to change states
    if (!state_change_handled_) {
      if (continue_pointing_)
        start();
      else
        stop();
    }

    // If we want to be pointing, do so
    if (continue_pointing_) {
      // Note that this line is blocking, so the stop CB needs to handle
      // canceling this function by calling stopMotion() If the camera is
      // already aligned this will end almost immediately and the while() here
      // will run quickly
      pose_tracking_->moveToPose(linear_tolerance_, rotational_tolerance_,
                                 0.1 /* target pose timeout */);
    }

    loop_rate_.sleep();
  }
}

bool CameraPointer::start() {
  // Make all linear dimensions drift
  moveit_msgs::ChangeDriftDimensions drift_serv;
  drift_serv.request.drift_x_translation = true;
  drift_serv.request.drift_y_translation = true;
  drift_serv.request.drift_z_translation = true;
  drift_serv.request.drift_x_rotation = false;
  drift_serv.request.drift_y_rotation = false;
  drift_serv.request.drift_z_rotation = false;

  if (!drift_dims_client_.call(drift_serv)) {
    ROS_ERROR_STREAM("NO RESPONSE FROM: drift dimensions server");
    return false;
  }

  pose_tracking_->resetTargetPose();

  // Start the publisher
  target_pose_publisher_->start();

  state_change_handled_ = true;
  return true;
}

bool CameraPointer::stop() {
  // Reset all drift dimensions
  moveit_msgs::ChangeDriftDimensions drift_serv;
  drift_serv.request.drift_x_translation = false;
  drift_serv.request.drift_y_translation = false;
  drift_serv.request.drift_z_translation = false;
  drift_serv.request.drift_x_rotation = false;
  drift_serv.request.drift_y_rotation = false;
  drift_serv.request.drift_z_rotation = false;

  if (!drift_dims_client_.call(drift_serv)) {
    ROS_ERROR_STREAM("NO RESPONSE FROM: drift dimensions server");
    return false;
  }

  // Stop publishing
  pose_tracking_->stopMotion();
  target_pose_publisher_->stop();

  state_change_handled_ = true;
  return true;
}
}  // namespace servo_camera_pointer
