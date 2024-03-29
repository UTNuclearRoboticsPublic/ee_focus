///////////////////////////////////////////////////////////////////////////////
//      Title     : ee_focus.cpp
//      Project   : ee_focus
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

#include <ee_focus/ee_focus.h>
#include <ee_focus/ee_focus_publisher_base_class.h>
#include <pluginlib/class_loader.h>

#include <stdexcept>

namespace ee_focus {
EEFocus::EEFocus(
    ros::NodeHandle& nh,
    std::unique_ptr<moveit_servo::PoseTracking> pose_tracking_object)
    : nh_(nh), loop_rate_(1.0), rotational_tolerance_(0.0) {
  // Set up drift_dims_client_ client
  drift_dims_client_ = nh_.serviceClient<moveit_msgs::ChangeDriftDimensions>(
      "change_drift_dimensions");

  // Set up the servers for starting/stopping the EE focusing
  start_pointing_server_ = nh_.advertiseService(
      ros::names::append(nh_.getNamespace(), "start_ee_focus"),
      &EEFocus::startPointingCB,
      this);
  stop_pointing_server_ = nh_.advertiseService(
      ros::names::append(nh_.getNamespace(), "stop_ee_focus"),
      &EEFocus::stopPointingCB,
      this);
  change_target_frame_server_ = nh_.advertiseService(
      ros::names::append(nh_.getNamespace(), "set_target_frame"),
      &EEFocus::setTargetFrameCB,
      this);

  // Read instance specific parameters
  std::string target_pose_publish_topic;
  double loop_rate;

  std::string plugin_name;
  if (!nh_.getParam("plugin_name", plugin_name)) {
    throw std::invalid_argument("Could not load parameter: 'plugin_name'");
  }
  if (!nh_.getParam("ee_frame_name", ee_frame_)) {
    throw std::invalid_argument("Could not load parameter: 'ee_frame_name'");
  }
  if (!nh_.getParam("target_frame_name", target_frame_)) {
    throw std::invalid_argument(
        "Could not load parameter: 'target_frame_name'");
  }

  nh_.param<double>("loop_rate", loop_rate, 50.0);
  nh_.param<std::string>(
      "target_pose_publish_topic", target_pose_publish_topic, "target_pose");

  // 0.0 default = tracking doesn't stop until manually told to
  nh_.param<double>("rotational_tolerance", rotational_tolerance_, 0.0);

  // Set up the target pose publisher
  pluginlib::ClassLoader<ee_focus::EEFPublisherBase> ee_focus_loader(
      "ee_focus", "ee_focus::EEFPublisherBase");

  try {
    target_pose_publisher_ = ee_focus_loader.createInstance(plugin_name);
    target_pose_publisher_->initialize(
        nh_, loop_rate, target_pose_publish_topic);
    target_pose_publisher_->setTargetFrame(target_frame_);
  } catch (pluginlib::PluginlibException& ex) {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s",
              ex.what());
  }

  // Set loop rate
  loop_rate_ = ros::Rate(loop_rate);

  // Steal the Pose Tracking instance
  pose_tracking_ = std::move(pose_tracking_object);
}

bool EEFocus::startPointingCB(std_srvs::Trigger::Request& req,
                              std_srvs::Trigger::Response& res) {
  continue_pointing_ = true;
  state_change_handled_ = false;
  res.success = true;
  res.message = std::string("Starting to focus EE '") + ee_frame_ +
                std::string("' at frame '") + target_frame_ + std::string("'.");
  return true;
}

bool EEFocus::stopPointingCB(std_srvs::Trigger::Request& req,
                             std_srvs::Trigger::Response& res) {
  pose_tracking_->stopMotion();
  continue_pointing_ = false;
  state_change_handled_ = false;
  res.success = true;
  res.message = "Stopping EE focusing";
  return true;
}

bool EEFocus::setTargetFrameCB(ee_focus::SetTargetFrame::Request& req,
                               ee_focus::SetTargetFrame::Response& res) {
  target_frame_ = req.target_frame;
  target_pose_publisher_->setTargetFrame(target_frame_);
  res.success = true;
  res.message = std::string("Set target frame to: ") + target_frame_;
  return true;
}

void EEFocus::spin() {
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
      // canceling this function by calling stopMotion() If the EE is
      // already aligned this will end almost immediately and the while() here
      // will run quickly
      pose_tracking_->moveToPose(linear_tolerance_,
                                 rotational_tolerance_,
                                 0.1 /* target pose timeout */);
    }

    loop_rate_.sleep();
  }

  pose_tracking_->stopMotion();
  target_pose_publisher_->stop();
  return;
}

bool EEFocus::start() {
  std::vector<bool> drift_dimensions;
  if (!nh_.getParam("drift_dimensions", drift_dimensions)) {
    throw std::invalid_argument("Could not load parameter: 'drift_dimensions'");
  }

  moveit_msgs::ChangeDriftDimensions drift_serv;
  drift_serv.request.drift_x_translation = drift_dimensions[0];
  drift_serv.request.drift_y_translation = drift_dimensions[1];
  drift_serv.request.drift_z_translation = drift_dimensions[2];
  drift_serv.request.drift_x_rotation = drift_dimensions[3];
  drift_serv.request.drift_y_rotation = drift_dimensions[4];
  drift_serv.request.drift_z_rotation = drift_dimensions[5];

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

bool EEFocus::stop() {
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
}  // namespace ee_focus
