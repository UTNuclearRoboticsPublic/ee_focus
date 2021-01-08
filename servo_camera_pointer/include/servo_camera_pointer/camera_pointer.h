///////////////////////////////////////////////////////////////////////////////
//      Title     : camera_pointer.h
//      Project   : servo_camera_pointer
//      Created   : 01/07/2020
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

#pragma once

#include <moveit_servo/pose_tracking.h>
#include <moveit_servo/servo.h>
#include <ros/ros.h>
#include <servo_camera_pointer/camera_pointer_publisher.h>
#include <std_srvs/Trigger.h>

#include <atomic>

namespace servo_camera_pointer {
class CameraPointer {
 public:
  CameraPointer(
      ros::NodeHandle& nh,
      std::unique_ptr<moveit_servo::PoseTracking> pose_tracking_object);
  ~CameraPointer(){};

  /* \brief Main spinning loop for this class */
  void spin();

 private:
  /* \brief Does all the setup when we want to start pointing */
  bool start();

  /* \brief Does all shutdown when we want to stop pointing */
  bool stop();

  /* \brief Service callback for starting */
  bool startPointingCB(std_srvs::Trigger::Request& req,
                       std_srvs::Trigger::Response& res);

  /* \brief Service callback for stopping */
  bool stopPointingCB(std_srvs::Trigger::Request& req,
                      std_srvs::Trigger::Response& res);

  // Server's to start and end camera pointing
  ros::ServiceServer start_pointing_server_;
  ros::ServiceServer stop_pointing_server_;

  // A client for changing the drift dimensions in Servo
  ros::ServiceClient drift_dims_client_;

  // node handle
  ros::NodeHandle nh_;

  // loop rate
  ros::Rate loop_rate_;

  // Only continue publishing while this is true. Another thread can set this to
  // false and stop publishing
  std::atomic<bool> continue_pointing_{false};

  // This atomic tracks our start/stop requests
  std::atomic<bool> state_change_handled_{true};

  // Hold the pose tracking object here for using
  std::unique_ptr<moveit_servo::PoseTracking> pose_tracking_;

  // Also hold the target pose publisher
  std::unique_ptr<servo_camera_pointer::CameraPointerPublisher>
      target_pose_publisher_;

  // Tolerances for when a move is "complete"
  double rotational_tolerance_;
  Eigen::Vector3d linear_tolerance_{1, 1, 1};
};
}  // namespace servo_camera_pointer
