///////////////////////////////////////////////////////////////////////////////
//      Title     : ee_focus_publisher.h
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

#pragma once

#include <look_at_pose/LookAtPose.h>
#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <thread>

namespace ee_focus {
class EEFocusPublisher {
 public:
  EEFocusPublisher(ros::NodeHandle& nh,
                   std::string ee_frame,
                   std::string z_axis_up_frame,
                   std::string target_frame,
                   double loop_rate,
                   std::string look_pose_server_name,
                   std::string publish_topic_name);
  ~EEFocusPublisher();

  /* \brief Starts the publisher indefinitely */
  void start();

  /* \brief Stops the publisher */
  void stop();

 private:
  /* \brief Where most of the publishing work actually happens */
  void mainPubLoop();

  // Server Client to use look at pose
  ros::ServiceClient look_pose_client_;

  // Publisher to send poses to Servo Pose Tracking
  ros::Publisher target_pose_pub_;

  // node handle
  ros::NodeHandle nh_;

  // tf listener
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // frame names for the frame to move and default "Up" frame
  std::string ee_frame_;
  std::string z_axis_up_frame_;
  std::string target_frame_;

  // loop rate
  ros::Rate loop_rate_;

  // Only continue publishing while this is true. Another thread can set this to
  // false and stop publishing
  std::atomic<bool> continue_publishing_{false};

  // Hold the thread we will run the main loop in - to avoid blocking whatever
  // calls start()
  std::thread thread_;
};
}  // namespace ee_focus
