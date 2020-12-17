///////////////////////////////////////////////////////////////////////////////
//      Title     : camera_pointer.h
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

#pragma once

#include <ros/ros.h>
#include <servo_camera_pointer/PointToPose.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

namespace servo_camera_pointer {
class CameraPointer {
public:
  CameraPointer(ros::NodeHandle &nh, std::string camera_frame,
                std::string z_axis_up_frame, double loop_rate,
                std::string look_pose_server_name);
  ~CameraPointer();

private:
  /** \brief Worker function that actually does the actions
   * @return true if the pose was reached or pointing started
   */
  bool sendPose();

  /* \brief Service callback for starting */
  void startPointingCB(servo_camera_pointer::PointToPose::Request &req,
                       servo_camera_pointer::PointToPose::Response &res);

  /* \brief Service callback for stopping */
  void stopPointingCB(std_srvs::Trigger::Request &req,
                      std_srvs::Trigger::Response &res);

  // Server's to start and end camera pointing
  ros::ServiceServer start_pointing_server_;
  ros::ServiceServer stop_pointing_server_;

  // Server Client to use look at pose
  ros::ServiceClient look_pose_client_;

  // tf listener
  // tf2_ros::TransformListener tf_listener_;

  // node handle
  ros::NodeHandle nh_;

  // frame names for the frame to move and default "Up" frame
  std::string camera_frame_;
  std::string z_axis_up_frame_;
};
} // namespace servo_camera_pointer