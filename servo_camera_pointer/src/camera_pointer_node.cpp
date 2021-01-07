///////////////////////////////////////////////////////////////////////////////
//      Title     : camera_pointer_node.cpp
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

// int main(int argc, char **argv) {
//   servo_camera_pointer::CameraPointer camera_pointer();
//   return 0;
// }

#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Int8.h>

#include <moveit_servo/make_shared_from_pool.h>
#include <moveit_servo/pose_tracking.h>
#include <moveit_servo/servo.h>
#include <moveit_servo/status_codes.h>
#include <thread>

static const std::string LOGNAME = "camera_pointing";

// Class for monitoring status of moveit_servo
class StatusMonitor
{
public:
  StatusMonitor(ros::NodeHandle& nh, const std::string& topic)
  {
    sub_ = nh.subscribe(topic, 1, &StatusMonitor::statusCB, this);
  }

private:
  void statusCB(const std_msgs::Int8ConstPtr& msg)
  {
    moveit_servo::StatusCode latest_status = static_cast<moveit_servo::StatusCode>(msg->data);
    if (latest_status != status_)
    {
      status_ = latest_status;
      const auto& status_str = moveit_servo::SERVO_STATUS_CODE_MAP.at(status_);
      ROS_INFO_STREAM_NAMED(LOGNAME, "Servo status: " << status_str);
    }
  }
  moveit_servo::StatusCode status_ = moveit_servo::StatusCode::INVALID;
  ros::Subscriber sub_;
};

/**
 * Instantiate the pose tracking interface.
 * Send a pose slightly different from the starting pose
 * Then keep updating the target pose a little bit
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, LOGNAME);
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(8);
  spinner.start();

  // Load the planning scene monitor
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
  if (!planning_scene_monitor->getPlanningScene())
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Error in setting up the PlanningSceneMonitor.");
    exit(EXIT_FAILURE);
  }

  planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->startWorldGeometryMonitor(
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
      false /* skip octomap monitor */);
  planning_scene_monitor->startStateMonitor();

  // Create the pose tracker
  moveit_servo::PoseTracking tracker(nh, planning_scene_monitor);

  // Subscribe to servo status (and log it when it changes)
  StatusMonitor status_monitor(nh, "status");

  // Tolerances for pose tracking - don't care about linear positions
  Eigen::Vector3d lin_tol{ 1, 1, 1 };
  double rot_tol = 0.1;

  // Hold on to a bunch of frame names etc
  std::string gravity_frame = "base_footprint";
  std::string camera_link = "camera_left_link";
  std::string target_frame = "r_temoto_end_effector";
  std::string look_pose_server_name = "/look_at_pose";
  std::string pose_publisher_topic = "target_pose";

  // Create pose publisher
  servo_camera_pointer::CameraPointerPublisher pose_publisher(nh, camera_link, gravity_frame, target_frame, 50,
                                                              look_pose_server_name, pose_publisher_topic);

  // Set up the drift dimension client for Servo
  ros::ServiceClient drift_client = nh.serviceClient<moveit_msgs::ChangeDriftDimensions>("change_drift_dimensions");
  drift_client.waitForExistence();

  // Make all linear dimensions drift
  moveit_msgs::ChangeDriftDimensions drift_serv;
  drift_serv.request.drift_x_translation = true;
  drift_serv.request.drift_y_translation = true;
  drift_serv.request.drift_z_translation = true;
  drift_serv.request.drift_x_rotation = false;
  drift_serv.request.drift_y_rotation = false;
  drift_serv.request.drift_z_rotation = false;

  if (!drift_client.call(drift_serv))
  {
    ROS_ERROR_STREAM("NO RESPONSE FROM: drift server");
  }

  // resetTargetPose() can be used to clear the target pose and wait for a new
  // one, e.g. when moving between multiple waypoints
  tracker.resetTargetPose();

  // Start the publisher in a new thread
  std::thread publish_target_thread([&pose_publisher] { pose_publisher.start(); });

  // Call Servo to move, this blocks until returning
  moveit_servo::PoseTrackingStatusCode result = tracker.moveToPose(lin_tol, rot_tol, 0.1 /* target pose timeout */);

  // Now stop the tracker and publisher, and clean up
  tracker.stopMotion();
  pose_publisher.stop();
  publish_target_thread.join();

  return EXIT_SUCCESS;
}
