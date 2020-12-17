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
#include <look_at_pose/LookAtPose.h>
#include <std_msgs/Int8.h>

#include <moveit_servo/make_shared_from_pool.h>
#include <moveit_servo/pose_tracking.h>
#include <moveit_servo/servo.h>
#include <moveit_servo/status_codes.h>
#include <thread>

static const std::string LOGNAME = "cpp_interface_example";

// Class for monitoring status of moveit_servo
class StatusMonitor {
public:
  StatusMonitor(ros::NodeHandle &nh, const std::string &topic) {
    sub_ = nh.subscribe(topic, 1, &StatusMonitor::statusCB, this);
  }

private:
  void statusCB(const std_msgs::Int8ConstPtr &msg) {
    moveit_servo::StatusCode latest_status =
        static_cast<moveit_servo::StatusCode>(msg->data);
    if (latest_status != status_) {
      status_ = latest_status;
      const auto &status_str = moveit_servo::SERVO_STATUS_CODE_MAP.at(status_);
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
int main(int argc, char **argv) {
  ros::init(argc, argv, LOGNAME);
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(8);
  spinner.start();

  // Load the planning scene monitor
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  planning_scene_monitor =
      std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
          "robot_description");
  if (!planning_scene_monitor->getPlanningScene()) {
    ROS_ERROR_STREAM_NAMED(LOGNAME,
                           "Error in setting up the PlanningSceneMonitor.");
    exit(EXIT_FAILURE);
  }

  planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->startWorldGeometryMonitor(
      planning_scene_monitor::PlanningSceneMonitor::
          DEFAULT_COLLISION_OBJECT_TOPIC,
      planning_scene_monitor::PlanningSceneMonitor::
          DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
      false /* skip octomap monitor */);
  planning_scene_monitor->startStateMonitor();

  // Create the pose tracker
  moveit_servo::PoseTracking tracker(nh, planning_scene_monitor);

  // Make a publisher for sending pose commands
  ros::Publisher target_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(
      "target_pose", 1 /* queue */, true /* latch */);

  // Subscribe to servo status (and log it when it changes)
  StatusMonitor status_monitor(nh, "status");

  Eigen::Vector3d lin_tol{0.01, 0.01, 0.01};
  double rot_tol = 0.1;

  // Get the current EE transform
  geometry_msgs::TransformStamped current_ee_tf;
  tracker.getCommandFrameTransform(current_ee_tf);

  // Convert it to a Pose
  geometry_msgs::PoseStamped target_pose;
  target_pose.header.frame_id = current_ee_tf.header.frame_id;
  target_pose.pose.position.x = current_ee_tf.transform.translation.x;
  target_pose.pose.position.y = current_ee_tf.transform.translation.y;
  target_pose.pose.position.z = current_ee_tf.transform.translation.z;
  target_pose.pose.orientation = current_ee_tf.transform.rotation;

  // Modify it a little bit
  target_pose.pose.position.x += 0.1;

  ros::Rate loop_rate(50);
  for (size_t i = 0; i < 500; ++i) {
    // Modify the pose target a little bit each cycle
    // This is a dynamic pose target
    // target_pose.pose.position.z += 0.0004;
    // target_pose.header.stamp = ros::Time::now();
    // target_pose_pub.publish(target_pose);

    loop_rate.sleep();
  }

  // resetTargetPose() can be used to clear the target pose and wait for a new
  // one, e.g. when moving between multiple waypoints
  tracker.resetTargetPose();

  // Publish target pose
  target_pose.header.stamp = ros::Time::now();
  // target_pose_pub.publish(target_pose);

  // Run the pose tracking in a new thread
  // std::thread move_to_pose_thread([&tracker, &lin_tol, &rot_tol] {
    // tracker.moveToPose(lin_tol, rot_tol, 0.1 /* target pose timeout */);
  // });

  // for (size_t i = 0; i < 500; ++i) {
  //   // Modify the pose target a little bit each cycle
  //   // This is a dynamic pose target
  //   // target_pose.pose.position.z += 0.0004;
  //   // target_pose.header.stamp = ros::Time::now();
  //   // target_pose_pub.publish(target_pose);

  //   loop_rate.sleep();
  // }

  // Make sure the tracker is stopped and clean up
  tracker.stopMotion();
  // move_to_pose_thread.join();

  std::string gravity = "base_footprint";
  std::string camera_link = "camera_left_link";
  std::string target_frame = "r_temoto_end_effector";

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped cam_gravity_tf;
  try{
    cam_gravity_tf = tfBuffer.lookupTransform(camera_link, gravity, ros::Time(0), ros::Duration(1));
    cam_gravity_tf.child_frame_id = "TEST_FRAME";
    cam_gravity_tf.transform.translation.x = 0;
    cam_gravity_tf.transform.translation.y = 0;
    cam_gravity_tf.transform.translation.z = 0;
    ROS_ERROR_STREAM(cam_gravity_tf);
  }
  catch (tf2::TransformException &ex){
      ROS_ERROR("%s",ex.what());
  }

  ros::ServiceClient client =
      nh.serviceClient<look_at_pose::LookAtPose>("/look_at_pose");
  geometry_msgs::PoseStamped init_cam_pose;
  init_cam_pose.header.frame_id = camera_link;
  init_cam_pose.header.stamp = ros::Time::now();
  init_cam_pose.pose.orientation.w = 1;

  geometry_msgs::PoseStamped target_look_pose;
  target_look_pose.header.frame_id = camera_link;
  target_look_pose.header.stamp = ros::Time::now();
  target_look_pose.pose.position.x = 1;
  target_look_pose.pose.position.y = 1;
  target_look_pose.pose.orientation.w = 1;

  geometry_msgs::Vector3Stamped up;
  up.vector.z = 1;
  up.header.frame_id = camera_link;

  look_at_pose::LookAtPose serv_msg;
  serv_msg.request.initial_cam_pose = init_cam_pose;
  serv_msg.request.target_pose = target_look_pose;
  serv_msg.request.up = up;

  client.waitForExistence();
  if (client.call(serv_msg)) {
    ROS_ERROR_STREAM(serv_msg.response);
  } else {
    ROS_ERROR_STREAM("NO RESPONSE");
  }

  while (ros::ok()) {
    br.sendTransform(cam_gravity_tf);
    loop_rate.sleep();
  }

  return EXIT_SUCCESS;
}
