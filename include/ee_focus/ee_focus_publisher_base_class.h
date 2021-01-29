#pragma once

#include <look_at_pose/LookAtPose.h>
#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <thread>

namespace ee_focus {

class EEFPublisherBase {
 public:
  /* \brief Pluginlib requires an intializer instead of constructor  */
  void initialize(ros::NodeHandle& nh,
                  std::string ee_frame,
                  std::string z_axis_up_frame,
                  std::string target_frame,
                  double loop_rate,
                  std::string look_pose_server_name,
                  std::string publish_topic_name);

  /* \brief Starts the publisher indefinitely */
  virtual void start();

  /* \brief Stops the publisher */
  virtual void stop();

  virtual ~EEFPublisherBase();

 protected:
  /* \brief Pluginlib requires empty base class constructor  */
  EEFPublisherBase();

 private:
  /* \brief Where generic publishing infrastructure is implemented  */
  void mainPubLoop();

  /* \brief Where all the work to calculate the desired pose is done  */
  virtual look_at_pose::LookAtPose poseCalulation(
      const geometry_msgs::TransformStamped ee_to_gravity_tf,
      const geometry_msgs::TransformStamped ee_to_target_tf) const = 0;

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

};  // namespace ee_focus
