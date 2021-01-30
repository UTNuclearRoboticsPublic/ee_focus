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
                  std::string publish_topic_name) {
    nh_ = nh;  // TODO does this do what I want it to do?
    ee_frame_ = ee_frame;
    z_axis_up_frame_ = z_axis_up_frame;
    target_frame_ = target_frame;
    loop_rate_ = loop_rate;

    return;
  }

  /* \brief Starts the publisher indefinitely */
  virtual void start() {
    continue_publishing_ = true;

    thread_ = std::thread([this] { mainPubLoop(); });

    return;
  }

  /* \brief Stops the publisher */
  virtual void stop() {
    continue_publishing_ = false;

    // Too many horror stories of problems caused by leaving out brackets,
    // sorry :)
    if (thread_.joinable()) {
      thread_.join();
    }

    return;
  }

  /* \brief Where all the work to calculate the desired pose is done  */
  virtual const bool poseCalulation(
      geometry_msgs::PoseStamped& target_pose) = 0;

  // TODO we have them getters, but do we need to give them setters too or no?
  ros::ServiceClient getLookPoseClient() { return look_pose_client_; }
  tf2_ros::Buffer& getTFBuffer() { return tf_buffer_; }
  std::string getEEFrame() { return ee_frame_; }
  std::string getZAxisUpFrame() { return z_axis_up_frame_; }
  std::string getTargetFrame() { return target_frame_; }

  virtual ~EEFPublisherBase() { stop(); }

 protected:
  /* \brief Pluginlib requires empty base class constructor  */
  EEFPublisherBase() : tf_buffer_(), tf_listener_(tf_buffer_), loop_rate_(10) {}

 private:
  /* \brief Where generic publishing infrastructure is implemented  */
  void mainPubLoop() {
    geometry_msgs::PoseStamped target_pose;
    // Keep going until ROS dies or stop requested
    while (ros::ok() && continue_publishing_) {
      // Do the bulk of the calculation
      if (poseCalulation(target_pose)) {
        target_pose.header.stamp = ros::Time::now();
        target_pose_pub_.publish(target_pose);
      }

      // Sleep at loop rate
      loop_rate_.sleep();
    }
    return;
  }

  // Server Client to use look at pose
  ros::ServiceClient look_pose_client_;  // TODO move to superclass
  // Publisher to send poses to Servo Pose Tracking
  ros::Publisher target_pose_pub_;
  // node handle
  ros::NodeHandle nh_;
  // tf listener
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  // frame names for the frame to move and default "Up" frame
  std::string ee_frame_;         // TODO move to superclass
  std::string z_axis_up_frame_;  // TODO move to superclass
  std::string target_frame_;     // TODO move to superclass
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
