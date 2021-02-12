#pragma once

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
                  double loop_rate,
                  std::string publish_topic_name) {
    nh_ = ros::NodeHandle(nh);
    loop_rate_ = ros::Rate(loop_rate);
    target_pose_pub_ =
        nh_.advertise<geometry_msgs::PoseStamped>(publish_topic_name, 1, true);

    // Load general parent parameters
    if (!nh_.getParam("ee_frame_name", ee_frame_)) {
      throw std::invalid_argument("Could not load parameter: 'ee_frame_name'"); // TODO should we use ROS_ERROR instead?
    }
    initialize_child();
    return;
  }

  /* \brief Starts the publisher indefinitely */
  void start() {
    start_child();

    continue_publishing_ = true;

    thread_ = std::thread([this] { mainPubLoop(); });

    return;
  }

  /* \brief Stops the publisher */
  void stop() {
    stop_child();

    continue_publishing_ = false;

    if (thread_.joinable()) {
      thread_.join();
    }

    return;
  }

  /* \brief Where all the work to calculate the desired pose is done  */
  virtual const bool poseCalculation(
      geometry_msgs::PoseStamped& target_pose) = 0;

  tf2_ros::Buffer& getTFBuffer() { return tf_buffer_; }

  virtual ~EEFPublisherBase() { stop(); }

 protected:
  /* \brief Pluginlib requires empty base class constructor  */
  EEFPublisherBase() : tf_buffer_(), tf_listener_(tf_buffer_), loop_rate_(10) {}

  /**
   * \brief Initializes the child class
   * Any child-specific behavoir should be implemented there, in a function of
   * the same name. This automatically gets called on plugin initialize()
   */
  virtual void initialize_child(){};

  /**
   * \brief Special starting behavoir for child class
   * Any child-specific behavoir should be implemented there, in a function of
   * the same name. This automatically gets called at the beginning of focus
   * start()
   */
  virtual void start_child(){};

  /**
   * \brief Special stopping behavoir for child class
   * Any child-specific behavoir should be implemented there, in a function of
   * the same name. This automatically gets called at the beginning of focus
   * stop()
   */
  virtual void stop_child(){};

  // node handle
  ros::NodeHandle nh_;
  // tf listener
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  // loop rate
  ros::Rate loop_rate_;

  // End effector frame name
   std::string ee_frame_;

 private:
  /* \brief Where generic publishing infrastructure is implemented  */
  void mainPubLoop() {
    geometry_msgs::PoseStamped target_pose;
    // Keep going until ROS dies or stop requested
    while (ros::ok() && continue_publishing_) {
      // Do the bulk of the calculation
      if (poseCalculation(target_pose)) {
        target_pose.header.stamp = ros::Time::now();
        target_pose_pub_.publish(target_pose);
      }

      // Sleep at loop rate
      loop_rate_.sleep();
    }
    return;
  }

  // Publisher to send poses to Servo Pose Tracking
  ros::Publisher target_pose_pub_;
  // Only continue publishing while this is true. Another thread can set this to
  // false and stop publishing
  std::atomic<bool> continue_publishing_{false};
  // Hold the thread we will run the main loop in - to avoid blocking whatever
  // calls start()
  std::thread thread_;
};

};  // namespace ee_focus
