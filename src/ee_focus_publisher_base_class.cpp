#include <ee_focus/ee_focus_publisher_base_class.h>

namespace ee_focus {

void EEFPublisherBase::initialize(ros::NodeHandle& nh,
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

void EEFPublisherBase::start() {
  continue_publishing_ = true;

  thread_ = std::thread([this] { mainPubLoop(); });

  return;
}

void EEFPublisherBase::stop() {
  continue_publishing_ = false;

  // Too many horror stories of problems caused by leaving out brackets,
  // sorry :)
  if (thread_.joinable()) {
    thread_.join();
  }

  return;
}

EEFPublisherBase::~EEFPublisherBase() { stop(); }

void EEFPublisherBase::mainPubLoop() {
  geometry_msgs::TransformStamped cam_to_gravity_tf, cam_to_target_tf;
  geometry_msgs::Vector3Stamped gravity;
  geometry_msgs::PoseStamped init_cam_pose;
  geometry_msgs::PoseStamped target_look_pose;
  look_at_pose::LookAtPose look_at_pose_service;

  // The initial EE pose is always identity in the EE frame
  init_cam_pose.header.frame_id = ee_frame_;
  init_cam_pose.pose.orientation.w = 1;

  // Keep going until ROS dies or stop requested
  while (ros::ok() && continue_publishing_) {
    // Look up the current transforms
    try {
      cam_to_gravity_tf = tf_buffer_.lookupTransform(
          ee_frame_, z_axis_up_frame_, ros::Time(0), ros::Duration(1));
      cam_to_target_tf = tf_buffer_.lookupTransform(
          ee_frame_, target_frame_, ros::Time(0), ros::Duration(1));
    } catch (tf2::TransformException& ex) {
      ROS_ERROR_THROTTLE(1, "%s", ex.what());
      loop_rate_.sleep();
      continue;
    }

    // Convert the transform for the gravity frame to a rotation matrix
    Eigen::Quaterniond q_gravity(cam_to_gravity_tf.transform.rotation.w,
                                 cam_to_gravity_tf.transform.rotation.x,
                                 cam_to_gravity_tf.transform.rotation.y,
                                 cam_to_gravity_tf.transform.rotation.z);
    Eigen::Matrix3d R_gravity = q_gravity.normalized().toRotationMatrix();

    // Populate gravity vector as the z-axis of rotation matrix
    gravity.header.frame_id = cam_to_gravity_tf.header.frame_id;
    gravity.vector.x = R_gravity(0, 2);
    gravity.vector.y = R_gravity(1, 2);
    gravity.vector.z = R_gravity(2, 2);

    // We need to update the time for the initial EE pose (identity)
    init_cam_pose.header.stamp = ros::Time::now();

    // Set the target pose in the EE frame using the found transformation
    target_look_pose.header.frame_id = cam_to_target_tf.header.frame_id;
    target_look_pose.header.stamp = cam_to_target_tf.header.stamp;
    target_look_pose.pose.position.x = cam_to_target_tf.transform.translation.x;
    target_look_pose.pose.position.y = cam_to_target_tf.transform.translation.y;
    target_look_pose.pose.position.z = cam_to_target_tf.transform.translation.z;
    target_look_pose.pose.orientation = cam_to_target_tf.transform.rotation;

    // Populate the look_at_pose service request
    look_at_pose_service.request.initial_cam_pose = init_cam_pose;
    look_at_pose_service.request.target_pose = target_look_pose;
    look_at_pose_service.request.up = gravity;

    // Call the look_at_pose service
    if (!look_pose_client_.call(look_at_pose_service)) {
      ROS_ERROR_STREAM_THROTTLE(1, "NO RESPONSE FROM: look_at_pose");
      loop_rate_.sleep();
      continue;
    }

    // Publish the pose
    look_at_pose_service.response.new_cam_pose.header.stamp = ros::Time::now();
    target_pose_pub_.publish(look_at_pose_service.response.new_cam_pose);

    // Sleep at loop rate
    loop_rate_.sleep();
  }
  return;
}

geometry_msgs::PoseStamped EEFPublisherBase::poseCalulation() {
  geometry_msgs::PoseStamped dummy;

  return dummy;
}

// TODO what about this default loop rate value?
// Rate can't be changed afer intialization, can we
// make it a non class var?
// We got lucky that the tf_* stuff doesn't require
// a real external paramaterization, but that's not
// the case with ros::Rate :()
EEFPublisherBase::EEFPublisherBase()
    : tf_buffer_(), tf_listener_(tf_buffer_), loop_rate_(10) {}

};  // namespace ee_focus

int main(int argc, char** argv) { return 0; }