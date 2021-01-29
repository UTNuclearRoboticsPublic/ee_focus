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

const bool EEFPublisherBase::poseCalulation(
    geometry_msgs::PoseStamped& target_pose) {
  // TODO in full implementation, these are prob class variables of super class
  // to avoid constantly init them
  geometry_msgs::TransformStamped ee_to_gravity_tf, ee_to_target_tf;
  geometry_msgs::PoseStamped init_ee_pose;

  // The initial EE pose is always identity in the EE frame
  init_ee_pose.header.frame_id = ee_frame_;  // TODO do in superclass init?
  init_ee_pose.pose.orientation.w = 1;       // TODO do in superclass init?

  // TODO pass look at pose service as a pointer
  look_at_pose::LookAtPose
      look_at_pose_service;  // TODO super class member variable?

  // Look up the current transforms
  try {
    ee_to_gravity_tf = tf_buffer_.lookupTransform(
        ee_frame_, z_axis_up_frame_, ros::Time(0), ros::Duration(1));
    ee_to_target_tf = tf_buffer_.lookupTransform(
        ee_frame_, target_frame_, ros::Time(0), ros::Duration(1));
  } catch (tf2::TransformException& ex) {
    ROS_ERROR_THROTTLE(1, "%s", ex.what());
    return false;
  }

  // Convert the transform for the gravity frame to a rotation matrix
  Eigen::Quaterniond q_gravity(ee_to_gravity_tf.transform.rotation.w,
                               ee_to_gravity_tf.transform.rotation.x,
                               ee_to_gravity_tf.transform.rotation.y,
                               ee_to_gravity_tf.transform.rotation.z);
  Eigen::Matrix3d R_gravity = q_gravity.normalized().toRotationMatrix();

  // Populate gravity vector as the z-axis of rotation matrix
  geometry_msgs::Vector3Stamped gravity;
  gravity.header.frame_id = ee_to_gravity_tf.header.frame_id;
  gravity.vector.x = R_gravity(0, 2);
  gravity.vector.y = R_gravity(1, 2);
  gravity.vector.z = R_gravity(2, 2);

  // Set the target pose in the EE frame using the found transformation
  geometry_msgs::PoseStamped target_look_pose;
  target_look_pose.header.frame_id = ee_to_target_tf.header.frame_id;
  target_look_pose.header.stamp = ee_to_target_tf.header.stamp;
  target_look_pose.pose.position.x = ee_to_target_tf.transform.translation.x;
  target_look_pose.pose.position.y = ee_to_target_tf.transform.translation.y;
  target_look_pose.pose.position.z = ee_to_target_tf.transform.translation.z;
  target_look_pose.pose.orientation = ee_to_target_tf.transform.rotation;

  // Populate the look_at_pose service request
  look_at_pose_service.request.target_pose = target_look_pose;
  look_at_pose_service.request.up = gravity;

  // We need to update the time for the initial EE pose (identity)
  init_ee_pose.header.stamp = ros::Time::now();
  // Populate the rest of the look_at_pose service request
  look_at_pose_service.request.initial_cam_pose = init_ee_pose;

  // Call the look_at_pose service
  if (!look_pose_client_.call(look_at_pose_service)) {
    ROS_ERROR_STREAM_THROTTLE(1, "NO RESPONSE FROM: look_at_pose");
    return false;
  }

  target_pose = look_at_pose_service.response.new_cam_pose;
  return true;
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
