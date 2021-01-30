#include <ee_focus/ee_focus_publisher_child_class.h>

namespace ee_focus {

UnconstrainedCameraPointer::UnconstrainedCameraPointer() {}

const bool UnconstrainedCameraPointer::poseCalulation(
    geometry_msgs::PoseStamped& target_pose) {
  // TODO in full implementation, these are prob class variables of super class
  // to avoid constantly init them
  geometry_msgs::TransformStamped ee_to_gravity_tf, ee_to_target_tf;
  geometry_msgs::PoseStamped init_ee_pose;

  // The initial EE pose is always identity in the EE frame
  init_ee_pose.header.frame_id = getEEFrame();  // TODO do in superclass init?
  init_ee_pose.pose.orientation.w = 1;          // TODO do in superclass init?

  // TODO pass look at pose service as a pointer
  look_at_pose::LookAtPose
      look_at_pose_service;  // TODO super class member variable?

  // Look up the current transforms
  try {
    ee_to_gravity_tf = getTFBuffer().lookupTransform(
        getEEFrame(), getZAxisUpFrame(), ros::Time(0), ros::Duration(1));
    ee_to_target_tf = getTFBuffer().lookupTransform(
        getEEFrame(), getTargetFrame(), ros::Time(0), ros::Duration(1));
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
  if (!getLookPoseClient().call(look_at_pose_service)) {
    ROS_ERROR_STREAM_THROTTLE(1, "NO RESPONSE FROM: look_at_pose");
    return false;
  }

  target_pose = look_at_pose_service.response.new_cam_pose;
  return true;
}

}  // namespace ee_focus