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

void EEFPublisherBase::mainPubLoop() { return; }

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