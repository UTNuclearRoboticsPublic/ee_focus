#pragma once

#include <ee_focus/ee_focus_publisher_base_class.h>

namespace ee_focus {

class UnconstrainedCameraPointer : public EEFPublisherBase {
 public:
  UnconstrainedCameraPointer();

  /* \brief Where all the work to calculate unconstrained camera pose happens */
  const bool poseCalulation(geometry_msgs::PoseStamped& target_pose);
};

}  // namespace ee_focus
