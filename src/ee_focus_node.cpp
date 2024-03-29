#include <ee_focus/ee_focus.h>

static const std::string LOGNAME = "ee_focus_main";

int main(int argc, char** argv) {
  ros::init(argc, argv, LOGNAME);
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(0);
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
  auto tracker =
      std::make_unique<moveit_servo::PoseTracking>(nh, planning_scene_monitor);

  ee_focus::EEFocus ee_focus(nh, std::move(tracker));
  ee_focus.spin();

  return EXIT_SUCCESS;
}
