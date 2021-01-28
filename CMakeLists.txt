cmake_minimum_required(VERSION 3.0.2)
project(ee_focus)

if(NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  look_at_pose
  message_generation
  moveit_servo
  std_srvs
  tf2
)

add_service_files(
  FILES
  PointToPose.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ee_focus src/ee_focus_main.cpp src/ee_focus.cpp src/ee_focus_publisher.cpp)
target_link_libraries(ee_focus ${catkin_LIBRARIES})
add_dependencies(ee_focus ${catkin_EXPORTED_TARGETS})

install(PROGRAMS scripts/servo_starter.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/dynamic_tf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
