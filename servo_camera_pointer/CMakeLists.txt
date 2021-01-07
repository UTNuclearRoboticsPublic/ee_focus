cmake_minimum_required(VERSION 3.0.2)
project(servo_camera_pointer)

if(NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
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

add_executable(camera_pointer src/camera_pointer_main.cpp src/camera_pointer.cpp src/camera_pointer_publisher.cpp)
target_link_libraries(camera_pointer ${catkin_LIBRARIES})
add_dependencies(camera_pointer ${catkin_EXPORTED_TARGETS})
