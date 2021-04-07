cmake_minimum_required(VERSION 3.0.2)
project(ee_focus)

if(NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(FOCUS_LIBRARY_NAME ee_focus_plugins)

find_package(catkin REQUIRED COMPONENTS
  look_at_pose
  message_generation
  moveit_servo
  std_msgs
  std_srvs
  tf2
)

add_service_files(
  FILES
  SetTargetFrame.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${FOCUS_LIBRARY_NAME}
  CATKIN_DEPENDS
    look_at_pose
    moveit_servo
    std_srvs
    tf2
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${FOCUS_LIBRARY_NAME}
  src/ee_focus.cpp
  src/ee_focus_default_plugins.cpp
  include/${PROJECT_NAME}/ee_focus_publisher_base_class.h
  include/${PROJECT_NAME}/unconstrained_camera_pointer.h
)
target_link_libraries(${FOCUS_LIBRARY_NAME} ${catkin_LIBRARIES})
add_dependencies(${FOCUS_LIBRARY_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(ee_focus src/ee_focus_node.cpp)
target_link_libraries(ee_focus ${catkin_LIBRARIES} ${FOCUS_LIBRARY_NAME})
add_dependencies(ee_focus ${catkin_EXPORTED_TARGETS})

##################
## INSTALLATION ##
##################

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${FOCUS_LIBRARY_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(PROGRAMS scripts/servo_starter.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/dynamic_tf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
