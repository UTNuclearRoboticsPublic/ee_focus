# Install
Going to install in a completely seperate workspace

First we follow the [MoveIt source install instructions](https://moveit.ros.org/install/source/):

```sh
# Download/setup Moveit
mkdir ws_camera_pointing
cd ws_camera_pointing
source /opt/ros/melodic/setup.bash
wstool init src
wstool merge -t src https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall
wstool update -t src
rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release 
```

Optionally we should maybe be sure to be on the same commit of MoveIt (the above grabs the most current from `master` branch on GitHub). Servo has been updated since the commit below and we should catch up to current, but right now the below commit is where I am at

```sh
# Checkout good MoveIt version
pushd src/moveit
git checkout 475e5810911961ae51cdb5cab0961471f5d4aa1a
popd
```

Build MoveIt. I always run with small amount of cores or my computer freezes up :(

```sh
catkin build -j 2
```

Install camera pointing package and build
```sh
pushd src
git clone --branch devel https://github.com/UTNuclearRobotics/vbats.git
git clone https://github.com/UTNuclearRoboticsPublic/look_at_pose.git
git clone https://github.com/ros-industrial/universal_robot.git
rosdep install --from-paths . --ignore-src -y
popd
catkin build
source devel/setup.bash
```

# Demo
Roughly following the instructions for the [Servo demo](https://github.com/ros-planning/moveit/tree/master/moveit_ros/moveit_servo).

In 3 seperate terminals (sorry...)
```sh
source ~/ws_camera_pointing/devel/setup.bash
roslaunch ur_gazebo ur5.launch gui:=false
```

```sh
source ~/ws_camera_pointing/devel/setup.bash
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true
```

```sh
source ~/ws_camera_pointing/devel/setup.bash
roslaunch ur5_moveit_config moveit_rviz.launch config:=true
```

In RViz, plan-execute the arm to somewhere that is not singular. In yet a 4th terminal:
```sh
source ~/ws_camera_pointing/devel/setup.bash

rosservice call /controller_manager/switch_controller "start_controllers:
- 'joint_group_position_controller'
stop_controllers:
- 'arm_controller'
strictness: 2"

roslaunch servo_camera_pointer camera_pointer_demo.launch
```
In I promise the final terminal:
```sh
rosservice call /servo_camera_pointer/start_camera_pointing
```
