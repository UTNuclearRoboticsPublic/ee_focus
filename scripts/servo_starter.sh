#!/bin/bash
# Sleep to let Gazebo finishing starting up...
sleep 10

# Unpause Gazebo
echo "Unpausing physics"
rosservice call --wait /gazebo/unpause_physics

# Set the controller for Servo commands instead of trajectories
echo "Starting Servo controller"
rosservice call --wait /controller_manager/switch_controller "start_controllers:
- 'joint_group_position_controller'
stop_controllers:
- 'arm_controller'
strictness: 2"
