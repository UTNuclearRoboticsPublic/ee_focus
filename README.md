# ee_focus

# Overview
This uses MoveIt Servo's pose tracking feature to continuously point a EE frame at a target frame (in the robot URDF)

Intended steps of use:
  1) Copy the launch file into your own project
  2) Modify the copied launch file to load the correct Servo parameters
  3) Determine the inputs to the `ee_focus` (frame names, loop rate, etc)
  4) Launch and pass input args (see below). Alternatively, set the defaults args in your modified launch file to match your values

## Launching
To launch from the command line:
```sh
roslaunch ee_focus ee_focus.launch ee_frame_name:=EEF gravity_frame_name:=GRAVITY target_frame_name:=TARGET loop_rate:=LOOP_RATE
```

To launch from another launch file:
```sh
<include file="$(find PACKAGE)/launch/ee_focus.launch">
    <arg name="ee_frame_name" value="EEF"/>
    <arg name="gravity_frame_name" value="GRAVITY"/>
    <arg name="target_frame_name" value="TARGET"/>
    <arg name="loop_rate" value="LOOP_RATE"/>
    <arg name="rotational_tolerance" value="0.05"/>
</include>
```
