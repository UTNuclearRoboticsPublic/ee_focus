# servo_camera_pointer

This uses MoveIt Servo's pose tracking feature to continuously point a camera at a target frame (in the robot URDF)

Intended steps of use:
  1) Copy the launch file into your own project
  2) Modify the copied launch file to load the correct Servo parameters
  3) Determine the inputs to the camera pointer (frame names, loop rate, etc)
  4) Launch and pass input args (see below). Alternatively, set the defaults args in your modified launch file to match your values


## Launching
To launch from the command line:
```sh
roslaunch servo_camera_pointer servo_camera_pointer.launch camera_frame_name:=CAMERA gravity_frame_name:=GRAVITY target_frame_name:=TARGET loop_rate:=LOOP_RATE
```

To launch from another launch file:
```sh
<include file="$(find PACKAGE)/launch/servo_camera_pointer.launch">
    <arg name="camera_frame_name" value="CAMERA"/>
    <arg name="gravity_frame_name" value="GRAVITY"/>
    <arg name="target_frame_name" value="TARGET"/>
    <arg name="loop_rate" value="LOOP_RATE"/>
</include>
```
