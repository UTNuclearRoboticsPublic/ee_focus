#!/usr/bin/env python

# __TODO__
# Make sigint like handler to return terminal control
# Make output only print on a single line
# Make output print entire state
# Make increment user configurable?

import rospy
import tf2_ros
import geometry_msgs.msg

import tty
import sys
import termios


if __name__ == '__main__':
    try:
        camera_frame_name, target_frame_name = sys.argv[1:3]
    except:
        msg = """You did not provide command line frame args!
        Example: 'rosrun servo_camera_pointer dynamic_tf.py ee_link demo_link'"""
        rospy.logerr(msg)
        quit()
    
    information_msg = """
    ********************************************************************
    dyanmic_tf.py 
    ********************************************************************
    Used to dynamically control a frame to demonstrate 
    the servo_camera_pointer capability.
    ********************************************************************
    Press the following keys to control the translation of the 
    target frame.

    -x : 1
    +x : 2
    -y : 3
    +y : 4
    -z : 5
    +z : 6

    Use ESC to exit!
    ********************************************************************
    """
    print(information_msg)
    
    # Initialize node/publisher
    rospy.init_node('dynamic_tf_broadcaster')
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    
    # Initialize transform frame ids
    transform_stamped = geometry_msgs.msg.TransformStamped()
    transform_stamped.header.frame_id = camera_frame_name
    transform_stamped.child_frame_id = target_frame_name
    transform_stamped.transform.rotation.w = 1.0

    # Publish first tf
    broadcaster.sendTransform(transform_stamped)

    state = {"x": 0.0, "y": 0.0, "z": 0.0}

    # Store original terminal settings so we can restore the terminal when done
    orig_settings = termios.tcgetattr(sys.stdin)

    # Makes it so we don't need to hit enter for every input
    tty.setcbreak(sys.stdin)

    key = 0
    increment = 0.1 # (m) How much each key stroke will translate in the given direction
    while key != chr(27): # ESC
        try:
            key = sys.stdin.read(1)[0]
        except:
            msg = "An input error occured, restoring original terminal settings and exiting"
            rospy.logerr(msg)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)  
            quit()            

        if not key.isdigit():
            print("You selected a letter or special character not a number!")
        elif int(key) == 1:
            state["x"] -= increment
        elif int(key) == 2:
            state["x"] += increment
        elif int(key) == 3:
            state["y"] -= increment
        elif int(key) == 4:
            state["y"] += increment
        elif int(key) == 5:
            state["z"] -= increment
        elif int(key) == 6:
            state["z"] += increment
        else:
            print("Not a valid number key!")

        transform_stamped.transform.translation.x = state["x"]
        transform_stamped.transform.translation.y = state["y"]
        transform_stamped.transform.translation.z = state["z"]
        broadcaster.sendTransform(transform_stamped)

        #print("STATE  x: " + str(state["x"]) + "  y: " + str(state["y"]) + "  z: " + str(state["z"]) , end="\r", flush=True)
        print("STATE    x: %.2f  y: %.2f  z: %.2f" % (state["x"], state["y"], state["z"]), end="\r", flush=True)


    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)  
    
