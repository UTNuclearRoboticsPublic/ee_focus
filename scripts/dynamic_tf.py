#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg

import tty
import sys
import termios

# Store original terminal settings so we can restore the terminal when done
orig_settings = termios.tcgetattr(sys.stdin)

# Makes it so we don't need to hit enter for every input
tty.setcbreak(sys.stdin)

def main():
    try:
        camera_frame_name, target_frame_name = sys.argv[1:3]
    except:
        msg = """You did not provide command line frame args!
        Example: 'rosrun ee_focus dynamic_tf.py ee_link demo_link'"""
        rospy.logerr(msg)
        quit()
    
    information_msg = """
    ********************************************************************
    dyanmic_tf.py 
    ********************************************************************
    Used to dynamically control a frame to demonstrate 
    the ee_focus capability.
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

    # Non zero intial state to avoid self collision
    state = {"x": 1.0, "y": 1.0, "z": 1.0}

    transform_stamped.transform.translation.x = state["x"]
    transform_stamped.transform.translation.y = state["y"]
    transform_stamped.transform.translation.z = state["z"]
    transform_stamped.transform.rotation.w = 1.0

    # Publish first tf - prevents user from having to hit key before requesting service
    # because stdin will block and therefore the init demo_frame will usually be at 
    # the arm base and cause a self collision
    broadcaster.sendTransform(transform_stamped)

    key = 0
    increment = 0.1 # (m) How much each key stroke will translate in the given direction
    while key != chr(27): # ESC
        try:
            key = sys.stdin.read(1)[0]
        except:
            msg = "An unkown input error occured - key not read"
            rospy.logerr(msg)
            quit()

        if not key.isdigit():
            print("You selected a letter or special character!")
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
            print("Not a valid number key (hint: 1-6)!")

        transform_stamped.transform.translation.x = state["x"]
        transform_stamped.transform.translation.y = state["y"]
        transform_stamped.transform.translation.z = state["z"]
        broadcaster.sendTransform(transform_stamped)

        print("STATE    x: %.2f  y: %.2f  z: %.2f\r" % (state["x"], state["y"], state["z"]))

if __name__ == '__main__':
    try:
        main()
    finally:
        # Restore terminal control back to user
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)  
    
