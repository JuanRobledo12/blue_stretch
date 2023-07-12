#!/usr/bin/env python

from move_arm_class import JointController, Joints
import rospy

def main():
    # Initialize the node with rospy
    rospy.init_node('move_camera_node')

    # Initialize the JointController object
    joint_controller = JointController()

    # Specify your camera endpoint. For example:
    camera_endpoint = [-0.9, -0.9]  # [pan_val, tilt_val]

    # Call the move_camera method of the JointController class
    joint_controller.move_camera(camera_endpoint)


    joint_controller.stow()

    # Keep the node alive
    rospy.spin()

if __name__ == '__main__':
    main()

'''
############################# JOINT LIMITS #############################
joint_lift:      lower_limit =  0.15,  upper_limit =  1.10  # in meters
wrist_extension: lower_limit =  0.00,  upper_limit =  0.50  # in meters
joint_wrist_yaw: lower_limit = -1.75,  upper_limit =  4.00  # in radians
joint_head_pan:  lower_limit = -2.80,  upper_limit =  2.90  # in radians
joint_head_tilt: lower_limit = -1.60,  upper_limit =  0.40  # in radians
joint_gripper_finger_left:  lower_limit = -0.35,  upper_limit =  0.165  # in radians

# INCLUDED JOINTS IN POSITION MODE
translate_mobile_base: No lower or upper limit. Defined by a step size in meters
rotate_mobile_base:    No lower or upper limit. Defined by a step size in radians
########################################################################
'''