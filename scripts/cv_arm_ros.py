#!/usr/bin/env python3

#RUN THIS IN TERMINAL
#stretch_robot_home.py
#roslaunch stretch_core stretch_driver.launch

import os
import playsound

import rospy
import time
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import hello_helpers.hello_misc as hm

import numpy as np
#import stretch_body.robot
from math import radians, degrees, atan2, sin, cos

import pyrealsense2 as rs
import cv2
import matplotlib.pyplot as plt
import math


class cv_indication(hm.HelloNode):
  
    def __init__(self):
        hm.HelloNode.__init__(self)
        self.rate = 10.0
        self.joint_states = None
        self.pan = None
        self.tilt = None
        self.pan_rotate = 0.0
        self.found = False
        self.depth = None


    def joint_states_callback(self, msg):
        self.joint_states = msg
        pan_index = self.joint_states.name.index('joint_head_pan')
        tilt_index = self.joint_states.name.index('joint_head_tilt')
        self.pan = self.joint_states.position[pan_index]
        self.tilt = self.joint_states.position[tilt_index]
        self.cv()

    def move_cam(self, pan, tilt):
        point = JointTrajectoryPoint()
        point.positions = [pan, tilt]
        point.velocities = [1.0, 1.0]
        point.accelerations = [1.5, 1.5]

        trajectory_goal = FollowJointTrajectoryGoal()
        trajectory_goal.trajectory.joint_names = ['joint_head_pan', 'joint_head_tilt']
        trajectory_goal.trajectory.points = [point]
        trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
        trajectory_goal.trajectory.header.frame_id = 'base_link'

        self.trajectory_client.send_goal(trajectory_goal)
        rospy.loginfo('Sent list of goals = {0}'.format(trajectory_goal))
        self.trajectory_client.wait_for_result()  

    def cv(self):

        # Configure depth and color streams
        pipeline = rs.pipeline()
        config = rs.config()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        found_rgb = False
        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            print("The demo requires Depth camera with Color sensor")
            exit(0)

        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        if device_product_line == 'L500':
            config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
        else:
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        pipeline.start(config)
        counter = 0
        previous_x = 0
        previous_y = 0
        boss_flag = False
        return_flag = False

        try:
            start = time.time()
            flag = 0
            #while True:
            if math.degrees(self.pan) < -180:
                return_flag = False
                #break
            

            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            #if not depth_frame or not color_frame:
                #continue
                
            # Create alignment primitive with color as its target stream:
            align = rs.align(rs.stream.color)
            frames = align.process(frames)

            # Update color and depth frames:
            aligned_depth_frame = frames.get_depth_frame()
            colorizer = rs.colorizer()
            colorized_depth = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())

            # Convert images to numpy arrays
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Boss add rotation 90CW
            depth_image = cv2.rotate(depth_image, cv2.ROTATE_90_CLOCKWISE) 
            color_image = cv2.rotate(color_image, cv2.ROTATE_90_CLOCKWISE) 
            depth_colormap_dim = depth_image.shape
            color_colormap_dim = color_image.shape
            
            # Detect circle in color image
            # Use masking
            HSVframe = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

            # Green
            sensitivity = 20
            lower = np.array([60-sensitivity, 90, 90])
            upper = np.array([60+sensitivity, 255, 255])

            mask = cv2.inRange(HSVframe, lower, upper)
            mask = cv2.bitwise_not(mask)

            # Blur image
            blur_img = cv2.medianBlur(mask,25)

            #cv2.imshow("mask", mask)

            # Blob detection
            detector = cv2.SimpleBlobDetector_create()

            # Blur image again
            blur_img2 = cv2.medianBlur(mask,5)

            #Detect Blobs
            keypoints = detector.detect(blur_img2)
            if len(keypoints) == 1:
                #rotate1 = 0.0
                #robot.head.move_to('head_pan',rotate1)
                print(keypoints[0].pt[0],keypoints[0].pt[1],keypoints[0].size)
                if (keypoints[0].pt[0] < (previous_x+10)) and (keypoints[0].pt[0] > (previous_x-10)):
                    if (keypoints[0].pt[1] < (previous_y+10)) and (keypoints[0].pt[1] > (previous_y-10)):
                        counter = counter + 1
                previous_x = keypoints[0].pt[0]
                previous_y = keypoints[0].pt[1]

            im_with_keypoints = cv2.drawKeypoints(blur_img2, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        
            # Show keypoints
            #cv2.imshow("Keypoints", im_with_keypoints)

            if cv2.waitKey(1) and counter > 10:
                print("Object found")
                x = int(keypoints[0].pt[0])
                y = int(keypoints[0].pt[1])
                    

                # Center camera on object # horizontal = based on real world, not camera's center
                horizontal_error = ((42 * x)/480) - 21
                if abs(horizontal_error) > 2.0:
                    horizontal_input = -math.radians(horizontal_error) + self.pan
                vertical_error = ((69 * y)/640) - 34.5
                if abs(vertical_error) > 2.0:
                    vertical_input = -math.radians(vertical_error) + self.tilt

                self.move_cam(horizontal_input, vertical_input)  

                #ignore 0 and nan values
                depth_image = depth_image.astype(float)
                depth_image[depth_image == 0.0] = np.NaN
                
                print(x,y)
                # Get depth original
                dist = aligned_depth_frame.get_distance(y,x) #XY must still be swapped because we pull orginal data from realsense
                print("Detected object {} meters away.".format(dist))
                return_flag = True

                time.sleep(1)
                flag = 1
                
                if boss_flag == True:
                    break
                boss_flag = True

            # the 'e' button is set as the quitting button
            elif cv2.waitKey(1) & 0xFF == ord('e'):
                break
            stop = time.time() # seconds

            # Rotate to find object 
            if (stop-start) > 2 and flag == 0:
                self.pan_rotate += -0.1
                print('hey')
                self.move_cam(self.pan_rotate, 0.52) # keep tilt to 30 degrees but rotate pan    
        finally:
            # Stop streaming
            pipeline.stop()
            if return_flag:
                self.found = True
                self.depth = float(dist)
                return True, float(dist)
            else:
                self.found = False
                self.depth = 0.00
                return False, 0.00
            
    def cv_test(self):
        return(True, 0.5)

    def arm_stow(self):
        point0 = JointTrajectoryPoint()
        point0.positions = [0.2, 0.0, 3.4]
        point0.velocities = [0.2, 0.2, 2.5]
        point0.accelerations = [1.0, 1.0, 3.5]

        trajectory_goal = FollowJointTrajectoryGoal()
        trajectory_goal.trajectory.joint_names = ['joint_lift', 'wrist_extension', 'joint_wrist_yaw']
        trajectory_goal.trajectory.points = [point0]
        trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
        trajectory_goal.trajectory.header.frame_id = 'base_link'

        self.trajectory_client.send_goal(trajectory_goal)
        rospy.loginfo('Sent list of goals = {0}'.format(trajectory_goal))
        self.trajectory_client.wait_for_result()  

    def arm_point(self, depth, tilt):
        """
        Function that makes an action call and sends multiple joint trajectory goals
        to the joint_lift, wrist_extension, and joint_wrist_yaw.
        :param self: The self reference.
        """

        #wrapToPi = lambda theta_rad: atan2(sin(theta_rad), cos(theta_rad))

        # Dimensions params
        h_cam = 1.3
        l_cam = 0.14
        h_grip = 0.173
        h_tolerance = 0.05
        l_grip = 0.216
        l_tolerance = 0.08

        lift_arm_real = h_cam + h_grip + h_tolerance - depth*np.sin(abs(tilt))
        extend_arm_real = depth*np.cos(abs(tilt)) + l_cam - l_grip - l_tolerance

        lift_arm_command = lift_arm_real - 0.19
        extend_arm_command = extend_arm_real - 0.28

        point0 = JointTrajectoryPoint()
        point0.positions = [0.2, 0.0, 3.4]
        point0.velocities = [0.2, 0.2, 2.5]
        point0.accelerations = [1.0, 1.0, 3.5]

        point1 = JointTrajectoryPoint()
        point1.positions = [lift_arm_command, 0.0, 3.14]  # lift arm but keep it stowed
        point1.positions = [0.7, 0.0, 3.14]

        point2 = JointTrajectoryPoint()
        point2.positions = [lift_arm_command, extend_arm_command, 0.0]  # stay at the height and extend, yaw gripper
        point2.positions = [0.7, 0.1, 0.0]

        trajectory_goal = FollowJointTrajectoryGoal()
        trajectory_goal.trajectory.joint_names = ['joint_lift', 'wrist_extension', 'joint_wrist_yaw']
        trajectory_goal.trajectory.points = [point0, point1, point2]
        trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
        trajectory_goal.trajectory.header.frame_id = 'base_link'

        self.trajectory_client.send_goal(trajectory_goal)
        rospy.loginfo('Sent list of goals = {0}'.format(trajectory_goal))
        self.trajectory_client.wait_for_result()  


    def play_audio_found(self):
        
        print('playing found sound')
        playsound.playsound("/home/hello-robot/Downloads/found_it.mp3",True)

    def play_audio_not_found(self):    
        print('playing not found sound')
        playsound.playsound("/home/hello-robot/Downloads/not_here.mp3",True)

    def main(self):
        """
        Main function: 
        """

        #************************CV PART*******************************
        hm.HelloNode.main(self, 'find_object_and_indicate', 'find_object_and_indicate', wait_for_first_pointcloud=False)

        self.joint_states_subscriber = rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)
        #self.cv()
        #found = True
        #depth = 0.5 # *** Input depth later ***
        #tilt = 0.5 # *** Input tilt later ***

        #*********************Indication Part*************************
        #hm.HelloNode.main(self, 'multipoint_command', 'multipoint_command', wait_for_first_pointcloud=False)
        rospy.loginfo('Running indication command...')
        return self.found, self.depth, self.tilt

    def indicate(self, found, depth, tilt):

        self.arm_stow()
        time.sleep(2)
        print('FOUND!!!!!!!!!!!!!')

        if found: 
            self.arm_point(depth,tilt)
            time.sleep(2)
            self.play_audio_found()
            return True
        else:
            self.play_audio_not_found()
            return False

if __name__ == '__main__':
    node = cv_indication()
    while not rospy.is_shutdown():
        found, depth, tilt = node.main()
        if found:
            break 
    node.indicate(found, depth, tilt)
    rospy.spin()

