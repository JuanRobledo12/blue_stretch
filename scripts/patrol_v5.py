#!/usr/bin/env python

'''
Description:
This is a ROS Node to navigate in a space and take photos on specific waypoints. YOLO V7 is used to classify the objects inside the images and saved them in folders.
In addition, a JSON file is updated containing all the metadata of the images.

This version only collect images, the object detection was removed.

Last mod: 2023-Aug-24
Version: 5

'''




# Dependencies
import json
import roslib
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import sample
from math import pow, sqrt, radians
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import hello_helpers.hello_misc as hm
from time import sleep
from move_robot_body_class import JointController

############################ CV BEGINS ##############################
import numpy as np
import datetime
import os
import argparse
import time
from pathlib import Path
import cv2
import torch
import torch.backends.cudnn as cudnn
from numpy import random
from json_handler_class import JSON_Handler
from img_classifier import detect
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


# create instance of class
json_handler = JSON_Handler(5, '/home/hello-robot/catkin_ws/src/blue_stretch/scripts/waypoint_info.json', new_json=True)


# CHANGE THIS WITH THE SPECIFIC WAYPOINTS AND PICTURES PER WAYPOINTS
waypoints_num = 12
photos_per_waypoint = 9

rows,cols = (waypoints_num, photos_per_waypoint)  # (waypoints, photos per waypoint)
timestamp_array = ([[0 for i in range(cols)] for j in range(rows)])



counter = 0
previous_x = 0
previous_y = 0
#################### CV ENDS #############################

class CollectData():
    def __init__(self):

        rospy.init_node('collect_data', anonymous=True)
        hm.HelloNode.__init__(self)
        
        rospy.on_shutdown(self.shutdown)
        
        # initializes a CvBridge class, subscriber, and save path.:param self: The self reference.
        self.bridge = CvBridge()
        self.img_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.camera_img_callback, queue_size=1)

        
        # How long in seconds should the robot pause at each location?
        self.rest_time = rospy.get_param("~rest_time", 1)
        
        # Are we running in the fake simulator?
        self.fake_test = rospy.get_param("~fake_test", False)
        
        
        # create instance of joint controller class and a list of endpoints to move the camera
        self.joint_controller = JointController()
        self.endpoints_ls = [[-0.7, 0.0], [0, 0.0], [0.7, 0.0], [0.7, -0.3], [0, -0.3], [-0.7, -0.3], [-0.7, -0.7], [0, -0.7], [0.7, -0.7]]
        
        
        # Goal state return values
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 
                       'SUCCEEDED', 'ABORTED', 'REJECTED',
                       'PREEMPTING', 'RECALLING', 'RECALLED',
                       'LOST']
        
        # Set up the goal locations. Poses are defined in the map frame.  
        # An easy way to find the pose coordinates is to point-and-click
        # Nav Goals in RViz when running in the simulator.
        # Pose coordinates are then displayed in the terminal
        # that was used to launch RViz.

        f = open("/home/hello-robot/catkin_ws/src/blue_stretch/scripts/waypoint_info.json")
        self.location_data = json.load(f)

        locations = []
        locations_name = []

        # saving the locations in a list
        for loc in self.location_data.keys():
            locations.append(Pose(Point(
                                        self.location_data[loc]["pose"]["position"][0],
                                        self.location_data[loc]["pose"]["position"][1],
                                        self.location_data[loc]["pose"]["position"][2]
                                        ), 
                                        Quaternion(
                                            self.location_data[loc]["pose"]["orientation"][0],
                                            self.location_data[loc]["pose"]["orientation"][1],
                                            self.location_data[loc]["pose"]["orientation"][2],
                                            self.location_data[loc]["pose"]["orientation"][3]
                                        )))
            locations_name.append(self.location_data[loc]["name"])
        print(locations)
        
        
        # Publisher to manually control the robot (e.g. to stop it)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")
        
        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))
        
        rospy.loginfo("Connected to move base server")
        
        # A variable to hold the initial pose of the robot to be set by 
        # the user in RViz
        initial_pose = PoseWithCovarianceStamped()
        
        # Variables to keep track of success rate, running time,
        # and distance traveled
        n_locations = len(locations)
        n_goals = 0
        n_successes = 0
        i = n_locations
        distance_traveled = 0
        start_time = rospy.Time.now()
        location = ""
        last_location = ""
        object_found = False
        
        # Get the initial pose from the user
        rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
        rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
        start_time = time.time()
        self.last_location = Pose()
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)
        
        # Make sure we have the initial pose
        while initial_pose.header.stamp == "":
            rospy.sleep(1)
            
        rospy.loginfo("Starting navigation test")
        
        # Begin the main loop and run through a sequence of locations
        while True:

            

            for loc_idx in range(len(locations)):
            # Get the next location in the current sequence
                location = locations[loc_idx]
                            
                
                distance = 000

                # Store the last location for distance calculations
                last_location = location
                
                # Increment the counters
                i += 1
                n_goals += 1
            
                # Set up the next goal location
                self.goal = MoveBaseGoal()
                self.goal.target_pose.pose = location
                self.goal.target_pose.header.frame_id = 'map'
                self.goal.target_pose.header.stamp = rospy.Time.now()
                
                # Let the user know where the robot is going next
                rospy.loginfo("Going to: " + str(location))
                
                # Start the robot toward the next location
                self.move_base.send_goal(self.goal)
                
                # Allow 5 minutes to get there
                finished_within_time = self.move_base.wait_for_result(rospy.Duration(300)) 
                
                # Check for success or failure
                if not finished_within_time:
                    self.move_base.cancel_goal()
                    rospy.loginfo("Timed out achieving goal")
                else:
                    state = self.move_base.get_state()
                    if state == GoalStatus.SUCCEEDED:
                        rospy.loginfo("Goal succeeded!")

                        #### CV CODE!!!!!#####
                        img_counter = 0
                        for endpoint in self.endpoints_ls:
                            self.joint_controller.move_camera(endpoint)
                            time.sleep(3)
                            self.get_image(loc_idx, img_counter)
                            img_counter += 1
                        self.joint_controller.move_camera([0.0, 0.0])
                        #################################
                        n_successes += 1
                        distance_traveled += distance
                        rospy.loginfo("State:" + str(state))
                    else:
                      rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))
                
                
                # Print a summary success/failure, distance traveled and time elapsed
                rospy.loginfo("Success so far: " + str(n_successes) + "/" + 
                              str(n_goals) + " = " + 
                              str(100 * n_successes/n_goals) + "%")
                rospy.sleep(self.rest_time)
            
            ######   CV BEGINS ######
            # Destroy all the windows
            cv2.destroyAllWindows()  

            #################### THIS SECTION IS COMMENTED OUT TO AVOID RUNNING THE OBJECT DETECTOR CLASS ####################

            # Classify objects in images
            # for w in range(waypoints_num): #Select the correct number of waypoints in your system.
                
            #     new_project = "/home/tony/yolov7_models/images_result/waypoint" + str(w + 1)
            #     new_source = "/home/tony/yolov7_models/images_demo/waypoint" + str(w + 1) + "/"

            #     for i in range(photos_per_waypoint): #change it according to the number of images you take per waypoint
            #         img_name = ""
            #         source=""
            #         img_no = str(i+1)
            #         ####### Print W and I and reduce the W range.
            #         date_time = timestamp_array[w][i]
            #         img_name = str(date_time) + "_img_" + str(img_no) + ".png"
            #         source = new_source + img_name

            #         detect(source,new_project,date_time,img_name,w)

            ######################################################################################################################

            rospy.loginfo('Data Collection Run Completed!!!!!')
            end_time = time.time()
            elapsed_time = end_time - start_time
            print(f"Total execution time: {elapsed_time:.5f} seconds")
            self.shutdown()
            break
        
        #Finish the program's run
        quit()
    
    
    ###### CV FUNCTION ##############
    def camera_img_callback(self, msg):
        """
        A callback function that converts the ROS image to a CV2 image and stores the
        image.
        :param self: The self reference.
        :param msg: The ROS image message type.
        """
        try:
            self.camera_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            rospy.logwarn('CV Bridge error: {0}'.format(e))
    
    def get_image(self,waypoint, img_counter):
        
        directory = "/home/hello-robot/yolov7/images_demo_test/waypoint" + str(waypoint + 1)
                
        # Wait for frames
        
        #camera_img_resized = cv2.resize(self.camera_image, (640, 640))
        camera_img_rotated = cv2.rotate(self.camera_image, cv2.ROTATE_90_CLOCKWISE)
        img_counter = img_counter + 1

        # Get time
        date_time = datetime.datetime.now().strftime("%H_%M_%S")
        timestamp_array[waypoint][img_counter-1] = date_time
        filename = date_time + "_img_" + str(img_counter) + ".png"  

        # Save data
        os.chdir(directory)
        cv2.imwrite(filename, camera_img_rotated)
        print('image saved',directory)


    ###### CV FUNC ENDS ################

    def update_initial_pose(self, initial_pose):
        self.initial_pose = initial_pose

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        #rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

def trunc(f, n):
    # Truncates/pads a float f to n decimal places without rounding
    slen = len('%.*f' % (n, f))
    return float(str(f)[:slen])

if __name__ == '__main__':
    try:
        CollectData()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL navigation test finished.")
