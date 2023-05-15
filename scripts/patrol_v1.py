#!/usr/bin/env python

import json
import roslib
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import sample
from math import pow, sqrt, radians
# f/home/hello-robot/home/hello-robotrom tsp_solver import *
#from test_merge_cv_arm_boss import *
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import hello_helpers.hello_misc as hm
import stretch_body.robot
from time import sleep

################# CV BEGINS ##############################
import numpy as np
import datetime
import os
import argparse
import time
from pathlib import Path
import pyrealsense2 as rs
import cv2
import torch
import torch.backends.cudnn as cudnn
from numpy import random
#from json_handler_class_edited import JSON_Handler


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
################## CV ENDS #############################

class NavTest():
    def __init__(self):
        rospy.init_node('nav_test', anonymous=True)
        hm.HelloNode.__init__(self)
        
        rospy.on_shutdown(self.shutdown)
        
        # How long in seconds should the robot pause at each location?
        self.rest_time = rospy.get_param("~rest_time", 5)
        
        # Are we running in the fake simulator?
        self.fake_test = rospy.get_param("~fake_test", False)
        
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
        # locations = dict()

        # import Locations from the .json file

        f = open("waypoints.json")
        location_data = json.load(f)

        locations = []
        locations_name = []

        # saving the locations in a list
        for loc in location_data.keys():
            locations.append([Pose(Point(location_data[loc]["Pose"]), Quaternion(location_data[loc]["Orientation"]))])
            locations_name.append(location_data[loc]["Name"])


        
        # locations['hall_foyer'] = Pose(Point(0.643, 4.720, 0.000), Quaternion(0.000, 0.000, 0.223, 0.975))
        # locations['hall_kitchen'] = Pose(Point(-1.994, 4.382, 0.000), Quaternion(0.000, 0.000, -0.670, 0.743))
        # locations['hall_bedroom'] = Pose(Point(-3.719, 4.401, 0.000), Quaternion(0.000, 0.000, 0.733, 0.680))
        # locations['living_room_1'] = Pose(Point(0.720, 2.229, 0.000), Quaternion(0.000, 0.000, 0.786, 0.618))
        # locations['living_room_2'] = Pose(Point(1.471, 1.007, 0.000), Quaternion(0.000, 0.000, 0.480, 0.877))
        # locations['dining_room_1'] = Pose(Point(-0.861, -0.019, 0.000), Quaternion(0.000, 0.000, 0.892, -0.451))

        
        
        # locations_np = np.array([[0.126, -0.249],[0.701, 0.224],[0.737, -1.851],[0.921, -3.645]])

        #####################################

        # calling a sorted index list from some function

        # path_finder = Path_finder('keys',locations_np,[1,2,3])
        
        # loc_list=path_finder.solve_tsp()

        # loc_list = probabalistic_things()
        #loc_list = [0,1,2]

        #####################################
        
        # Publisher to manually control the robot (e.g. to stop it)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)
        
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
        running_time = 0
        location = ""
        last_location = ""
        object_found = False
        
        # Get the initial pose from the user
        rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
        rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
        self.last_location = Pose()
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)
        
        # Make sure we have the initial pose
        while initial_pose.header.stamp == "":
            rospy.sleep(1)
            
        rospy.loginfo("Starting navigation test")
        
        # Begin the main loop and run through a sequence of locations
        while not rospy.is_shutdown():

            # # If we've gone through the current sequence,
            # # start with a new random sequence
            # if i == n_locations:
            #     i = 0
            #     sequence = sample(locations, n_locations)
            #     # Skip over first location if it is the same as
            #     # the last location
            #     if sequence[0] == last_location:
            #         i = 1

            for loc_idx in range(len(locations)):
            # Get the next location in the current sequence
                location = locations[loc_idx]
                            
                # Keep track of the distance traveled.
                # Use updated initial pose if available.
                # if initial_pose.header.stamp == "":
                #     distance = sqrt(pow(location.position.x - 
                #                         locations[last_location].position.x, 2) +
                #                     pow(locations[location].position.y - 
                #                         locations[last_location].position.y, 2))
                # else:
                #     rospy.loginfo("Updating current pose.")
                #     distance = sqrt(pow(locations[location].position.x - 
                #                         initial_pose.pose.pose.position.x, 2) +
                #                     pow(locations[location].position.y - 
                #                         initial_pose.pose.pose.position.y, 2))
                #     initial_pose.header.stamp = ""
                
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

                        self.get_image(loc_idx)
                        
                        #################################
                        n_successes += 1
                        distance_traveled += distance
                        rospy.loginfo("State:" + str(state))
                        self.rotate_cam()

                        ########################################################### Calling Perception and interaction things
                        #           Interaction as subpart of perception
                        #object_found = main_cv_arm()

                        #if object_found == True:
                        #    print("OBJECT FOUND!!!!")
                        #    self.shutdown()



                        ###########################################################
                    else:
                      rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))
                
                # How long have we been running?
                running_time = rospy.Time.now() - start_time
                running_time = running_time.secs / 60.0
                
                # Print a summary success/failure, distance traveled and time elapsed
                rospy.loginfo("Success so far: " + str(n_successes) + "/" + 
                              str(n_goals) + " = " + 
                              str(100 * n_successes/n_goals) + "%")
                rospy.loginfo("Running time: " + str(trunc(running_time, 1)) + 
                              " min Distance: " + str(trunc(distance_traveled, 1)) + " m")
                rospy.sleep(self.rest_time)
    
    ###### CV FUNCTION ##############
    def get_image(self,waypoint):   
        img_counter = 0
        timestamp_array = [None] * 6

        if waypoint == 0:
            directory = "/home/hello-robot/yolov7/images_demo/waypoint1"
        elif waypoint==1:
            directory = "/home/hello-robot/yolov7/images_demo/waypoint2"
        elif waypoint==2:
            directory = "/home/hello-robot/yolov7/images_demo/waypoint3"
                
        while(img_counter < 5):
      
            # Wait for frames
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data())
            frame = cv2.resize(color_image,(500,500))
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
            img_counter = img_counter + 1

            # Save data
            
            os.chdir(directory)
            filename = str(img_counter) + ".png"
            cv2.imwrite(filename,frame)
            print('image saved',img_counter)

            # Get time
            timestamp_array[img_counter]= datetime.datetime.now()

        # Destroy all the windows
        pipeline.stop()
        cv2.destroyAllWindows()

    ###### CV FUNC ENDS ################

    def update_initial_pose(self, initial_pose):
        self.initial_pose = initial_pose

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        #rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

    def rotate_cam(self):
        robot=stretch_body.robot.Robot()
        robot.head.move_by('head_tilt',-0.52)
        sleep(1.5)
        robot.head.move_by('head_pan',radians(45))
        sleep(1.5)
        robot.head.move_by('head_pan',radians(0))
        sleep(1.5)
        robot.head.move_by('head_pan',radians(-45))
        sleep(1.5)
        robot.head.move_by('head_pan',radians(-90))
        sleep(1.5)
        robot.head.move_by('head_pan',radians(-135))
        sleep(1.5)
        robot.head.move_by('head_pan',radians(-180))
        sleep(1.5)

def trunc(f, n):
    # Truncates/pads a float f to n decimal places without rounding
    slen = len('%.*f' % (n, f))
    return float(str(f)[:slen])

if __name__ == '__main__':
    try:
        NavTest()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL navigation test finished.")

