#!/usr/bin/env python3

"""
Description:
This ROS node enables user-robot communication using speech. 
It subscribes to another speech-to-text node (saved in the respeaker_ros folder) to receive the string containing what the user says. 
The string is parsed to identify keywords from a preset list. It can retrieve saved pictures that contain the keyword as the label and display them in a GUI using the GUI class. 
Finally, the robot can guide the user to the location where the photo was taken or provide information about the object's whereabouts.

Version Info:
Last Modified: 2023-Jun-30
Version: V3 with navigation, speech, and pointing capabilities.

"""


import rospy
from enum import Enum, auto
from std_msgs.msg import String
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from gtts import gTTS
import playsound
import csv
import os


#GUI dependencies
from gui_class import ImageGallery 
from modern_csv_gui import Application

#Navigation dependencies
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import sample
from math import pow, sqrt

#Manipulation dependencies
from std_srvs.srv import Trigger, TriggerResponse
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import time
from move_arm_class import JointController

class State(Enum):
    STATE_A = auto()
    STATE_B = auto()
    STATE_C = auto()

class StateMachine:
    def __init__(self):

        # ---------------- Helper function for manipulation ----------
        self.jointcontrol = JointController()
        
        
        #----------------- Navgation variable ----------------#

        # How long in seconds should the robot pause at each location?
        self.rest_time = rospy.get_param("~rest_time", 5)
        
        # Are we running in the fake simulator?
        self.fake_test = rospy.get_param("~fake_test", False)
        
        # Goal state return values
        self.goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 
                       'SUCCEEDED', 'ABORTED', 'REJECTED',
                       'PREEMPTING', 'RECALLING', 'RECALLED',
                       'LOST']
        # Set up the goal locations. Poses are defined in the map frame.  
        # An easy way to find the pose coordinates is to point-and-click
        # Nav Goals in RViz when running in the simulator.
        # Pose coordinates are then displayed in the terminal
        # that was used to launch RViz.
        
        self.locations = []
        
        self.locations_to_visit = [0]

    
        
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
        self.initial_pose = PoseWithCovarianceStamped()
        
        # Variables to keep track of success rate, running time,
        # and distance traveled
        self.n_locations = len(self.locations)
        self.n_goals = 0
        self.n_successes = 0
        self.i = self.n_locations
        self.distance_traveled = 0
        self.start_time = rospy.Time.now()
        self.running_time = 0
        self.location = ""
        self.last_location = ""
        self.object_found = False
        
        # Get the initial pose from the user
        rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
        rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
        self.last_location = Pose()
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)
        
        # Make sure we have the initial pose
        while self.initial_pose.header.stamp == "":
            rospy.loginfo('no initial pose info yet...')
            rospy.sleep(1)
            
        rospy.loginfo("Navigation setup done! Starting Interaction setup...")
        self.machine_state = State.STATE_A
        self.pub = rospy.Publisher('state_topic', String, queue_size=10)
        
        #------------------- Speech Interaction ----------------#
        self.text_sub = rospy.Subscriber('speech_to_text', SpeechRecognitionCandidates, self.speechText_callback)
        self.speechText_receiver = False
        self.user_msg = None
        self.blue_speech_interaction_flag = False
        self.user_name = 'Tony'
        #--------------- Opening the items csv and turing it into a list -------------#
        self.csv_filepath = './stretch_misc_files/items.csv'
        self.object_list = self.create_items_list(self.csv_filepath)
        #self.object_list = ['books', 'book', 'chair', 'person', 'apple']
        rospy.loginfo("Speech Interface setup done, starting main program...")
        

#### --------------- NAVIGATION FUNCTIONS -------------------- ####
    def update_initial_pose(self, initial_pose):
        self.initial_pose = initial_pose

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        #rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

#### --------------- SPEECH INTERACTION FUNCTIONS -------------------- ####
    def speechText_callback(self, msg):
        rospy.loginfo('Message received: %s', msg.transcript)
        self.speechText_receiver = True
        self.user_msg = msg.transcript[0]
        print(type(self.user_msg))
    
    def create_items_list(self, filepath):
        item_list = []
        # Open the CSV file in read mode
        with open(filepath, 'r') as csvfile:
            reader = csv.reader(csvfile)

            # Skip the header row
            next(reader)

            # Read the items and add them to the list
            for row in reader:
                item_list.append(row[0])

        print("Created Item list:", item_list)
        return item_list
    
    def add_item_to_csv(self, item, filepath):
    # Open the CSV file in append mode
        with open(filepath, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)

            # Write the new item
            writer.writerow([item])

        print(f"Item '{item}' added to {filepath}!")


    def find_keyword(self):
        for object in self.object_list:
                if object in self.user_msg:
                    return object
                elif self.user_msg == 'add object':
                    return 'add object'
                elif self.user_msg == 'hey blue':
                    return 'hey blue'
        return -1
    def determine_user_decision(self):

        if "take me there" in self.user_msg:
            return 1
        elif "tell me location" in self.user_msg:
            return -1
        else:
            return False
    
#### --------------- Run FUNCTIONS -------------------- ####   
    def run(self):
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            self.transition()
            state_msg = String()
            state_msg.data = str(self.machine_state)
            self.pub.publish(state_msg)
            rate.sleep()

#### --------------- Manipulation FUNCTIONS -------------------- ####   
    def move_arm(self,endpoint): 
        point0 = JointTrajectoryPoint()
        point0.positions = [0.2, 0.0, 3.14,0.0,0.0]
        point0.velocities = [0.2, 0.2, 2.5, 1.0, 1.0]
        point0.accelerations = [1.0, 1.0, 3.5, 1.0, 1.0]

        point1 = JointTrajectoryPoint()
        point1.positions = endpoint

        trajectory_goal = FollowJointTrajectoryGoal()
        trajectory_goal.trajectory.joint_names = ['joint_lift', 'wrist_extension', 'joint_wrist_yaw', 'joint_head_pan', 'joint_head_tilt']
        trajectory_goal.trajectory.points = [point0, point1]
        trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
        trajectory_goal.trajectory.header.frame_id = 'base_link'

        self.trajectory_client.send_goal(trajectory_goal)
        rospy.loginfo('Sent list of goals = {0}'.format(trajectory_goal))
        self.trajectory_client.wait_for_result()

        time.sleep(1)  

#### --------------- STATE MACHINE TRANSITION -------------------- ####
    def transition(self):

        
        
        #--------------- Identify the keyword in users Query --------------# 
        if self.machine_state == State.STATE_A and self.speechText_receiver:
            
            

            #-------------- Getting the item keyword ---------------#
            self.keyword = self.find_keyword()
            if self.keyword == 'hey blue' and not self.blue_speech_interaction_flag:
                self.blue_speech_interaction_flag = True
                salute_msg = 'Hi, {}'.format(self.user_name)
                tts = gTTS(text=salute_msg, lang='en')
                tts.save('./stretch_audio_files/salute_msg.mp3')
                playsound.playsound('./stretch_audio_files/salute_msg.mp3', True)
            
            if self.blue_speech_interaction_flag:
                if self.keyword == -1:
                    rospy.loginfo('Query does not contain keyword')
                    repeat_queary_msg = 'Sorry, I couldn\'t understand your request. If you want me to keep track of a new object, please say "add object". I\'ll be happy to assist you with that.'
                    tts = gTTS(text=repeat_queary_msg, lang='en')
                    tts.save('./stretch_audio_files/no_query_message.mp3')
                    playsound.playsound('./stretch_audio_files/no_query_message.mp3', True)
                    self.speechText_receiver = False
                elif self.keyword == 'add object':
                    add_object_response_msg = 'Alright! Please look at my screen'
                    tts = gTTS(text=add_object_response_msg, lang='en')
                    tts.save('./stretch_audio_files/add_object_response.mp3')
                    playsound.playsound('./stretch_audio_files/add_object_response.mp3', True)
                    add_object_gui = Application()
                    add_object_gui.run()
                    self.object_list = self.create_items_list(self.csv_filepath) #updates the object list
                    self.blue_speech_interaction_flag = False
                    self.speechText_receiver = False

                elif self.keyword != 'hey blue' and self.keyword != -1 and self.keyword != 'add object':
                    rospy.loginfo('This is the keyword: %s', self.keyword)
                    received_qry_msg = 'Sure! Please wait a moment, I am searching through my database of photos to see if I can locate it for you'
                    tts = gTTS(text=received_qry_msg, lang='en')
                    #This file is saved in the directory where you run the code
                    tts.save('./stretch_audio_files/query_response.mp3')
                    playsound.playsound('./stretch_audio_files/query_response.mp3', True)
                    self.speechText_receiver = False
                    self.blue_speech_interaction_flag = False
                    self.machine_state = State.STATE_B
        
        #------------------- Display the GUI ----------------------#
        elif self.machine_state == State.STATE_B:
            rospy.loginfo('State B')
            keyword_object_inGallery_flag = False
            # Create an instance of the ImageGallery class
            gallery = ImageGallery(self.keyword)
            # Run the image gallery
            try:
                self.object_location, self.object_pose = gallery.run()
                keyword_object_inGallery_flag = True
                
            except:
                self.machine_state = State.STATE_A
            
            if keyword_object_inGallery_flag:
                print(self.object_location, self.object_pose)
                take_to_location_q = "You selected a photo. Say: \'take me there'\, if you want me to guide you or say: \'tell me location\', if you want to know where I took the picture"
                tts = gTTS(text=take_to_location_q, lang='en')
                #This file is saved in the directory where you run the code
                tts.save('./stretch_audio_files/location_question.mp3')
                playsound.playsound('./stretch_audio_files/location_question.mp3', True)
                self.machine_state = State.STATE_C

        
        #------------------- Navigation to goal ----------------------#
        elif self.machine_state == State.STATE_C:
            user_decision = self.determine_user_decision()
            print('user_decision: ', user_decision)
            if user_decision == 1:
                tts = gTTS(text='Alright, Please follow me', lang='en')
                #This file is saved in the directory where you run the code
                tts.save('./stretch_audio_files/location_answer_1.mp3')
                playsound.playsound('./stretch_audio_files/location_answer_1.mp3', True)

                # --------------- NAVIGATION LOOP STARTS ---------------- #
                self.object_pose= Pose(Point(
                                        self.object_pose["position"][0],
                                        self.object_pose["position"][1],
                                        self.object_pose["position"][2]
                                        ), 
                                        Quaternion(
                                            self.object_pose["orientation"][0],
                                            self.object_pose["orientation"][1],
                                            self.object_pose["orientation"][2],
                                            self.object_pose["orientation"][3]
                                        ))
                print(self.object_pose)
                self.locations.append(self.object_pose)
                for loc_idx in self.locations_to_visit:
                # Get the next location in the current sequence
                    self.location = self.locations[loc_idx]
                    
                    distance = 000

                    # Store the last location for distance calculations
                    self.last_location = self.location
                    
                    # Increment the counters
                    self.i += 1
                    self.n_goals += 1
                
                    # Set up the next goal location
                    self.goal = MoveBaseGoal()
                    self.goal.target_pose.pose = self.location
                    self.goal.target_pose.header.frame_id = 'map'
                    self.goal.target_pose.header.stamp = rospy.Time.now()
                    
                    # Let the user know where the robot is going next
                    rospy.loginfo("Going to: " + str(self.location))
                    
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
                            self.n_successes += 1
                            self.distance_traveled += distance
                            rospy.loginfo("State:" + str(state))

                        else:
                            rospy.loginfo("Goal failed with error code: " + str(self.goal_states[state]))
                    
                    # How long have we been running?
                    running_time = rospy.Time.now() - self.start_time
                    running_time = running_time.secs / 60.0
                    self.locations.clear()
                
                # --------------- NAVIGATION LOOP ENDS ---------------- #


                # --------------- INDICATION LOOP STARTS ---------------- #

                #### Manipulation CODE!!!####

                #Switching from navigation mode to position mode to move the robotic arm
                self.switch_base_to_manipulation = rospy.ServiceProxy('/switch_to_position_mode', Trigger)

                # Wait for the service to become available
                rospy.wait_for_service('/switch_to_position_mode')

                # Then call the service
                try:
                    self.switch_base_to_manipulation()
                except rospy.ServiceException as e:
                    print("ERROR: Switch to Position Mode Service call failed: %s"%e)

                # Rotate base CCW 90 degrees to align arm with waypoint
                rospy.loginfo('issuing BOSS ARM MANIPULATION command...')

                rospy.loginfo('issuing BOSS ARM MANIPULATION - Rotate base command...')
                self.jointcontrol.rotate_base([1.57])
                                   
                rospy.loginfo('issuing BOSS ARM MANIPULATION - Move arm command...')
                # INPUT = [joint_wrist_yaw, head_pan, head_tilt, gripper_aperture, wrist_extension, joint_lift]
                self.jointcontrol.move_arm([0.0, -0.9, -0.9, 0.0, 0.05, 1.05])
                   

                #### ROTATE CAMERA CODE!!!####
                #self.rotate_cam()

                # --------------- INDICATION LOOP ENDS ---------------- #
                self.machine_state = State.STATE_A
                tts = gTTS(text='Here is where I took the photo', lang='en')
                #This file is saved in the directory where you run the code
                tts.save('./stretch_audio_files/location_answer_3.mp3')
                playsound.playsound('./stretch_audio_files/location_answer_3.mp3', True)
                rospy.loginfo('State machine finished, waiting for next command')
                
                
                # Initialize the service proxy for returning to navigation mode
                self.switch_base_to_navigation = rospy.ServiceProxy('/switch_to_navigation_mode', Trigger)

                # Wait for the service to become available
                rospy.wait_for_service('/switch_to_navigation_mode')

                # Then call the service
                try:
                    self.switch_base_to_navigation()
                except rospy.ServiceException as e:
                    print("ERROR: Switch to Navigation Mode Service call failed: %s"%e)
                            
            elif user_decision == -1:
                tts = gTTS(text="Alright, the object should be on the " + self.object_location + ".", lang='en')
                
                
                #This file is saved in the directory where you run the code
                tts.save('./stretch_audio_files/location_answer_2.mp3')
                playsound.playsound('./stretch_audio_files/location_answer_2.mp3', True)
                
                self.machine_state = State.STATE_A
                rospy.loginfo('State machine finished, waiting for next command')
                

    

def trunc(f, n):
    # Truncates/pads a float f to n decimal places without rounding
    slen = len('%.*f' % (n, f))
    return float(str(f)[:slen])


if __name__ == '__main__':
    rospy.init_node('state_machine', anonymous=True)
    state_machine = StateMachine()
    state_machine.run()
    
