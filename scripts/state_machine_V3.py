#!/usr/bin/env python3

################## DESCRIPTION ##################

# This ROS node is allows a user to communicate with the robot using speech.
# The node is subscribed to another speech-to-text (saved in the respeaker_ros folder) node to receive the string containing what the user says.
# The string is parsed to find if it contains a keyword from a list of preset keywords.
# Then it can retreive the saved pictures containing the keyword as the label. Then show them in a GUI by calling the GUI class.
# Finally, after the picture is selected the robot can guide the user to where the photo was taken or it can tell the user where the object is.

################## Version Info ##################
# Last mod: 2023-May-15.
# This is the V3 with navigation, speech, and pointing capabilities.

import rospy
from enum import Enum, auto
from std_msgs.msg import String
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from gtts import gTTS
import os
import playsound

#GUI dependencies
from gui_class import ImageGallery #Run this in the directory where the gui is unless you have the global path of fake_gui

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
        
        # self.locations= [
        #             #Pose(Point(0.126, -0.249, 0.000), Quaternion(0.000, 0.000, 0.013, 0.999)), # origin \
        #             Pose(Point(0.701, 0.224, 0.000), Quaternion(0.000, 0.000, 0.716, 0.697)), # waypoint1 \
        #             Pose(Point(0.737, -1.851, 0.000), Quaternion(0.000, 0.000, 0.999, 0.014)), # waypoint2 \
        #             Pose(Point(0.921, -3.645, 0.000), Quaternion(0.000, 0.000, -0.702, 0.711)), # waypoint3 \
        #             ]
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
        self.text_sub = rospy.Subscriber('speech_to_text', SpeechRecognitionCandidates, self.speechText_callback)
        self.speechText_receiver = False
        self.user_msg = None
        self.object_list = ['cell phone', 'wallet', 'keys', 'phone', 'purse', 'glasses', 'dog', 'mouse', 'book', 'bottles']
        rospy.loginfo("Interaction setup done, starting main program...")
        

#### --------------- NAVIGATION FUNCTIONS -------------------- ####
    def update_initial_pose(self, initial_pose):
        self.initial_pose = initial_pose

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        #rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

#### --------------- INTERACTION FUNCTIONS -------------------- ####
    def speechText_callback(self, msg):
        rospy.loginfo('Message received: %s', msg.transcript)
        self.speechText_receiver = True
        self.user_msg = msg.transcript[0]
        print(type(self.user_msg))
    
    def find_keyword(self):
        for object in self.object_list:
                #index = self.user_msg.find(object)
                #if index != -1:
                if object in self.user_msg:
                    return object
                else:
                    continue
        return -1
    def determine_user_decision(self):

        if "yes" in self.user_msg:
            return 1
        elif "no" in self.user_msg:
            return -1
        else:
            return False
        

#### --------------- STATE MACHINE TRANSITION -------------------- ####
    def transition(self):

        #--------------- Identify the keyword in users Query --------------# 
        if self.machine_state == State.STATE_A and self.speechText_receiver:
            self.keyword = self.find_keyword()
            if self.keyword == -1:
                rospy.loginfo('Query does not contain keyword')
            else:
                rospy.loginfo('This is the keyword: %s', self.keyword)
                received_qry_msg = 'Sure! I can help you find your ' + self.keyword + ', please look at my screen to see if it is in the photos I took'
                print(received_qry_msg)
                tts = gTTS(text=received_qry_msg, lang='en')
                
                #This file is saved in the directory where you run the code
                tts.save('./stretch_audio_files/query_response.mp3')
                playsound.playsound('./stretch_audio_files/query_response.mp3', True)
                self.machine_state = State.STATE_B
        
        #------------------- Display the GUI ----------------------#
        elif self.machine_state == State.STATE_B:
            rospy.loginfo('State B')
            # Create an instance of the ImageGallery class
            gallery = ImageGallery(self.keyword)
            # Run the image gallery
            try:
                self.object_location, self.object_pose = gallery.run()
                print(self.object_location, self.object_pose)
                take_to_location_q = "Ok. Do you want me to take you to that photo's location?"
                tts = gTTS(text=take_to_location_q, lang='en')
                #This file is saved in the directory where you run the code
                tts.save('./stretch_audio_files/location_question.mp3')
                playsound.playsound('./stretch_audio_files/location_question.mp3', True)
                self.machine_state = State.STATE_C
            except:
                tts_1 = gTTS(text="I am sorry, I don't have photos of that object", lang='en')
                tts_1.save('./stretch_audio_files/no_object.mp3')
                playsound.playsound('./stretch_audio_files/no_object.mp3', True)
                self.machine_state = State.STATE_A
                self.speechText_receiver = False
            
        
        #------------------- Navigation to goal ----------------------#
        elif self.machine_state == State.STATE_C:
            user_decision = self.determine_user_decision()
            print('user_decision: ', user_decision)
            if user_decision == 1:
                tts = gTTS(text='Alright, let me take you there', lang='en')
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
                
                # # Print a summary success/failure, distance traveled and time elapsed
                # rospy.loginfo("Success so far: " + str(n_successes) + "/" + 
                #               str(n_goals) + " = " + 
                #               str(100 * n_successes/n_goals) + "%")
                # rospy.loginfo("Running time: " + str(trunc(running_time, 1)) + 
                #               " min Distance: " + str(trunc(distance_traveled, 1)) + " m")
                # rospy.sleep(self.rest_time)

                # self.shutdown()
                # print("NODE SHUTDOWNNNNN LOOOL!!!")

                # --------------- NAVIGATION LOOP ENDS ---------------- #
                # --------------- INDICATION LOOP STARTS ---------------- #

                #### Manipulation CODE!!!####
               
                # Rotate base CCW 90 degrees to align arm with waypoint
                rospy.loginfo('issuing BOSS ARM MANIPULATION command...')
                self.switch_base_to_manipulation = rospy.ServiceProxy('/switch_to_position_mode', Trigger)
                self.switch_base_to_manipulation() 

                rospy.loginfo('issuing BOSS ARM MANIPULATION - Rotate base command...')
                self.jointcontrol.rotate_base([1.57])
                                   
                rospy.loginfo('issuing BOSS ARM MANIPULATION - Move arm command...')
                # INPUT = [joint_wrist_yaw, head_pan, head_tilt, gripper_aperture, wrist_extension, joint_lift]
                self.jointcontrol.move_arm([0.0, -0.9, -0.9, 0.0, 0.05, 1.05])
                        

                #### ROTATE CAMERA CODE!!!####
                #self.rotate_cam()

                # --------------- INDICATION LOOP ENDS ---------------- #
                self.speechText_receiver = False
                self.machine_state = State.STATE_A
                tts = gTTS(text='Here is where I took the photo', lang='en')
                #This file is saved in the directory where you run the code
                tts.save('./stretch_audio_files/location_answer_3.mp3')
                playsound.playsound('./stretch_audio_files/location_answer_3.mp3', True)
                rospy.loginfo('State machine finished, waiting for next command')
            elif user_decision == -1:
                tts = gTTS(text="Alright, the object should be on the living room table .Let me know if there is anything else I can do for you", lang='en')
                #This file is saved in the directory where you run the code
                tts.save('./stretch_audio_files/location_answer_2.mp3')
                playsound.playsound('./stretch_audio_files/location_answer_2.mp3', True)
                
                self.speechText_receiver = False
                self.machine_state = State.STATE_A
                rospy.loginfo('State machine finished, waiting for next command')
                

    def run(self):
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            self.transition()
            state_msg = String()
            state_msg.data = str(self.machine_state)
            self.pub.publish(state_msg)
            rate.sleep()

    ####### MANIPULATION FUNC #################### 
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
    ###### MANIPULATION FUNC ENDS ################

def trunc(f, n):
    # Truncates/pads a float f to n decimal places without rounding
    slen = len('%.*f' % (n, f))
    return float(str(f)[:slen])


if __name__ == '__main__':
    rospy.init_node('state_machine', anonymous=True)
    state_machine = StateMachine()
    state_machine.run()
    