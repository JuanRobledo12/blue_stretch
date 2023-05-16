#!/usr/bin/env python3
import os
import playsound

def cv():

    import pyrealsense2 as rs
    import numpy as np
    import cv2
    import matplotlib.pyplot as plt
    import time
    import math
    import stretch_body.robot

    robot=stretch_body.robot.Robot()
    robot.startup()

    #camera head: find the pan and tilt values 
    print("camera pan =",robot.head.status['head_pan']['pos'])
    print("camera tilt =",robot.head.status['head_tilt']['pos'])
    #parameters
    pan = robot.head.status['head_pan']['pos']
    tilt = robot.head.status['head_tilt']['pos']

    robot.head.move_to('head_pan',math.radians(30))
    robot.head.move_to('head_tilt',math.radians(-30))


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
        while True:
            if math.degrees(robot.head.status['head_pan']['pos']) < -180:
                return_flag = False
                break
        

            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue
            
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
                pan = robot.head.status['head_pan']['pos']
                print(horizontal_error)
                print('uncenteredh', pan) 
                if abs(horizontal_error) > 2.0:
                    horizontal_input = -math.radians(horizontal_error) + pan
                    robot.head.move_to('head_pan',horizontal_input)
                    #time.sleep(3)
                    print(horizontal_input)
                print('centeredh', robot.head.status['head_pan']['pos'])       

                vertical_error = ((69 * y)/640) - 34.5
                tilt = robot.head.status['head_tilt']['pos']
                print('uncenteredv', tilt) 
                if abs(vertical_error) > 2.0:
                    vertical_input = -math.radians(vertical_error) + tilt
                    robot.head.move_to('head_tilt',vertical_input)
                    #time.sleep(3)
                    print(vertical_input)                  
                print('centeredv', robot.head.status['head_tilt']['pos']) 

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
                rotate1 = -0.025
                print('hey')
                robot.head.move_by('head_pan',rotate1)
    finally:

        # Stop streaming
        pipeline.stop()
        # Destroy all the windows
        #cv2.destroyAllWindows()
        # Stop robot
        robot.stop()
        if return_flag:
            return True, float(dist)
        else:
            return False, 0.00


def arm_point(depth):
    import time
    import numpy as np
    import stretch_body.robot
    from math import radians, degrees, atan2, sin, cos
    wrapToPi = lambda theta_rad: atan2(sin(theta_rad), cos(theta_rad))

    robot=stretch_body.robot.Robot()
    robot.startup()

    #camera head: pull the pan and tilt values from robot
    print("camera pan =",robot.head.status['head_pan']['pos'])
    print("camera tilt =",robot.head.status['head_tilt']['pos'])
    #parameters
    pan = robot.head.status['head_pan']['pos'] + radians(90)
    tilt = robot.head.status['head_tilt']['pos']
    depth_cam = depth
    print("*********************recieved intel ***************************")
    print(depth_cam)

    #####JUST FOR THE TESTING#####
    #tilt = radians(-35) # + is tilt up, - is tilt down
    #pan = radians(0)+radians(90) # + is CCW, - is CW   # the latter is to rotate 90 degree CCW so the arm matches the object direction
    #depth_cam = 1.00
    #####JUST FOR THE TESTING#####

    # Dimensions params
    h_cam = 1.3
    l_cam = 0.14
    h_grip = 0.173
    h_tolerance = 0.05
    l_grip = 0.216
    l_tolerance = 0.08
    #camera head move
    robot.head.move_to('head_pan',-radians(90))  #Zero the camera to the same dir as the arm             
    #robot.head.move_to('head_tilt',tilt) #Zero the camera to the same dir as the arm    

    #####JUST FOR THE TESTING#####
    #####JUST FOR THE TESTING#####
    #####JUST FOR THE TESTING#####

    def rotate_robot(rotate_angle):
        #turn the robot
        turn_rad = rotate_angle            #degrees of rotation
        turn_rads = radians(20)             #velocit? not sure. increasing it makes robot rotate faster
        turn_radss = radians(40)            #no idea what this does
        robot.base.rotate_by(turn_rad, v_r=turn_rads, a_r=turn_radss) 
        robot.push_command()
        time.sleep(abs(turn_rad/turn_rads))     #amt of time for robot to turn..value must be positive
        time.sleep(1)           #additional pause? idk why or if it is needed

        #get amount that robot rotated
        robot_status = robot.get_status()
        rotPos = robot_status['base']['theta']
        print("Stretch rotated: %6.1f deg" % degrees(wrapToPi(rotPos)))

    def lift_arm(l_command):
        robot.lift.move_to(l_command)                               #value is meters from the top of the base
        robot.push_command()
        robot.lift.wait_until_at_setpoint()
        print("lift = ",robot.lift.status['pos'])                     #says height of arm

    def extend_arm(ext_command):
        robot.arm.move_to(ext_command)                                          #value is how many meters from edge of base
        robot.push_command()
        robot.arm.wait_until_at_setpoint()
        print("extension =", robot.arm.status['pos'])                     #says how far arm is extended

    def rotate_wrist(wrist_command):
        wrist_yaw_vel = radians(140)
        wrist_yaw_accel = radians(180)

        #****************RECHECK******************

        robot.end_of_arm.move_to('wrist_yaw', wrist_command , wrist_yaw_vel, wrist_yaw_accel)                                        
        robot.push_command()
        time.sleep(3) 
        print("Wrist =", robot.end_of_arm.status['wrist_yaw']['pos'])


    lift_arm_real = h_cam + h_grip + h_tolerance - depth_cam*np.sin(abs(tilt))
    extend_arm_real = depth_cam*np.cos(abs(tilt)) + l_cam - l_grip - l_tolerance

    lift_arm_command = lift_arm_real - 0.19
    extend_arm_command = extend_arm_real - 0.28

    #stow arm  and wrist before moving
    extend_arm(0)
    rotate_wrist(3.14)

    rotate_robot(pan)
    #rotate_robot(radians(-90)) # JUST USE TO ROTATE BACK IN TESTING

    lift_arm(lift_arm_command)
    extend_arm(extend_arm_command)
    rotate_wrist(0)


    robot.stop()

def stow_arm():
    import time
    import numpy as np
    import stretch_body.robot
    from math import radians, degrees, atan2, sin, cos

    robot=stretch_body.robot.Robot()
    robot.startup()

    def extend_arm(ext_command):
        robot.arm.move_to(ext_command)                                          #value is how many meters from edge of base
        robot.push_command()
        robot.arm.wait_until_at_setpoint()
        print("extension =", robot.arm.status['pos'])                     #says how far arm is extended

    def rotate_wrist(wrist_command):
        wrist_yaw_vel = radians(140)
        wrist_yaw_accel = radians(180)

        #****************RECHECK******************

        robot.end_of_arm.move_to('wrist_yaw', wrist_command , wrist_yaw_vel, wrist_yaw_accel)                                        
        robot.push_command()
        time.sleep(3) 
        print("Wrist =", robot.end_of_arm.status['wrist_yaw']['pos'])

    extend_arm(0)
    rotate_wrist(3.14)

    robot.stop()

def play_audio_found():
    
    print('playing found sound')
    playsound.playsound("/home/hello-robot/Downloads/found_it.mp3",True)
    
def play_audio_not_found():
    
    print('playing not found sound')
    playsound.playsound("/home/hello-robot/Downloads/not_here.mp3",True)
    
def main_cv_arm():
    stow_arm()
    found, depth = cv()
    if found: 
        arm_point(depth)
        play_audio_found()
        return True
    else:
        play_audio_not_found()
        return False


main_cv_arm()
