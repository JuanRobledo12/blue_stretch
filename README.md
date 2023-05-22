# blue_stretch

Repository for the ROS package designed to assist in locating misplaced objects using the Stretch RE2 robot from Hello Robot.

## Team Blue Members

* Juan Antonio Robledo Lara
* Oluwatofunmi Sodimu
* Thanapol Tantagunninat
* Kanishk
* Nikhil Chittaluru
* Erin Kelly
* Daniel Lewis

## About

This project was developed by Team Blue during the [BMED 4833 ROB & BMED 8813 ROB](https://sites.gatech.edu/robotic-caregivers/) course at Georgia Tech. The aim of this system is to provide assistance to older adults living with mild cognitive impairment (MCI) in finding misplaced objects. A more detailed explanation of this problem can be found in the extension of this project carried out during CS 7633 Human-Robot Interaction at Georgia Tech. Please refer to the [HRI Final Project Report](HRI_Final_Project_Report.pdf) for further information. Additionally, we invite you to click in the image below to watch our demo video showcasing the functionality of the system:

[![Blue Stretch Demo Video](https://img.youtube.com/vi/ELt5CJxZVqI/0.jpg)](https://youtu.be/ELt5CJxZVqI)

## Project Setup

To set up this project, follow the steps below:

1. Save this repository inside the `src` directory of your catkin workspace (`catkin_ws`) as it is a ROS package.
2. Ensure that the telescopic arm is stowed by running `stretch_robot_stow.py` before utilizing the navigation capabilities of this package.
3. The navigation capabilities of this package rely on ROS navstack. Therefore, it is necessary to map the environment first. Please refer to the hello robot [tutorial](https://docs.hello-robot.com/0.2/stretch-tutorials/ros1/navigation_stack/) for mapping with Stretch.
4. The system utilizes the [respeaker_ros](https://github.com/furushchev/respeaker_ros) package for speech recognition capabilities. Make sure you have this package installed in your `catkin_ws`.
5. The system uses [YOLO V7](https://github.com/WongKinYiu/yolov7) for object classification in images. Ensure that you have it in Stretch's home directory. You can download the modified version we used from [here](https://drive.google.com/file/d/1VAxxNwaTtaYlC2AeoEVJbUbNwdnbNnHw/view).

## Setting up New Waypoints

Assuming you have already mapped your environment, open `json_handler_class.py` and set the position and orientation of the desired waypoints in the `init()` function inside the class. You can obtain both position and orientation coordinates from the `move_base/goal` topic when utilizing the nav-to-goal functionality in Rviz.

## Using the `patrol_v3.py` Node

Ensure that you have set up the desired waypoints as mentioned in the previous step. Additionally, make sure you have installed the `yolov7` directory in the home directory of Stretch. Follow the steps below to use the node:

1. Launch the `navstack` with the desired map by executing the following command:
   ```
   roslaunch stretch_navigation navigation.launch map_yaml:=path_to_your_map.yaml
   ```
   Make sure to replace `path_to_your_map.yaml` with the actual path to your map YAML file.

2. Run the `patrol_v3.py` file, and the robot will begin navigating to the waypoints to capture photos. Once the navigation is complete, the robot will pass the recently taken images to the object classifier, updating the JSON file with the latest information.

3. You can review the new images in the `yolov7` directory within the `images_demo` and `images_results` folders.

## Using the `state_machine_V3.py` Node

Before using the `state_machine_V3.py` file, ensure that you have calibrated the telescopic arm by running `stretch_robot_home.py`.

This node enables Stretch to interact with a user. The user can inquire about a misplaced object, and the node will locate any pictures in its database containing the desired object. If the object is found, a GUI will display the pictures where the object appears. The user can then select an image, and Stretch will ask if they want to be taken to where the picture was taken. If the answer is yes, the robot will navigate to the saved location. However, if the answer is no, the robot will provide the user with the information about where the photo was taken.

To run this node, please follow these instructions:

1. Launch the `respeaker` file using the command:
   ```
   roslaunch respeaker_ros respeaker.launch
   ```

2. Run the `speech_to_text.py` node located in the `script` folder of the `respeaker` ROS package:
   ```
   python3 speech_to_text.py
   ```

3. Launch the `navstack` with the desired map by executing the following command:
   ```
   roslaunch stretch_navigation navigation.launch map_yaml:=path_to_your_map.yaml
   ```
   Make sure to replace `path_to_your_map.yaml` with the actual path to your map YAML file.

4. Finally, run the state machine node located in the `script` folder of this repository using the command:
   ```
   python3 state_machine_V3.py
   ```

## Important Notes and Limitations

* The robot will attempt to move its telescopic arm to point at the location where the photo was taken if the user wants to be guided there. However, this feature is a simple implementation and does not include object detection to point specifically at the object if found. Future work could focus on improving this feature with vision-controlled grasping.
* The CS 7633 Final Project Report is part of a class project, and the results should not be considered as formal research.
* The patrol node overwrites the JSON file every time it is executed, so it needs to be modified if multiple runs are going to be performed in the same environment.
