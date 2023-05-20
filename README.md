# find_objects-ros_package

Repository for the ROS package to find misplaced objects using Stretch RE2 from Hello Robot.

## How to set up this project
* Save this repository inside under the src directory in the catkin_ws folder of Stretch since it is a ROS pkg.

* Make sure the telescopic arm is stowed by running `stretch_robot_stow.py`before using the navigation capabiliites of this pkg.


## How to set up new waypoints

Considering that you have already mapped your environment, open the json_handler_class.py and set the waypoint's position and orientation in the init() function inside the class. Both position and orientation coordinates can be obtain from the move_base/goal topic when using the nav-to-goal functionality in Rviz.

## How to use the `patrol_v3.py` node:
Make sure you have set up the desired waypoints in the previous step.Then make sure you have installed the yolov7 directory in the home directory of Stretch. Follow these steps to use the node:
1. Start the `navstack` launch file with the desired map using:
   ```
   roslaunch stretch_navigation navigation.launch map_yaml:=path_to_your_map.yaml
   ```
2. Run the `patrol_v3.py` file and the robot should start navigating to the waypoints to take photos. After the navigation is done, the robot will pass the recently taken images to the object classifier and it will update the JSON file with the newest information.
3. You can review the new images in the yolov7 directory inside the images_demo and images_results folders. 

## How to use the `state_machine_V3.py` node:

Before using this file make use you have calibrated the telescopic by running `stretch_robot_home.py`.

This node enables Stretch to interact with a user. The user can ask Stretch about a misplaced object, and the node will take care of locating any pictures containing the desired object in its database. If the object is found, a GUI will pop up showing the pictures where the object appears. Then the user can select an image, and Stretch will ask the user if they want to be taken to where the picture was taken. If the answer is yes, the robot will navigate to the picture's saved location. However, if the answer is no, the robot will tell the user where the photo was taken.

To run this node, please follow these instructions:

1. Start the `respeaker` launch file with:
   ```
   roslaunch respeaker_ros respeaker.launch
   ```

2. Run the `speech_to_text.py` node inside the script folder in the `respeaker` ROS package:
   ```
   python3 speech_to_text.py
   ```

3. Start the `navstack` launch file with the desired map using:
   ```
   roslaunch stretch_navigation navigation.launch map_yaml:=path_to_your_map.yaml
   ```

4. Finally, run the state machine node inside the script folder of this repository with:
   ```
   python3 state_machine_V3.py
   ```

Make sure to replace `path_to_your_map.yaml` with the actual path to your map YAML file.

## Important Notes and Limitations

* The Robot will try to move its telescopic arm to point at the location where the photo was taken if the user wants to be guided there. This feature is a very simple implementation and it does not include any object detection to point at the object if it is found. Future work could be focused in improving this feature with vision controlled grasping.