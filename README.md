# find_objects-ros_package

Repository for the ROS package to find misplaced objects using Stretch RE2 from Hello Robot.

## How to use the `state_machine_V3.py` node:

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

This documentation provides clear instructions on how to use the `state_machine_v3.py` node and run the necessary commands to set up the environment correctly.

## How to set up new waypoints

Open the waypoints_info.json and set the amount of waypoints you want, provide both position and orientation coordinates which can be obtain from the move_base/goal topic when using the nav-to-goal functionality in Rviz.
