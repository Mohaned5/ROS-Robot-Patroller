# ROS-Robot-Patroller

This repository contains the code for a TurtleBot3 service robot designed to enforce house rules during a party. The robot navigates within specific rooms (A, B, and D) to ensure rule compliance. The rules include restrictions on kitchen access and the simultaneous presence of the cat and dog in a room.

## Preview
https://www.youtube.com/watch?v=fJBNdK0D03M

## House Rules

  No people in the kitchen (room D) at any time.
  The cat and dog cannot be in the same room simultaneously.

## Components
1. ROS Service Implementation

The repository includes a ROS service that enables the robot to move to any location within rooms A, B, or D.

2. Actionlib Server

A node implements the actionlib server, executing the specified behavior based on the action specification. The action specification includes requests, feedback, and results, helping the robot adhere to house rules.

3. SMACH State Machine

The state machine orchestrates the robot's behavior, encompassing tasks such as room patrolling, navigation, rule checking, and rule enforcement using Text-to-Speech (TTS).

## State Machine States:
  Room Patrolling: Alternates between rooms A, B and D, checking them the specified number of times.
  Room Navigation (Concurrent): Spends 30 seconds navigating around each room while using YOLO for rule detection.
  Feedback Publication: Publishes action feedback when a rule is broken, providing the robot's position and the violated rule.
  Count Check: Checks the number of times each room has been patrolled.

## YOLO Detection

The YOLO (You Only Look Once) algorithm is integrated into the state machine to detect rule violations concurrently during room navigation. YOLO is employed on the camera feed to identify instances where house rules are compromised. The detected violations trigger the appropriate actions within the state machine, ensuring prompt rule enforcement.

## Main Node

The main node initiates the robot's behavior, calling the service to patrol a room and executing the actionlib server to enforce house rules. It prints the number of rule violations in the terminal.

## Launch File and Usage

A launch file, named "itr_cw.launch," is provided to start all necessary nodes for testing. It includes arguments for the number of checks and the video player node's video folder path.

Place the folder into your ROS Workspace.
1. Launch the empty stage plan:

       roslaunch rosplan_stage_demo empty_stage_single_robot.launch

2. Run RViZ view of stage demo (put your correct path):

       rosrun rviz rviz -d /opt/itr_ws/src/rosplan_stage_demo/config/rosplan_stage_demo.rviz

3. Launch the launch file (any number of checks):

       roslaunch second_coursework itr_cw.launch nchecks:=1

4. Run image view to see camera:

       rosrun rqt_image_view rqt_image_view

Feel free to explore, integrate, and enhance this repository for your robotic applications.
