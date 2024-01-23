#!/usr/bin/env python3

import rospy
import actionlib
from second_coursework.srv import MoveRobot, MoveRobotRequest
from second_coursework.msg import CheckRoomsAction, CheckRoomsGoal

rospy.init_node('main_node', anonymous = True)

rospy.wait_for_service('move_robot')

move_robot_proxy = rospy.ServiceProxy('move_robot', MoveRobot)
move = move_robot_proxy('A')
rospy.loginfo(f"Service called successfully. Response: {move}")


client = actionlib.SimpleActionClient('check_rooms', CheckRoomsAction)
client.wait_for_server()

goal = CheckRoomsGoal()
goal.num_checks = rospy.get_param('~nchecks', 3)
client.send_goal(goal)
client.wait_for_result()
result = client.get_result()
rospy.loginfo(f"Action Succeeded. Result: {result}")
if client.get_result():
    rospy.loginfo(f"Rule 1 broken {result.rules_broken_count[0]} times.")
    rospy.loginfo(f"Rule 2 broken {result.rules_broken_count[1]} times.")
