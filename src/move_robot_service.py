#!/usr/bin/env python3

import rospy
from second_coursework.srv import MoveRobot
from geometry_msgs.msg import Twist
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class MoveRobotService:
    def __init__(self):
        rospy.init_node('move_robot_service')
        self.service = rospy.Service('move_robot', MoveRobot, self.handle_move_room)
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


    def handle_move_room(self, request):
        room_name = request.room_name

        valid_rooms = {'A': (1.95, 7.4), 'B': (5.96, 7.4), 'D': (1.95, 2)}

        self.next_room_dict = {'A': 'B',
                               'B': 'D',
                               'D': 'A'}

        goal_x = valid_rooms[room_name][0]
        goal_y = valid_rooms[room_name][1]

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = goal_x
        goal.target_pose.pose.position.y = goal_y
        goal.target_pose.pose.orientation.w = 1.0

        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()
        return True


if __name__ == '__main__':
    try:
        move_robot_service = MoveRobotService()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
