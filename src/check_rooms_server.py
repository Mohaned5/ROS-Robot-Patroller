#!/usr/bin/env python3

import rospy
import smach
import smach_ros
import time
import random
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from smach_ros import ServiceState
from second_coursework.srv import MoveRobot, MoveRobotRequest
import actionlib
from second_coursework.msg import CheckRoomsAction, CheckRoomsFeedback, CheckRoomsResult
from smach import CBState
import tf2_ros
from geometry_msgs.msg import Point
from std_msgs.msg import String
from states.yolo import YoloHandling


#controls the number of checks
@smach.cb_interface(outcomes=['succeeded', 'preempted', 'aborted', 'unready', 'completed'],
                    input_keys=['num_checks', 'count'],
                    output_keys=['count'])
def count_cb(userdata):
    if userdata.count < userdata.num_checks:
        userdata.count += 1
        rospy.loginfo(f"COUNT: {userdata.count}, NUM_CHECKS = {userdata.num_checks}")
        return 'succeeded'
    else:
        return 'completed'


@smach.cb_interface(outcomes=['succeeded', 'preempted', 'aborted', 'unready', 'completed'],
                    input_keys=['feedback'],
                    output_keys=['feedback'])
def feedback(userdata):
    for feedback in userdata.feedback:
        data = CheckRoomsFeedback()
        data.rule_broken = feedback[0]
        data.robot_position = feedback[1]
        server.publish_feedback(data)
        rospy.loginfo(f"Published Feedback {data}")
    userdata.feedback.clear()
    return 'succeeded'


@smach.cb_interface(outcomes=['succeeded', 'preempted', 'aborted', 'unready', 'completed'],
                    input_keys=['room_name', 'duration', 'd_duration'])
def navigator(userdata):
    move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    move_base_client.wait_for_server()

    room_name = userdata.room_name
    duration = userdata.duration if room_name != 'D' else userdata.d_duration
    start_time = time.time()

    room_bounds = {'A': ((0.64, 6.6), (3.2, 10.1)), 'B': ((4.4, 6.6), (7.8, 10.1)),
                   'D': ((0.64, 0.6), (3.2, 5.4))}

    #randomly chooses a point and moves for 2 seconds before picking another point
    #always moving
    while time.time() - start_time < duration:
        new_x = random.uniform(room_bounds[room_name][0][0], room_bounds[room_name][1][0])
        new_y = random.uniform(room_bounds[room_name][0][1], room_bounds[room_name][1][1])
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = new_x
        goal.target_pose.pose.position.y = new_y
        goal.target_pose.pose.orientation.w = 1.0

        move_base_client.send_goal(goal)
        move_base_client.wait_for_result(timeout=rospy.Duration(2))

        rospy.sleep(0.1)

    return 'succeeded'


def execute(goal):
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted', 'unready', 'completed'])
    sm.userdata['room_name'] = 'A'
    sm.userdata['num_checks'] = goal.num_checks
    sm.userdata['duration'] = 30
    sm.userdata['d_duration'] = 10
    sm.userdata['count'] = 0
    sm.userdata['rules_broken_count'] = [0, 0]
    sm.userdata['feedback'] = []

    def child_term_cb(outcome_map):
        if outcome_map['NAVIGATE']:
            return True
        return False

    with sm:
        smach.StateMachine.add('CHECK_COUNT', CBState(count_cb),
                               transitions={'succeeded': 'GO_TO_ROOM_A', 'preempted': 'preempted',
                                            'completed': 'completed'},
                               remapping={'count': 'count'})

        smach.StateMachine.add('GO_TO_ROOM_A', ServiceState('move_robot', MoveRobot,
                                                            request_slots=['room_name']),
                                transitions={'succeeded': 'NAVIGATE_A', 'aborted': 'GO_TO_ROOM_A', 'preempted': 'preempted'},
                                remapping = {'room_name':'room_name'})

        smach.StateMachine.add('GO_TO_ROOM_B', ServiceState('move_robot', MoveRobot,
                                                            request_slots=['room_name']),
                                transitions={'succeeded': 'NAVIGATE_B', 'aborted': 'GO_TO_ROOM_B', 'preempted': 'preempted'},
                                remapping = {'room_name':'room_name'})

        smach.StateMachine.add('GO_TO_ROOM_D', ServiceState('move_robot', MoveRobot,
                                                            request_slots=['room_name']),
                                transitions={'succeeded': 'NAVIGATE_D', 'aborted': 'GO_TO_ROOM_D', 'preempted': 'preempted'},
                                remapping = {'room_name':'room_name'})

        smach.StateMachine.add('PROCESS_FEEDBACK_A', CBState(feedback),
                               transitions={'succeeded': 'GO_TO_ROOM_B'})

        smach.StateMachine.add('PROCESS_FEEDBACK_B', CBState(feedback),
                               transitions={'succeeded': 'GO_TO_ROOM_D'})

        smach.StateMachine.add('PROCESS_FEEDBACK_D', CBState(feedback),
                               transitions={'succeeded': 'CHECK_COUNT'})

        concurrent_a = smach.Concurrence(outcomes=['succeeded', 'preempted', 'aborted', 'completed'],
                                        default_outcome='succeeded',
                                        input_keys = ['room_name', 'duration', 'rules_broken_count', 'd_duration','feedback'],
                                        output_keys = ['room_name', 'rules_broken_count','feedback'],
                                        child_termination_cb=child_term_cb)
        concurrent_b = smach.Concurrence(outcomes=['succeeded', 'preempted', 'aborted', 'completed'],
                                        default_outcome='succeeded',
                                        input_keys = ['room_name', 'duration', 'rules_broken_count', 'd_duration','feedback'],
                                        output_keys=['room_name', 'rules_broken_count','feedback'],
                                        child_termination_cb=child_term_cb)
        concurrent_d = smach.Concurrence(outcomes=['succeeded', 'preempted', 'aborted', 'completed'],
                                        default_outcome='succeeded',
                                        input_keys = ['room_name', 'duration', 'rules_broken_count', 'd_duration','feedback'],
                                        output_keys=['room_name', 'rules_broken_count','feedback'],
                                        child_termination_cb=child_term_cb)

        #NAVIGATE = movement around room
        #SCAN = YOLO scanning of videos
        with concurrent_a:
            smach.Concurrence.add('NAVIGATE', CBState(navigator),
                                  remapping={'room_name':'room_name'})
            smach.Concurrence.add('SCAN', YoloHandling(),
                                  remapping={'rules_broken_count':'rules_broken_count', 'room_name':'room_name', 'feedback':'feedback'})

        with concurrent_b:
            smach.Concurrence.add('NAVIGATE', CBState(navigator),
                                  remapping={'room_name':'room_name'})
            smach.Concurrence.add('SCAN', YoloHandling(),
                                  remapping={'rules_broken_count':'rules_broken_count', 'room_name':'room_name', 'feedback':'feedback'})

        with concurrent_d:
            smach.Concurrence.add('NAVIGATE', CBState(navigator),
                                  remapping={'room_name':'room_name'})
            smach.Concurrence.add('SCAN', YoloHandling(),
                                  remapping={'rules_broken_count':'rules_broken_count', 'room_name':'room_name', 'feedback':'feedback'})

        smach.StateMachine.add('NAVIGATE_A', concurrent_a,
                               transitions={'succeeded': 'PROCESS_FEEDBACK_A', 'preempted': 'GO_TO_ROOM_B'})

        smach.StateMachine.add('NAVIGATE_B', concurrent_b,
                               transitions={'succeeded': 'PROCESS_FEEDBACK_B', 'preempted': 'GO_TO_ROOM_D'})

        smach.StateMachine.add('NAVIGATE_D', concurrent_d,
                               transitions={'succeeded': 'PROCESS_FEEDBACK_D', 'preempted': 'CHECK_COUNT'})

    intro_server = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    intro_server.start()

    outcome = sm.execute()

    intro_server.stop()

    if outcome == 'completed':
        rospy.loginfo(f"Rules Broken: {sm.userdata['rules_broken_count']}")
        result = CheckRoomsResult()
        result.rules_broken_count = sm.userdata['rules_broken_count']
        server.set_succeeded(result)
    else:
        server.set_aborted()


rospy.init_node('check_rooms_server')
server = actionlib.SimpleActionServer('check_rooms', CheckRoomsAction, execute, auto_start=False)
server.start()
rospy.spin()
