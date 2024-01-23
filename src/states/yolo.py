#!/usr/bin/env python3

import rospy
import smach
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
from yolov4 import Detector
import tf2_ros
from geometry_msgs.msg import Point
from std_msgs.msg import String


class YoloHandling(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted', 'unready', 'completed'],
                             input_keys=['room_name', 'duration', 'rules_broken_count', 'd_duration', 'feedback'],
                             output_keys=['rules_broken_count', 'room_name', 'feedback'])
        self.cv_image = None
        self.bridge = CvBridge()
        self.cam_subs = rospy.Subscriber("/camera/image", Image, self.img_callback)
        self.detector = Detector(gpu_id=0, config_path='/opt/darknet/cfg/yolov4.cfg',
                                 weights_path='/opt/darknet/yolov4.weights',
                                 lib_darknet_path='/opt/darknet/libdarknet.so',
                                 meta_path='/home/k22003847/ros_ws/src/second_coursework/cfg/coco.data')
        self.tfb = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfb)
        self.tts_pub = rospy.Publisher('/speech', String, queue_size=10)
        self.next_room_dict = {'A': 'B',
                          'B': 'D',
                          'D': 'A'}


    def img_callback(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def get_detections(self):
        if self.cv_image is not None:
            img_arr = cv2.resize(self.cv_image, (self.detector.network_width(), self.detector.network_height()))
            cv_height, cv_width, _ = self.cv_image.shape
            detections = self.detector.perform_detect(image_path_or_buf=img_arr, show_image=True)
        return detections

    def execute(self, userdata):
        room_name = userdata.room_name
        duration = userdata.duration if room_name != 'D' else userdata.d_duration
        rules_broken_count = userdata.rules_broken_count
        dog = False
        cat = False
        person = False
        detections = []

        #duration controlled by navigator
        while not self.preempt_requested():
            detections = self.get_detections()
            for detection in detections:
                if detection.class_name == 'dog':
                    dog = True
                elif detection.class_name == 'cat':
                    cat = True
                elif detection.class_name == 'person':
                    person = True
            if dog and cat and room_name != 'D':
                transformation = self.tfb.lookup_transform('map', 'base_link', rospy.Time(0))
                current_point = Point(transformation.transform.translation.x, transformation.transform.translation.y,
                                      transformation.transform.translation.z)
                rules_broken_count[1] += 1
                rospy.loginfo("Rule 2 broken, dog and cat in same room.")
                self.tts_pub.publish("Could the cat or the dog get out of the room.")
                userdata.feedback.append([2, current_point])
                dog = False
                cat = False
            if person and room_name == 'D':
                transformation = self.tfb.lookup_transform('map', 'base_link', rospy.Time(0))
                current_point = Point(transformation.transform.translation.x, transformation.transform.translation.y,
                                      transformation.transform.translation.z)
                rules_broken_count[0] += 1
                person = False
                rospy.loginfo("Rule 1 broken, human in room D.")
                self.tts_pub.publish("Could the human get out of the room.")
                userdata.feedback.append([1, current_point])
        self.service_preempt()
        userdata.rules_broken_count = rules_broken_count
        userdata.room_name = self.next_room_dict[room_name]

        return 'succeeded'