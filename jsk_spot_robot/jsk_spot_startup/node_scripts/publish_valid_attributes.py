#!/usr/bin/env python

import rospy
from jsk_recognition_msgs.msg import ClassificationResult


rospy.init_node('publihs_valid_attributes')

def callback(msg):
    # print(msg)
    if len(msg.probabilities) > 0:
        pub.publish(msg)

pub = rospy.Publisher('/aws_detect_faces/attributes/valid', ClassificationResult,queue_size=1)
sub = rospy.Subscriber('/aws_detect_faces/attributes', ClassificationResult, callback)
rospy.spin()
