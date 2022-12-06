#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from jsk_recognition_msgs.msg import ClassificationResult
from dialogflow_task_executive.msg import DialogTextActionResult
from sound_play.libsoundplay import SoundClient
import time
import message_filters

from threading import Lock

class Emotion():
    def __init__(self):
        self.sound = SoundClient(sound_action='sound_play',blocking=True)
        
        #Publisher
        self.pub = rospy.Publisher('/emotion',String, queue_size=1)

        #Subscriber
        self.sub_face = rospy.Subscriber('/aws_detect_faces/attributes',ClassificationResult,callback=self.face_cb)
        self.sub_dif = rospy.Subscriber('/dialogflow_client/text_action/result', DialogTextActionResult, callback=self.dif_cb)
        ###TODO
        ##
        #self.sub_col = rospy.Subscriber()

        self.duration_time = rospy.Duration(30)
        self.prev_pose_detected_time = rospy.Time.now() - self.duration_time

        self.lock = Lock()

        queue_size = 10
        fps = 100.
        delay = 1 / fps* 0.5

    def pose_cb(self,msg):
        if (rospy.Timenow() -  self.prev_pose_detected_time) < self.duration_time:
            return
        poses = msg.poses
        if len(poses) >= 1:
            item = poses[0].limb_names
            scores = poses[0].scores
            target_limbs = [
                "left_eye", "nose", "right_eye", "right_ear",
                "left_ear"]
            if all([name in item for name in target_limbs]):
                message = "happy"
                rospy.loginfo("received {}, current emotion is {}".format(item, message))
                with self.lock:
                    self.publish(message)
                    self.prev_pose_detected_time = rospy.Time.now()
                    rospy.sleep(5.0)

    def dif_cb(self,data):
        rospy.loginfo(data)
        rospy.loginfo(data.result.response.action)
        self.publish(str(data.result.response.action))

    def face_cb(self,msg):
        emotions = {}
        probabilities = []
        print("===========")
        #print(msg)
        #for i in
        if len(msg.label_names) <= 11:
            return
        print("aaaa")
        for i in range(3,11):
            emotion = msg.label_names[i]
            probability = msg.probabilities[i]
            emotions[emotion]=probability
        print(emotions)
        print("max:{}".format(max(emotions,key=emotions.get)))
        print("label_names:{}".format(msg.label_names[13]))
        print("probabilities:{}".format(msg.probabilities[13]))


    def publish(self,msg):
        self.sound.actionclient.wait_for_server()
        #self.sound.playWave("/home/a-ichikura//spot_ws/src/jsk_robot/jsk_spot_robot/jsk_spot_startup/sound/output.wav",replace=False)
        print("Successfully moved")
        self.pub.publish(msg)
        


if __name__ == '__main__':
    rospy.init_node('emotion_reaction')

    time.sleep(3.0)
    node = Emotion()

    while not rospy.is_shutdown():
        rospy.sleep(0.1)
