#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from jsk_recognition_msgs.msg import ClassificationResult
from dialogflow_task_executive.msg import DialogTextActionResult
from spot_msgs.msg import PickObjectInImageActionResult
from cv_bridge import CvBridge, CvBridgeError
import time
import message_filters
import datetime
from threading import Lock
import os
import cv2
import requests
import json

class Emotion():
    def __init__(self):
        #Subscriber
        self.sub_face = rospy.Subscriber('/aws_detect_faces/attributes', ClassificationResult, callback=self.face_cb)
        self.sub_dif = rospy.Subscriber('/dialogflow_client/text_action/result', DialogTextActionResult, callback=self.dif_cb)
        self.sub_play = rospy.Subscriber("/spot/pick_object_in_image/action/result",PickObjectInImageActionResult,self.play_cb)
        self.sub_play = rospy.Subscriber("/spot/pick_object_in_image/action/goal",PickObjectInImageActionGoal,self.play_start_cb)
        
        ###TODO
        ##
        #self.sub_col = rospy.Subscriber()

        self.bridge = CvBridge()
        self.duration_time = rospy.Duration(10)
        self.prev_pose_detected_time = rospy.Time.now() - self.duration_time

        self.lock = Lock()
        self.gc_img_cnt = 0
        self.aws_img_cnt = 0
        self.google_chat_list = []

        self.key = '3bc0fc64-5b60-653f-0fb1-7b3370ae50a8:fx'
        self.source_lang = "JA"
        self.target_lang = "EN"

    def face_cb(self,msg):
        if (rospy.Time.now() - self.prev_pose_detected_time) < self.duration_time:
            return
        try:
            confidence = msg.probabilities[0]
        except IndexError:
            confidence = 0
        if confidence >= 80:
            emotions = {}
            probabilities = []
            print("===========")
            #CALM #ANGRY #SAD #HAPPY #FEAR #CONFUSED #SURPRISED #DISGUSTED
            #choose Emotion which has highest probability
            for i in range(3,11):
                emotion = msg.label_names[i]
                probability = msg.probabilities[i]
                emotions[emotion]=probability
            print(emotions)
            max_emo = max(emotions,key=emotions.get)
            emotion = max_emo.lower()
            print("max:{}".format(max_emo))
            print("label_names:{}".format(msg.label_names[13])) #smile
            print("probabilities:{}".format(msg.probabilities[13])) #True or False
            self.aws_face_memory(emotion,1)
        else:
            pass

    def dif_cb(self,data):
        emotion = str(data.result.response.action).lower()
        query = str(data.result.response.query)
        self.google_chat_memory(emotion,0,query)

    def play_cb(self,data):
        status = data
        self.ball_play(2,status)

    def play_start_cb(self,data):
        print("do something")
        self.pick_image_object_image = rospy.wait_for_message(rospy.get_param("~image","/spot/camera/hand_colo/image"),Image,timeout=None)

    def google_chat_memory(self,emotion,service_num,query):
        self.gc_img_cnt = self.gc_img_cnt + 1
        today = datetime.date.today()
        today = str(today)
        memory_path = "/home/a-ichikura/memory/" + today + "/google_chat_ros/"
        if not os.path.isdir(memory_path):
            os.makedirs(memory_path)
        msg = rospy.wait_for_message(rospy.get_param("~image","/spot/camera/hand_color/image"),Image,timeout=None)
        rospy.loginfo(emotion)

        #decide emotion
        if emotion == "happy" or emotion == "input.welcome":
            emo_num = 1
        elif emotion == "smirking" or emotion == "squinting":
            emo_num = 2
        elif emotion == "love":
            emo_num = 3
        elif emotion == "fearful" or emotion == "cry" or emotion == "sad":
            emo_num = 4
        elif emotion == "relived" or emotion == "calm":
            emo_num = 5
        elif emotion == "boring" or emotion == "unpleasant" or emotion == "disgusted":
            emo_num = 6
        elif emotion == "input.unknown" or emotion == "confused":
            emo_num = 7
        elif emotion == "angry" or emotion == "astonished" or emotion == "surprised":
            emo_num = 8
        else:
            emo_num = 9
        #decide service
        sev_num = service_num

        rospy.loginfo("save iamges... {}".format(type(msg)))
        try:
            cv2_img = self.bridge.imgmsg_to_cv2(msg,"bgr8")
        except CvBridgeError as e:
            rospy.logerr("ERROR!!!!!!!!!!!!!!!!!!")
            rospy.logerr(e)
        else:
            cv2.imwrite(memory_path + str(self.gc_img_cnt).zfill(4) + str(sev_num) + str(emo_num) + ".png", cv2_img)
            query = query
            translated = self.translate_message(query)
            list_ = ["service_num",self.gc_img_cnt,translated,emotion]
            self.google_chat_list.append(list_)
            all_dict = {"header": self.google_chat_list}
            with open(memory_path + "memory.json","w") as f:
                json.dump(all_dict,f,indent=2)
            rospy.loginfo("Save an image ! to " + memory_path)

    def ball_play_memory(self,service_num,status):
        self.gc_img_cnt = self.gc_img_cnt + 1
        today = datetime.date.today()
        today = str(today)
        memory_path = "/home/a-ichikura/memory/" + today + "/google_chat_ros/"
        if not os.path.isdir(memory_path):
            os.makedirs(memory_path)
        msg = rospy.wait_for_message(rospy.get_param("~image","/spot/camera/frontright/image"),Image,timeout=None)
        sev_num = service_num

        if status == "true":
            emo_num = 1
            emotion = "happy"
        else:
            emo_num  = 4
            emotion = "sad"
            
        rospy.loginfo("save iamges... {}".format(type(msg)))
        try:
            cv2_img = self.bridge.imgmsg_to_cv2(msg,"bgr8")
        except CvBridgeError as e:
            rospy.logerr("ERROR!!!!!!!!!!!!!!!!!!")
            rospy.logerr(e)
        else:
            cv2.imwrite(memory_path + "ball_play" + str(self.gc_img_cnt).zfill(4) + str(sev_num) + str(emo_num) + ".png", cv2_img)
            list_ = [service_num,self.gc_img_cnt,status,emotion]
            self.google_chat_list.append(list_)
            all_dict = {"header": self.google_chat_list}
            with open(memory_path + "memory.json","w") as f:
                json.dump(all_dict,f,indent=2)
            rospy.loginfo("Save an image ! to " + memory_path)

        

    def aws_face_memory(self,emotion,service_num):
        self.aws_img_cnt = self.aws_img_cnt + 1
        today = datetime.date.today()
        today = str(today)
        memory_path = "/home/a-ichikura/memory/" + today + "/aws_detect_face/"
        if not os.path.isdir(memory_path):
            os.makedirs(memory_path)
        msg = rospy.wait_for_message(rospy.get_param("~image","/spot/camera/hand_color/image"),Image,timeout=None)
        rospy.loginfo(emotion)

        #decide emotion
        if emotion == "happy" or emotion == "input.welcome":
            emo_num = 1
        elif emotion == "smirking" or emotion == "squinting":
            emo_num = 2
        elif emotion == "love":
            emo_num = 3
        elif emotion == "fearful" or emotion == "cry" or emotion == "sad":
            emo_num = 4
        elif emotion == "relived" or emotion == "calm":
            emo_num = 5
        elif emotion == "boring" or emotion == "unpleasant" or emotion == "disgusted":
            emo_num = 6
        elif emotion == "input.unknown" or emotion == "confused":
            emo_num = 7
        elif emotion == "angry" or emotion == "astonished" or emotion == "surprised":
            emo_num = 8
        else:
            emo_num = 9
        #decide service
        sev_num = service_num

        rospy.loginfo("save iamges... {}".format(type(msg)))
        try:
            cv2_img = self.bridge.imgmsg_to_cv2(msg,"bgr8")
        except CvBridgeError as e:
            rospy.logerr("ERROR!!!!!!!!!!!!!!!!!!")
            rospy.logerr(e)
        else:
            cv2.imwrite(memory_path + str(self.aws_img_cnt).zfill(4) + str(sev_num) + str(emo_num) + ".png", cv2_img)
            rospy.loginfo("Save an image ! to " + memory_path)

    def translate_message(self,word):
        params = {
            'auth_key':self.key,
            'text':word,
            'source_lang':self.source_lang,
            'target_lang':self.target_lang,
        }
        request = requests.post('https://api-free.deepl.com/v2/translate',data=params)
        result = request.json()
        deepl = result["translations"][0]["text"]
        return deepl

    
class No_emotion():
    def __init__(self):
        self.start = time.time()
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(rospy.get_param("~image","/spot/camera/hand_color/image"),Image,callback=self.make_simple_memory)
        self.q_ = 0
        self.img_cnt = 0
        self.no_ret_cnt = 0
        
    def make_simple_memory(self,msg):
        time_now = time.time()
        tim = int(time_now - self.start)
        q = tim // 30 #colect image every 300 minutes

        if not rospy.get_param("~image","/spot/camera/hand_color/image"):
            self.no_ret_cnt += 1
                
        if self.no_ret_cnt > 1000:
            print("there is no image message")
            rospy.is_shutdown()
                
        if rospy.get_param("~image","/spot/camera/hand_color/image") and self.q_ != q:
            self.img_cnt += 1
            today = datetime.date.today()
            today = str(today)
            #memory_path = "/home/ichikura/spot_ws/src/jsk-ros-pkg/jsk_robot/jsk_spot_robot/jsk_spot_startup/memory/" + today
            memory_path = "/home/ichikura/memory/" + today

            try:
                cv2_img = self.bridge.imgmsg_to_cv2(msg,"bgr8")
            except CvBridgeError as e:
                print(e)
            else:
                cv2.imwrite(memory_path + "/" + str(self.img_cnt).zfill(4) + ".png", cv2_img)
            self.q_ = q

    

if __name__ == '__main__':
    rospy.init_node('make_memory')
    today = datetime.date.today()
    today = str(today)
    memory_path = "/home/a-ichikura/memory/" + today
    if not os.path.exists(memory_path):
        os.makedirs(memory_path)
    time.sleep(1.0)
    emotion = rospy.get_param('~emotion',True)
    #emotion = True
    print(emotion)
    if emotion == True:
        node = Emotion()
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
    else:
        node = No_emotion()
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
