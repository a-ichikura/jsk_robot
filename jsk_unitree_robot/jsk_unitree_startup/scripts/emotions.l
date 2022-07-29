#!/usr/bin/env roseus

;; load simple model for fast-boot
(require :go1 "package://unitreeeus/go1-simple.l")

;; copied from https://github.com/jsk-ros-pkg/jsk_demos/blob/0a82540da013b8e21307354bb1c6a552185e36a2/jsk_spot_watch_dog/scripts/watch-dog.l
(load "package://unitreeeus/unitree-interface.l")

(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-msgs "speech_recognition_msgs")
;;;
;;; Happy
;;;
(defun happy-pose-1(&optional (time 3000))
  (progn (send *go1* :angle-vector (send *ri* :state :potentio-vector))
	 (send *go1* :body-pose '(0.4 -0.4 -0.2))
	 (send *ri* :body-pose '(0.4 -0.4 -0.2))
	 ))

(defun happy-pose-2(&optional (time 3000))
  (progn (send *go1* :angle-vector (send *ri* :state :potentio-vector))
	 (send *go1* :body-pose '(-0.4 -0.4 0.2))
	 (send *ri* :body-pose '(-0.4 -0.4 0,2))
	 ))

(defun reset-pose(&optional (time 3000))
  (progn (send *go1* :angle-vector (send *ri* :state :potentio-vector))
	 (send *go1* :angle-vector #f(0.0 45.0 -90.0 0.0 45.0 -90.0 0.0 45.0 -90.0 0.0 45.0 -90.0))
	 (send *ri* :body-pose '(0 0 0))
	 ))

(defun happy ()
  (reset-pose 1000)
  (happy-pose-1 800)
  (unix:usleep (* 1000 1000))
  (reset-pose 800)
  (unix:usleep (* 1000 800))
  (happy-pose-2 800)
  (unix:usleep (* 1000 1000))
  (reset-pose 800)
  (unix:usleep (* 1000 800))

  (happy-pose-1 800)
  (unix:usleep (* 1000 1000))
  (reset-pose 800)
  (unix:usleep (* 1000 800))
  (happy-pose-2 800)
  (unix:usleep (* 1000 1000))
  (reset-pose 800)
  (unix:usleep (* 1000 800))
  )

;;;
;;; Main
;;;

(go1-init)
(ros::ros-info "go1-initialized")

(defun people-cb (msg)
  (ros::ros-info "~A" (send-all (send msg :poses) :limb_names))
  (if (> (length (send msg :poses)) 0)
      (happy))
  )
(defun speech-cb (msg)
  (ros::ros-info "~A" (send msg :transcript))
  (if (string= (car (send msg :transcript)) "こんにちは")
      (happy)
      ))
(ros::subscribe "people_pose" jsk_recognition_msgs::PeoplePoseArray #'people-cb)
(ros::subscribe "speech_to_text" speech_recognition_msgs::SpeechRecognitionCandidates #'speech-cb)

(ros::rate 10)
(do-until-key
    (ros::sleep)
    (ros::spin-once))
;; (send *ri* :play-sound
;;       (pathname (ros::get-param "~bark_sound" "/opt/jsk/User/src/jsk_robot/jsk_unitree_robot/jsk_unitree_startup/autostart/bark.wav")))
;; (ros::ros-info "send :play-sound, wait for 2 sec..")
;; (unix::sleep 2)
;; (ros::ros-info "exitting...")
;; (sys::exit 0)
;
