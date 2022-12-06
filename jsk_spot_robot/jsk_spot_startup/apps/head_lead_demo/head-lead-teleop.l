#!/usr/bin/env roseus

(setq *continue* t)
(load "package://spoteus/spot-interface.l")
(spot-init)
(if (eq (send *ri* :state :power-state-shore-power-state) 'on-shore-power)
    (send *ri* :undock))
;; (send *ri* :set-impedance-param :linear-stiffness #f(250 250 250) :rotational-stiffness #f(30 30 30) :linear-damping #f(1.5 1.5 1.5) :rotational-damping #f(0.5 0.5 0.5))
;; (send *ri* :set-impedance-param :linear-stiffness #f(250 25 25) :rotational-stiffness #f(30 5 5) :linear-damping #f(1.5 1.5 1.5) :rotational-damping #f(0.5 0.5 0.5))


(defun start-func (args)
  (when (not (eq (send *ri* :state :power-state-shore-power-state) 'off-shore-power))
    (send *ri* :speak "Robot is on dock")
    (return-from :start-func :finished))
  (send *ri* :speak "Hello, let's start walking")
  (send *spot* :arm :angle-vector #f(0.0 -130.0 120.0 0.0 10.0 0.0))
  (send *ri* :angle-vector (send *spot* :angle-vector) 2000 :default-controller)
  (send *ri* :wait-interpolation)
  (send *ri* :gripper-open)
  ;;(send *ri* :set-impedance-param :linear-stiffness #f(250 50 50) :rotational-stiffness #f(10 10 30) :linear-damping #f(1.5 1.5 1.5) :rotational-damping #f(0.5 0.5 0.5))
  (send *ri* :set-impedance-param :linear-stiffness #f(250 30 30) :rotational-stiffness #f(10 10 30) :linear-damping #f(1.5 1.0 1.) :rotational-damping #f(0.5 0.5 0.5))
  :started)

(defun end-func (args)
  (send *ri* :speak "Thank You")
  (send *ri* :gripper-close)
  (send *ri* :stow-arm)
  :finished)

(defun walk-func (args)
  (ros::rate 2)
  (do-until-key
   (if (null *continue*) (return-from walk-func :finished))
   (ros::spin-once)
   (ros::sleep))
  :finished)
;;
(load "package://roseus_smach/src/state-machine-ros.l")
(defun walk-sm ()
  (let (sm)
    (setq sm
          (make-state-machine
           '((:start :started :walk)    ;; transitions (node transition node)
             (:start :finished :end)
             (:walk :finished :end)
             (:end :finished :goal)
             )
           '((:start    'start-func)       ;; node-to-function maps
             (:end      'end-func)
             (:walk     'walk-func)
             )
           '(:start)                    ;; initial node
           '(:goal)                     ;; goal node
           ))
    (send sm :arg-keys 'description)
    sm))

;; create robot interface
(unless (boundp '*ri*) (spot-init))
(objects (list *spot*))

;;

;; this does not work...
;; (unix:signal unix::sigint '(lambda-closure nil 0 0 (sig code) (setq *continue* nil)))

;; https://github.com/jsk-ros-pkg/jsk_roseus/pull/717
;; did not work, when we subscribe image data ???

(defun ros::roseus-sigint-handler (sig code)
  (ros::ros-warn (format nil "ros::roseus-sigint-handler ~A" sig))
  (setq *continue* nil))
(unix:signal unix::sigint 'ros::roseus-sigint-handler)

(ros::subscribe "/joy_dualsense" sensor_msgs::Joy
		#'(lambda (msg)
		    ;; (ros::ros-error "~A" msg)
		    ;; (ros::ros-error "~A" (length (send msg :buttons)))
		    ;; (ros::ros-error "~A" (elt (send msg :buttons) 11))
		    (if (and (> (length (send msg :buttons)) 11)
			     (= (elt (send msg :buttons) 11) 1))
			(start-func nil)))
		20)
;; state machine
(exec-state-machine (walk-sm) '((description . "お散歩しました！")(image . "")))
(exit)
