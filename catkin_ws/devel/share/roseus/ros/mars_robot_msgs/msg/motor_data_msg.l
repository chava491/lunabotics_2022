;; Auto-generated. Do not edit!


(when (boundp 'mars_robot_msgs::motor_data_msg)
  (if (not (find-package "MARS_ROBOT_MSGS"))
    (make-package "MARS_ROBOT_MSGS"))
  (shadow 'motor_data_msg (find-package "MARS_ROBOT_MSGS")))
(unless (find-package "MARS_ROBOT_MSGS::MOTOR_DATA_MSG")
  (make-package "MARS_ROBOT_MSGS::MOTOR_DATA_MSG"))

(in-package "ROS")
;;//! \htmlinclude motor_data_msg.msg.html


(defclass mars_robot_msgs::motor_data_msg
  :super ros::object
  :slots (_auger_current _auger_speed _right_loco_current _left_loco_current ))

(defmethod mars_robot_msgs::motor_data_msg
  (:init
   (&key
    ((:auger_current __auger_current) 0.0)
    ((:auger_speed __auger_speed) 0.0)
    ((:right_loco_current __right_loco_current) 0.0)
    ((:left_loco_current __left_loco_current) 0.0)
    )
   (send-super :init)
   (setq _auger_current (float __auger_current))
   (setq _auger_speed (float __auger_speed))
   (setq _right_loco_current (float __right_loco_current))
   (setq _left_loco_current (float __left_loco_current))
   self)
  (:auger_current
   (&optional __auger_current)
   (if __auger_current (setq _auger_current __auger_current)) _auger_current)
  (:auger_speed
   (&optional __auger_speed)
   (if __auger_speed (setq _auger_speed __auger_speed)) _auger_speed)
  (:right_loco_current
   (&optional __right_loco_current)
   (if __right_loco_current (setq _right_loco_current __right_loco_current)) _right_loco_current)
  (:left_loco_current
   (&optional __left_loco_current)
   (if __left_loco_current (setq _left_loco_current __left_loco_current)) _left_loco_current)
  (:serialization-length
   ()
   (+
    ;; float32 _auger_current
    4
    ;; float32 _auger_speed
    4
    ;; float32 _right_loco_current
    4
    ;; float32 _left_loco_current
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _auger_current
       (sys::poke _auger_current (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _auger_speed
       (sys::poke _auger_speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _right_loco_current
       (sys::poke _right_loco_current (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _left_loco_current
       (sys::poke _left_loco_current (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _auger_current
     (setq _auger_current (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _auger_speed
     (setq _auger_speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _right_loco_current
     (setq _right_loco_current (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _left_loco_current
     (setq _left_loco_current (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get mars_robot_msgs::motor_data_msg :md5sum-) "1a39659befe0182ae29a42de47aa67fc")
(setf (get mars_robot_msgs::motor_data_msg :datatype-) "mars_robot_msgs/motor_data_msg")
(setf (get mars_robot_msgs::motor_data_msg :definition-)
      "float32 auger_current
float32 auger_speed
float32 right_loco_current
float32 left_loco_current
")



(provide :mars_robot_msgs/motor_data_msg "1a39659befe0182ae29a42de47aa67fc")


