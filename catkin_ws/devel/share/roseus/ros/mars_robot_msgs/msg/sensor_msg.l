;; Auto-generated. Do not edit!


(when (boundp 'mars_robot_msgs::sensor_msg)
  (if (not (find-package "MARS_ROBOT_MSGS"))
    (make-package "MARS_ROBOT_MSGS"))
  (shadow 'sensor_msg (find-package "MARS_ROBOT_MSGS")))
(unless (find-package "MARS_ROBOT_MSGS::SENSOR_MSG")
  (make-package "MARS_ROBOT_MSGS::SENSOR_MSG"))

(in-package "ROS")
;;//! \htmlinclude sensor_msg.msg.html


(defclass mars_robot_msgs::sensor_msg
  :super ros::object
  :slots (_laser_top_hit _laser_left_hit _laser_right_hit _depth_bottom_switch _depth_top_switch _yaw _mass ))

(defmethod mars_robot_msgs::sensor_msg
  (:init
   (&key
    ((:laser_top_hit __laser_top_hit) nil)
    ((:laser_left_hit __laser_left_hit) nil)
    ((:laser_right_hit __laser_right_hit) nil)
    ((:depth_bottom_switch __depth_bottom_switch) nil)
    ((:depth_top_switch __depth_top_switch) nil)
    ((:yaw __yaw) 0.0)
    ((:mass __mass) 0.0)
    )
   (send-super :init)
   (setq _laser_top_hit __laser_top_hit)
   (setq _laser_left_hit __laser_left_hit)
   (setq _laser_right_hit __laser_right_hit)
   (setq _depth_bottom_switch __depth_bottom_switch)
   (setq _depth_top_switch __depth_top_switch)
   (setq _yaw (float __yaw))
   (setq _mass (float __mass))
   self)
  (:laser_top_hit
   (&optional __laser_top_hit)
   (if __laser_top_hit (setq _laser_top_hit __laser_top_hit)) _laser_top_hit)
  (:laser_left_hit
   (&optional __laser_left_hit)
   (if __laser_left_hit (setq _laser_left_hit __laser_left_hit)) _laser_left_hit)
  (:laser_right_hit
   (&optional __laser_right_hit)
   (if __laser_right_hit (setq _laser_right_hit __laser_right_hit)) _laser_right_hit)
  (:depth_bottom_switch
   (&optional __depth_bottom_switch)
   (if __depth_bottom_switch (setq _depth_bottom_switch __depth_bottom_switch)) _depth_bottom_switch)
  (:depth_top_switch
   (&optional __depth_top_switch)
   (if __depth_top_switch (setq _depth_top_switch __depth_top_switch)) _depth_top_switch)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:mass
   (&optional __mass)
   (if __mass (setq _mass __mass)) _mass)
  (:serialization-length
   ()
   (+
    ;; bool _laser_top_hit
    1
    ;; bool _laser_left_hit
    1
    ;; bool _laser_right_hit
    1
    ;; bool _depth_bottom_switch
    1
    ;; bool _depth_top_switch
    1
    ;; float32 _yaw
    4
    ;; float32 _mass
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _laser_top_hit
       (if _laser_top_hit (write-byte -1 s) (write-byte 0 s))
     ;; bool _laser_left_hit
       (if _laser_left_hit (write-byte -1 s) (write-byte 0 s))
     ;; bool _laser_right_hit
       (if _laser_right_hit (write-byte -1 s) (write-byte 0 s))
     ;; bool _depth_bottom_switch
       (if _depth_bottom_switch (write-byte -1 s) (write-byte 0 s))
     ;; bool _depth_top_switch
       (if _depth_top_switch (write-byte -1 s) (write-byte 0 s))
     ;; float32 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _mass
       (sys::poke _mass (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _laser_top_hit
     (setq _laser_top_hit (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _laser_left_hit
     (setq _laser_left_hit (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _laser_right_hit
     (setq _laser_right_hit (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _depth_bottom_switch
     (setq _depth_bottom_switch (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _depth_top_switch
     (setq _depth_top_switch (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _yaw
     (setq _yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _mass
     (setq _mass (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get mars_robot_msgs::sensor_msg :md5sum-) "57d6b1ca27430172008d546be1f39dfb")
(setf (get mars_robot_msgs::sensor_msg :datatype-) "mars_robot_msgs/sensor_msg")
(setf (get mars_robot_msgs::sensor_msg :definition-)
      "bool laser_top_hit
bool laser_left_hit
bool laser_right_hit
bool depth_bottom_switch
bool depth_top_switch
float32 yaw
float32 mass

")



(provide :mars_robot_msgs/sensor_msg "57d6b1ca27430172008d546be1f39dfb")


