; Auto-generated. Do not edit!


(cl:in-package mars_robot_msgs-msg)


;//! \htmlinclude sensor_msg.msg.html

(cl:defclass <sensor_msg> (roslisp-msg-protocol:ros-message)
  ((laser_top_hit
    :reader laser_top_hit
    :initarg :laser_top_hit
    :type cl:boolean
    :initform cl:nil)
   (laser_left_hit
    :reader laser_left_hit
    :initarg :laser_left_hit
    :type cl:boolean
    :initform cl:nil)
   (laser_right_hit
    :reader laser_right_hit
    :initarg :laser_right_hit
    :type cl:boolean
    :initform cl:nil)
   (depth_bottom_switch
    :reader depth_bottom_switch
    :initarg :depth_bottom_switch
    :type cl:boolean
    :initform cl:nil)
   (depth_top_switch
    :reader depth_top_switch
    :initarg :depth_top_switch
    :type cl:boolean
    :initform cl:nil)
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0)
   (mass
    :reader mass
    :initarg :mass
    :type cl:float
    :initform 0.0))
)

(cl:defclass sensor_msg (<sensor_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <sensor_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'sensor_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mars_robot_msgs-msg:<sensor_msg> is deprecated: use mars_robot_msgs-msg:sensor_msg instead.")))

(cl:ensure-generic-function 'laser_top_hit-val :lambda-list '(m))
(cl:defmethod laser_top_hit-val ((m <sensor_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mars_robot_msgs-msg:laser_top_hit-val is deprecated.  Use mars_robot_msgs-msg:laser_top_hit instead.")
  (laser_top_hit m))

(cl:ensure-generic-function 'laser_left_hit-val :lambda-list '(m))
(cl:defmethod laser_left_hit-val ((m <sensor_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mars_robot_msgs-msg:laser_left_hit-val is deprecated.  Use mars_robot_msgs-msg:laser_left_hit instead.")
  (laser_left_hit m))

(cl:ensure-generic-function 'laser_right_hit-val :lambda-list '(m))
(cl:defmethod laser_right_hit-val ((m <sensor_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mars_robot_msgs-msg:laser_right_hit-val is deprecated.  Use mars_robot_msgs-msg:laser_right_hit instead.")
  (laser_right_hit m))

(cl:ensure-generic-function 'depth_bottom_switch-val :lambda-list '(m))
(cl:defmethod depth_bottom_switch-val ((m <sensor_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mars_robot_msgs-msg:depth_bottom_switch-val is deprecated.  Use mars_robot_msgs-msg:depth_bottom_switch instead.")
  (depth_bottom_switch m))

(cl:ensure-generic-function 'depth_top_switch-val :lambda-list '(m))
(cl:defmethod depth_top_switch-val ((m <sensor_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mars_robot_msgs-msg:depth_top_switch-val is deprecated.  Use mars_robot_msgs-msg:depth_top_switch instead.")
  (depth_top_switch m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <sensor_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mars_robot_msgs-msg:yaw-val is deprecated.  Use mars_robot_msgs-msg:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'mass-val :lambda-list '(m))
(cl:defmethod mass-val ((m <sensor_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mars_robot_msgs-msg:mass-val is deprecated.  Use mars_robot_msgs-msg:mass instead.")
  (mass m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <sensor_msg>) ostream)
  "Serializes a message object of type '<sensor_msg>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'laser_top_hit) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'laser_left_hit) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'laser_right_hit) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'depth_bottom_switch) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'depth_top_switch) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'mass))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <sensor_msg>) istream)
  "Deserializes a message object of type '<sensor_msg>"
    (cl:setf (cl:slot-value msg 'laser_top_hit) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'laser_left_hit) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'laser_right_hit) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'depth_bottom_switch) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'depth_top_switch) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'mass) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<sensor_msg>)))
  "Returns string type for a message object of type '<sensor_msg>"
  "mars_robot_msgs/sensor_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'sensor_msg)))
  "Returns string type for a message object of type 'sensor_msg"
  "mars_robot_msgs/sensor_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<sensor_msg>)))
  "Returns md5sum for a message object of type '<sensor_msg>"
  "57d6b1ca27430172008d546be1f39dfb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'sensor_msg)))
  "Returns md5sum for a message object of type 'sensor_msg"
  "57d6b1ca27430172008d546be1f39dfb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<sensor_msg>)))
  "Returns full string definition for message of type '<sensor_msg>"
  (cl:format cl:nil "bool laser_top_hit~%bool laser_left_hit~%bool laser_right_hit~%bool depth_bottom_switch~%bool depth_top_switch~%float32 yaw~%float32 mass~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'sensor_msg)))
  "Returns full string definition for message of type 'sensor_msg"
  (cl:format cl:nil "bool laser_top_hit~%bool laser_left_hit~%bool laser_right_hit~%bool depth_bottom_switch~%bool depth_top_switch~%float32 yaw~%float32 mass~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <sensor_msg>))
  (cl:+ 0
     1
     1
     1
     1
     1
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <sensor_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'sensor_msg
    (cl:cons ':laser_top_hit (laser_top_hit msg))
    (cl:cons ':laser_left_hit (laser_left_hit msg))
    (cl:cons ':laser_right_hit (laser_right_hit msg))
    (cl:cons ':depth_bottom_switch (depth_bottom_switch msg))
    (cl:cons ':depth_top_switch (depth_top_switch msg))
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':mass (mass msg))
))
