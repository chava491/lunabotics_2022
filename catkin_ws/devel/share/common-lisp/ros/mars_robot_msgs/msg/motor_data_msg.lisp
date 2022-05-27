; Auto-generated. Do not edit!


(cl:in-package mars_robot_msgs-msg)


;//! \htmlinclude motor_data_msg.msg.html

(cl:defclass <motor_data_msg> (roslisp-msg-protocol:ros-message)
  ((auger_current
    :reader auger_current
    :initarg :auger_current
    :type cl:float
    :initform 0.0)
   (auger_speed
    :reader auger_speed
    :initarg :auger_speed
    :type cl:float
    :initform 0.0)
   (right_loco_current
    :reader right_loco_current
    :initarg :right_loco_current
    :type cl:float
    :initform 0.0)
   (left_loco_current
    :reader left_loco_current
    :initarg :left_loco_current
    :type cl:float
    :initform 0.0))
)

(cl:defclass motor_data_msg (<motor_data_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <motor_data_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'motor_data_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mars_robot_msgs-msg:<motor_data_msg> is deprecated: use mars_robot_msgs-msg:motor_data_msg instead.")))

(cl:ensure-generic-function 'auger_current-val :lambda-list '(m))
(cl:defmethod auger_current-val ((m <motor_data_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mars_robot_msgs-msg:auger_current-val is deprecated.  Use mars_robot_msgs-msg:auger_current instead.")
  (auger_current m))

(cl:ensure-generic-function 'auger_speed-val :lambda-list '(m))
(cl:defmethod auger_speed-val ((m <motor_data_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mars_robot_msgs-msg:auger_speed-val is deprecated.  Use mars_robot_msgs-msg:auger_speed instead.")
  (auger_speed m))

(cl:ensure-generic-function 'right_loco_current-val :lambda-list '(m))
(cl:defmethod right_loco_current-val ((m <motor_data_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mars_robot_msgs-msg:right_loco_current-val is deprecated.  Use mars_robot_msgs-msg:right_loco_current instead.")
  (right_loco_current m))

(cl:ensure-generic-function 'left_loco_current-val :lambda-list '(m))
(cl:defmethod left_loco_current-val ((m <motor_data_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mars_robot_msgs-msg:left_loco_current-val is deprecated.  Use mars_robot_msgs-msg:left_loco_current instead.")
  (left_loco_current m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <motor_data_msg>) ostream)
  "Serializes a message object of type '<motor_data_msg>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'auger_current))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'auger_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_loco_current))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'left_loco_current))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <motor_data_msg>) istream)
  "Deserializes a message object of type '<motor_data_msg>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'auger_current) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'auger_speed) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_loco_current) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_loco_current) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<motor_data_msg>)))
  "Returns string type for a message object of type '<motor_data_msg>"
  "mars_robot_msgs/motor_data_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motor_data_msg)))
  "Returns string type for a message object of type 'motor_data_msg"
  "mars_robot_msgs/motor_data_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<motor_data_msg>)))
  "Returns md5sum for a message object of type '<motor_data_msg>"
  "1a39659befe0182ae29a42de47aa67fc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'motor_data_msg)))
  "Returns md5sum for a message object of type 'motor_data_msg"
  "1a39659befe0182ae29a42de47aa67fc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<motor_data_msg>)))
  "Returns full string definition for message of type '<motor_data_msg>"
  (cl:format cl:nil "float32 auger_current~%float32 auger_speed~%float32 right_loco_current~%float32 left_loco_current~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'motor_data_msg)))
  "Returns full string definition for message of type 'motor_data_msg"
  (cl:format cl:nil "float32 auger_current~%float32 auger_speed~%float32 right_loco_current~%float32 left_loco_current~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <motor_data_msg>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <motor_data_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'motor_data_msg
    (cl:cons ':auger_current (auger_current msg))
    (cl:cons ':auger_speed (auger_speed msg))
    (cl:cons ':right_loco_current (right_loco_current msg))
    (cl:cons ':left_loco_current (left_loco_current msg))
))
