
(cl:in-package :asdf)

(defsystem "mars_robot_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "motor_data_msg" :depends-on ("_package_motor_data_msg"))
    (:file "_package_motor_data_msg" :depends-on ("_package"))
    (:file "sensor_msg" :depends-on ("_package_sensor_msg"))
    (:file "_package_sensor_msg" :depends-on ("_package"))
  ))