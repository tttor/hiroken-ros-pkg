; Auto-generated. Do not edit!


(cl:in-package hiro_control-srv)


;//! \htmlinclude ControlArm-request.msg.html

(cl:defclass <ControlArm-request> (roslisp-msg-protocol:ros-message)
  ((desired
    :reader desired
    :initarg :desired
    :type trajectory_msgs-msg:JointTrajectoryPoint
    :initform (cl:make-instance 'trajectory_msgs-msg:JointTrajectoryPoint)))
)

(cl:defclass ControlArm-request (<ControlArm-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlArm-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlArm-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hiro_control-srv:<ControlArm-request> is deprecated: use hiro_control-srv:ControlArm-request instead.")))

(cl:ensure-generic-function 'desired-val :lambda-list '(m))
(cl:defmethod desired-val ((m <ControlArm-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hiro_control-srv:desired-val is deprecated.  Use hiro_control-srv:desired instead.")
  (desired m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlArm-request>) ostream)
  "Serializes a message object of type '<ControlArm-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'desired) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlArm-request>) istream)
  "Deserializes a message object of type '<ControlArm-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'desired) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlArm-request>)))
  "Returns string type for a service object of type '<ControlArm-request>"
  "hiro_control/ControlArmRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlArm-request)))
  "Returns string type for a service object of type 'ControlArm-request"
  "hiro_control/ControlArmRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlArm-request>)))
  "Returns md5sum for a message object of type '<ControlArm-request>"
  "58875e2aaba0886d8df9015de4efb1ec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlArm-request)))
  "Returns md5sum for a message object of type 'ControlArm-request"
  "58875e2aaba0886d8df9015de4efb1ec")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlArm-request>)))
  "Returns full string definition for message of type '<ControlArm-request>"
  (cl:format cl:nil "~%~%~%trajectory_msgs/JointTrajectoryPoint desired~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%float64[] positions~%float64[] velocities~%float64[] accelerations~%duration time_from_start~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlArm-request)))
  "Returns full string definition for message of type 'ControlArm-request"
  (cl:format cl:nil "~%~%~%trajectory_msgs/JointTrajectoryPoint desired~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%float64[] positions~%float64[] velocities~%float64[] accelerations~%duration time_from_start~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlArm-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'desired))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlArm-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlArm-request
    (cl:cons ':desired (desired msg))
))
;//! \htmlinclude ControlArm-response.msg.html

(cl:defclass <ControlArm-response> (roslisp-msg-protocol:ros-message)
  ((actual
    :reader actual
    :initarg :actual
    :type trajectory_msgs-msg:JointTrajectoryPoint
    :initform (cl:make-instance 'trajectory_msgs-msg:JointTrajectoryPoint)))
)

(cl:defclass ControlArm-response (<ControlArm-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlArm-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlArm-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hiro_control-srv:<ControlArm-response> is deprecated: use hiro_control-srv:ControlArm-response instead.")))

(cl:ensure-generic-function 'actual-val :lambda-list '(m))
(cl:defmethod actual-val ((m <ControlArm-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hiro_control-srv:actual-val is deprecated.  Use hiro_control-srv:actual instead.")
  (actual m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlArm-response>) ostream)
  "Serializes a message object of type '<ControlArm-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'actual) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlArm-response>) istream)
  "Deserializes a message object of type '<ControlArm-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'actual) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlArm-response>)))
  "Returns string type for a service object of type '<ControlArm-response>"
  "hiro_control/ControlArmResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlArm-response)))
  "Returns string type for a service object of type 'ControlArm-response"
  "hiro_control/ControlArmResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlArm-response>)))
  "Returns md5sum for a message object of type '<ControlArm-response>"
  "58875e2aaba0886d8df9015de4efb1ec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlArm-response)))
  "Returns md5sum for a message object of type 'ControlArm-response"
  "58875e2aaba0886d8df9015de4efb1ec")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlArm-response>)))
  "Returns full string definition for message of type '<ControlArm-response>"
  (cl:format cl:nil "~%~%~%trajectory_msgs/JointTrajectoryPoint actual~%~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%float64[] positions~%float64[] velocities~%float64[] accelerations~%duration time_from_start~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlArm-response)))
  "Returns full string definition for message of type 'ControlArm-response"
  (cl:format cl:nil "~%~%~%trajectory_msgs/JointTrajectoryPoint actual~%~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%float64[] positions~%float64[] velocities~%float64[] accelerations~%duration time_from_start~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlArm-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'actual))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlArm-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlArm-response
    (cl:cons ':actual (actual msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ControlArm)))
  'ControlArm-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ControlArm)))
  'ControlArm-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlArm)))
  "Returns string type for a service object of type '<ControlArm>"
  "hiro_control/ControlArm")