; Auto-generated. Do not edit!


(cl:in-package hiro_control-srv)


;//! \htmlinclude MoveArm-request.msg.html

(cl:defclass <MoveArm-request> (roslisp-msg-protocol:ros-message)
  ((trajectory
    :reader trajectory
    :initarg :trajectory
    :type trajectory_msgs-msg:JointTrajectory
    :initform (cl:make-instance 'trajectory_msgs-msg:JointTrajectory)))
)

(cl:defclass MoveArm-request (<MoveArm-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveArm-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveArm-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hiro_control-srv:<MoveArm-request> is deprecated: use hiro_control-srv:MoveArm-request instead.")))

(cl:ensure-generic-function 'trajectory-val :lambda-list '(m))
(cl:defmethod trajectory-val ((m <MoveArm-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hiro_control-srv:trajectory-val is deprecated.  Use hiro_control-srv:trajectory instead.")
  (trajectory m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveArm-request>) ostream)
  "Serializes a message object of type '<MoveArm-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'trajectory) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveArm-request>) istream)
  "Deserializes a message object of type '<MoveArm-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'trajectory) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveArm-request>)))
  "Returns string type for a service object of type '<MoveArm-request>"
  "hiro_control/MoveArmRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveArm-request)))
  "Returns string type for a service object of type 'MoveArm-request"
  "hiro_control/MoveArmRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveArm-request>)))
  "Returns md5sum for a message object of type '<MoveArm-request>"
  "874d1766247c2b01a9a335bc13e0897e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveArm-request)))
  "Returns md5sum for a message object of type 'MoveArm-request"
  "874d1766247c2b01a9a335bc13e0897e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveArm-request>)))
  "Returns full string definition for message of type '<MoveArm-request>"
  (cl:format cl:nil "trajectory_msgs/JointTrajectory trajectory~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectory~%Header header~%string[] joint_names~%JointTrajectoryPoint[] points~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%float64[] positions~%float64[] velocities~%float64[] accelerations~%duration time_from_start~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveArm-request)))
  "Returns full string definition for message of type 'MoveArm-request"
  (cl:format cl:nil "trajectory_msgs/JointTrajectory trajectory~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectory~%Header header~%string[] joint_names~%JointTrajectoryPoint[] points~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%float64[] positions~%float64[] velocities~%float64[] accelerations~%duration time_from_start~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveArm-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'trajectory))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveArm-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveArm-request
    (cl:cons ':trajectory (trajectory msg))
))
;//! \htmlinclude MoveArm-response.msg.html

(cl:defclass <MoveArm-response> (roslisp-msg-protocol:ros-message)
  ((successful
    :reader successful
    :initarg :successful
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass MoveArm-response (<MoveArm-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveArm-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveArm-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hiro_control-srv:<MoveArm-response> is deprecated: use hiro_control-srv:MoveArm-response instead.")))

(cl:ensure-generic-function 'successful-val :lambda-list '(m))
(cl:defmethod successful-val ((m <MoveArm-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hiro_control-srv:successful-val is deprecated.  Use hiro_control-srv:successful instead.")
  (successful m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveArm-response>) ostream)
  "Serializes a message object of type '<MoveArm-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'successful) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveArm-response>) istream)
  "Deserializes a message object of type '<MoveArm-response>"
    (cl:setf (cl:slot-value msg 'successful) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveArm-response>)))
  "Returns string type for a service object of type '<MoveArm-response>"
  "hiro_control/MoveArmResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveArm-response)))
  "Returns string type for a service object of type 'MoveArm-response"
  "hiro_control/MoveArmResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveArm-response>)))
  "Returns md5sum for a message object of type '<MoveArm-response>"
  "874d1766247c2b01a9a335bc13e0897e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveArm-response)))
  "Returns md5sum for a message object of type 'MoveArm-response"
  "874d1766247c2b01a9a335bc13e0897e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveArm-response>)))
  "Returns full string definition for message of type '<MoveArm-response>"
  (cl:format cl:nil "bool successful~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveArm-response)))
  "Returns full string definition for message of type 'MoveArm-response"
  (cl:format cl:nil "bool successful~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveArm-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveArm-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveArm-response
    (cl:cons ':successful (successful msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MoveArm)))
  'MoveArm-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MoveArm)))
  'MoveArm-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveArm)))
  "Returns string type for a service object of type '<MoveArm>"
  "hiro_control/MoveArm")