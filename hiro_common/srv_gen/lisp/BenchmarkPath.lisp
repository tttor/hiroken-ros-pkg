; Auto-generated. Do not edit!


(cl:in-package hiro_common-srv)


;//! \htmlinclude BenchmarkPath-request.msg.html

(cl:defclass <BenchmarkPath-request> (roslisp-msg-protocol:ros-message)
  ((robot_trajectory
    :reader robot_trajectory
    :initarg :robot_trajectory
    :type arm_navigation_msgs-msg:RobotTrajectory
    :initform (cl:make-instance 'arm_navigation_msgs-msg:RobotTrajectory)))
)

(cl:defclass BenchmarkPath-request (<BenchmarkPath-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BenchmarkPath-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BenchmarkPath-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hiro_common-srv:<BenchmarkPath-request> is deprecated: use hiro_common-srv:BenchmarkPath-request instead.")))

(cl:ensure-generic-function 'robot_trajectory-val :lambda-list '(m))
(cl:defmethod robot_trajectory-val ((m <BenchmarkPath-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hiro_common-srv:robot_trajectory-val is deprecated.  Use hiro_common-srv:robot_trajectory instead.")
  (robot_trajectory m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BenchmarkPath-request>) ostream)
  "Serializes a message object of type '<BenchmarkPath-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'robot_trajectory) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BenchmarkPath-request>) istream)
  "Deserializes a message object of type '<BenchmarkPath-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'robot_trajectory) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BenchmarkPath-request>)))
  "Returns string type for a service object of type '<BenchmarkPath-request>"
  "hiro_common/BenchmarkPathRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BenchmarkPath-request)))
  "Returns string type for a service object of type 'BenchmarkPath-request"
  "hiro_common/BenchmarkPathRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BenchmarkPath-request>)))
  "Returns md5sum for a message object of type '<BenchmarkPath-request>"
  "2165d26d9002c2d13ba6746a298d0f67")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BenchmarkPath-request)))
  "Returns md5sum for a message object of type 'BenchmarkPath-request"
  "2165d26d9002c2d13ba6746a298d0f67")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BenchmarkPath-request>)))
  "Returns full string definition for message of type '<BenchmarkPath-request>"
  (cl:format cl:nil "arm_navigation_msgs/RobotTrajectory robot_trajectory~%~%================================================================================~%MSG: arm_navigation_msgs/RobotTrajectory~%trajectory_msgs/JointTrajectory joint_trajectory~%arm_navigation_msgs/MultiDOFJointTrajectory multi_dof_joint_trajectory~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectory~%Header header~%string[] joint_names~%JointTrajectoryPoint[] points~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%float64[] positions~%float64[] velocities~%float64[] accelerations~%duration time_from_start~%================================================================================~%MSG: arm_navigation_msgs/MultiDOFJointTrajectory~%#A representation of a multi-dof joint trajectory~%duration stamp~%string[] joint_names~%string[] frame_ids~%string[] child_frame_ids~%MultiDOFJointTrajectoryPoint[] points~%~%================================================================================~%MSG: arm_navigation_msgs/MultiDOFJointTrajectoryPoint~%geometry_msgs/Pose[] poses~%duration time_from_start~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BenchmarkPath-request)))
  "Returns full string definition for message of type 'BenchmarkPath-request"
  (cl:format cl:nil "arm_navigation_msgs/RobotTrajectory robot_trajectory~%~%================================================================================~%MSG: arm_navigation_msgs/RobotTrajectory~%trajectory_msgs/JointTrajectory joint_trajectory~%arm_navigation_msgs/MultiDOFJointTrajectory multi_dof_joint_trajectory~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectory~%Header header~%string[] joint_names~%JointTrajectoryPoint[] points~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%float64[] positions~%float64[] velocities~%float64[] accelerations~%duration time_from_start~%================================================================================~%MSG: arm_navigation_msgs/MultiDOFJointTrajectory~%#A representation of a multi-dof joint trajectory~%duration stamp~%string[] joint_names~%string[] frame_ids~%string[] child_frame_ids~%MultiDOFJointTrajectoryPoint[] points~%~%================================================================================~%MSG: arm_navigation_msgs/MultiDOFJointTrajectoryPoint~%geometry_msgs/Pose[] poses~%duration time_from_start~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BenchmarkPath-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'robot_trajectory))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BenchmarkPath-request>))
  "Converts a ROS message object to a list"
  (cl:list 'BenchmarkPath-request
    (cl:cons ':robot_trajectory (robot_trajectory msg))
))
;//! \htmlinclude BenchmarkPath-response.msg.html

(cl:defclass <BenchmarkPath-response> (roslisp-msg-protocol:ros-message)
  ((length
    :reader length
    :initarg :length
    :type cl:float
    :initform 0.0))
)

(cl:defclass BenchmarkPath-response (<BenchmarkPath-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BenchmarkPath-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BenchmarkPath-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hiro_common-srv:<BenchmarkPath-response> is deprecated: use hiro_common-srv:BenchmarkPath-response instead.")))

(cl:ensure-generic-function 'length-val :lambda-list '(m))
(cl:defmethod length-val ((m <BenchmarkPath-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hiro_common-srv:length-val is deprecated.  Use hiro_common-srv:length instead.")
  (length m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BenchmarkPath-response>) ostream)
  "Serializes a message object of type '<BenchmarkPath-response>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'length))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BenchmarkPath-response>) istream)
  "Deserializes a message object of type '<BenchmarkPath-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'length) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BenchmarkPath-response>)))
  "Returns string type for a service object of type '<BenchmarkPath-response>"
  "hiro_common/BenchmarkPathResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BenchmarkPath-response)))
  "Returns string type for a service object of type 'BenchmarkPath-response"
  "hiro_common/BenchmarkPathResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BenchmarkPath-response>)))
  "Returns md5sum for a message object of type '<BenchmarkPath-response>"
  "2165d26d9002c2d13ba6746a298d0f67")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BenchmarkPath-response)))
  "Returns md5sum for a message object of type 'BenchmarkPath-response"
  "2165d26d9002c2d13ba6746a298d0f67")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BenchmarkPath-response>)))
  "Returns full string definition for message of type '<BenchmarkPath-response>"
  (cl:format cl:nil "float64 length~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BenchmarkPath-response)))
  "Returns full string definition for message of type 'BenchmarkPath-response"
  (cl:format cl:nil "float64 length~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BenchmarkPath-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BenchmarkPath-response>))
  "Converts a ROS message object to a list"
  (cl:list 'BenchmarkPath-response
    (cl:cons ':length (length msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'BenchmarkPath)))
  'BenchmarkPath-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'BenchmarkPath)))
  'BenchmarkPath-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BenchmarkPath)))
  "Returns string type for a service object of type '<BenchmarkPath>"
  "hiro_common/BenchmarkPath")