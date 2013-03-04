; Auto-generated. Do not edit!


(cl:in-package planner_manager-srv)


;//! \htmlinclude Plan-request.msg.html

(cl:defclass <Plan-request> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Plan-request (<Plan-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Plan-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Plan-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name planner_manager-srv:<Plan-request> is deprecated: use planner_manager-srv:Plan-request instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <Plan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader planner_manager-srv:mode-val is deprecated.  Use planner_manager-srv:mode instead.")
  (mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Plan-request>) ostream)
  "Serializes a message object of type '<Plan-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Plan-request>) istream)
  "Deserializes a message object of type '<Plan-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Plan-request>)))
  "Returns string type for a service object of type '<Plan-request>"
  "planner_manager/PlanRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Plan-request)))
  "Returns string type for a service object of type 'Plan-request"
  "planner_manager/PlanRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Plan-request>)))
  "Returns md5sum for a message object of type '<Plan-request>"
  "0d799699f9a6c4ebc406059b14aaf985")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Plan-request)))
  "Returns md5sum for a message object of type 'Plan-request"
  "0d799699f9a6c4ebc406059b14aaf985")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Plan-request>)))
  "Returns full string definition for message of type '<Plan-request>"
  (cl:format cl:nil "uint8 mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Plan-request)))
  "Returns full string definition for message of type 'Plan-request"
  (cl:format cl:nil "uint8 mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Plan-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Plan-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Plan-request
    (cl:cons ':mode (mode msg))
))
;//! \htmlinclude Plan-response.msg.html

(cl:defclass <Plan-response> (roslisp-msg-protocol:ros-message)
  ((ctamp_sol
    :reader ctamp_sol
    :initarg :ctamp_sol
    :type (cl:vector trajectory_msgs-msg:JointTrajectory)
   :initform (cl:make-array 0 :element-type 'trajectory_msgs-msg:JointTrajectory :initial-element (cl:make-instance 'trajectory_msgs-msg:JointTrajectory))))
)

(cl:defclass Plan-response (<Plan-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Plan-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Plan-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name planner_manager-srv:<Plan-response> is deprecated: use planner_manager-srv:Plan-response instead.")))

(cl:ensure-generic-function 'ctamp_sol-val :lambda-list '(m))
(cl:defmethod ctamp_sol-val ((m <Plan-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader planner_manager-srv:ctamp_sol-val is deprecated.  Use planner_manager-srv:ctamp_sol instead.")
  (ctamp_sol m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Plan-response>) ostream)
  "Serializes a message object of type '<Plan-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'ctamp_sol))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'ctamp_sol))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Plan-response>) istream)
  "Deserializes a message object of type '<Plan-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'ctamp_sol) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'ctamp_sol)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'trajectory_msgs-msg:JointTrajectory))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Plan-response>)))
  "Returns string type for a service object of type '<Plan-response>"
  "planner_manager/PlanResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Plan-response)))
  "Returns string type for a service object of type 'Plan-response"
  "planner_manager/PlanResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Plan-response>)))
  "Returns md5sum for a message object of type '<Plan-response>"
  "0d799699f9a6c4ebc406059b14aaf985")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Plan-response)))
  "Returns md5sum for a message object of type 'Plan-response"
  "0d799699f9a6c4ebc406059b14aaf985")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Plan-response>)))
  "Returns full string definition for message of type '<Plan-response>"
  (cl:format cl:nil "trajectory_msgs/JointTrajectory[] ctamp_sol~%~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectory~%Header header~%string[] joint_names~%JointTrajectoryPoint[] points~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%float64[] positions~%float64[] velocities~%float64[] accelerations~%duration time_from_start~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Plan-response)))
  "Returns full string definition for message of type 'Plan-response"
  (cl:format cl:nil "trajectory_msgs/JointTrajectory[] ctamp_sol~%~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectory~%Header header~%string[] joint_names~%JointTrajectoryPoint[] points~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%float64[] positions~%float64[] velocities~%float64[] accelerations~%duration time_from_start~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Plan-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'ctamp_sol) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Plan-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Plan-response
    (cl:cons ':ctamp_sol (ctamp_sol msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Plan)))
  'Plan-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Plan)))
  'Plan-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Plan)))
  "Returns string type for a service object of type '<Plan>"
  "planner_manager/Plan")