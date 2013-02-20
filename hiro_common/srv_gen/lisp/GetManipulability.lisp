; Auto-generated. Do not edit!


(cl:in-package hiro_common-srv)


;//! \htmlinclude GetManipulability-request.msg.html

(cl:defclass <GetManipulability-request> (roslisp-msg-protocol:ros-message)
  ((jstate
    :reader jstate
    :initarg :jstate
    :type sensor_msgs-msg:JointState
    :initform (cl:make-instance 'sensor_msgs-msg:JointState))
   (jspace
    :reader jspace
    :initarg :jspace
    :type cl:string
    :initform ""))
)

(cl:defclass GetManipulability-request (<GetManipulability-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetManipulability-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetManipulability-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hiro_common-srv:<GetManipulability-request> is deprecated: use hiro_common-srv:GetManipulability-request instead.")))

(cl:ensure-generic-function 'jstate-val :lambda-list '(m))
(cl:defmethod jstate-val ((m <GetManipulability-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hiro_common-srv:jstate-val is deprecated.  Use hiro_common-srv:jstate instead.")
  (jstate m))

(cl:ensure-generic-function 'jspace-val :lambda-list '(m))
(cl:defmethod jspace-val ((m <GetManipulability-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hiro_common-srv:jspace-val is deprecated.  Use hiro_common-srv:jspace instead.")
  (jspace m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetManipulability-request>) ostream)
  "Serializes a message object of type '<GetManipulability-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'jstate) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'jspace))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'jspace))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetManipulability-request>) istream)
  "Deserializes a message object of type '<GetManipulability-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'jstate) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'jspace) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'jspace) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetManipulability-request>)))
  "Returns string type for a service object of type '<GetManipulability-request>"
  "hiro_common/GetManipulabilityRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetManipulability-request)))
  "Returns string type for a service object of type 'GetManipulability-request"
  "hiro_common/GetManipulabilityRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetManipulability-request>)))
  "Returns md5sum for a message object of type '<GetManipulability-request>"
  "c55e73d3b1934abd866df4c7652e9c73")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetManipulability-request)))
  "Returns md5sum for a message object of type 'GetManipulability-request"
  "c55e73d3b1934abd866df4c7652e9c73")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetManipulability-request>)))
  "Returns full string definition for message of type '<GetManipulability-request>"
  (cl:format cl:nil "sensor_msgs/JointState jstate~%string jspace~%~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetManipulability-request)))
  "Returns full string definition for message of type 'GetManipulability-request"
  (cl:format cl:nil "sensor_msgs/JointState jstate~%string jspace~%~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetManipulability-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'jstate))
     4 (cl:length (cl:slot-value msg 'jspace))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetManipulability-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetManipulability-request
    (cl:cons ':jstate (jstate msg))
    (cl:cons ':jspace (jspace msg))
))
;//! \htmlinclude GetManipulability-response.msg.html

(cl:defclass <GetManipulability-response> (roslisp-msg-protocol:ros-message)
  ((m
    :reader m
    :initarg :m
    :type cl:float
    :initform 0.0))
)

(cl:defclass GetManipulability-response (<GetManipulability-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetManipulability-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetManipulability-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hiro_common-srv:<GetManipulability-response> is deprecated: use hiro_common-srv:GetManipulability-response instead.")))

(cl:ensure-generic-function 'm-val :lambda-list '(m))
(cl:defmethod m-val ((m <GetManipulability-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hiro_common-srv:m-val is deprecated.  Use hiro_common-srv:m instead.")
  (m m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetManipulability-response>) ostream)
  "Serializes a message object of type '<GetManipulability-response>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'm))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetManipulability-response>) istream)
  "Deserializes a message object of type '<GetManipulability-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'm) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetManipulability-response>)))
  "Returns string type for a service object of type '<GetManipulability-response>"
  "hiro_common/GetManipulabilityResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetManipulability-response)))
  "Returns string type for a service object of type 'GetManipulability-response"
  "hiro_common/GetManipulabilityResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetManipulability-response>)))
  "Returns md5sum for a message object of type '<GetManipulability-response>"
  "c55e73d3b1934abd866df4c7652e9c73")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetManipulability-response)))
  "Returns md5sum for a message object of type 'GetManipulability-response"
  "c55e73d3b1934abd866df4c7652e9c73")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetManipulability-response>)))
  "Returns full string definition for message of type '<GetManipulability-response>"
  (cl:format cl:nil "float64 m~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetManipulability-response)))
  "Returns full string definition for message of type 'GetManipulability-response"
  (cl:format cl:nil "float64 m~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetManipulability-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetManipulability-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetManipulability-response
    (cl:cons ':m (m msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetManipulability)))
  'GetManipulability-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetManipulability)))
  'GetManipulability-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetManipulability)))
  "Returns string type for a service object of type '<GetManipulability>"
  "hiro_common/GetManipulability")