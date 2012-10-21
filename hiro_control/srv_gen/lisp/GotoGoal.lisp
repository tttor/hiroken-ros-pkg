; Auto-generated. Do not edit!


(cl:in-package hiro_control-srv)


;//! \htmlinclude GotoGoal-request.msg.html

(cl:defclass <GotoGoal-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GotoGoal-request (<GotoGoal-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GotoGoal-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GotoGoal-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hiro_control-srv:<GotoGoal-request> is deprecated: use hiro_control-srv:GotoGoal-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GotoGoal-request>) ostream)
  "Serializes a message object of type '<GotoGoal-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GotoGoal-request>) istream)
  "Deserializes a message object of type '<GotoGoal-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GotoGoal-request>)))
  "Returns string type for a service object of type '<GotoGoal-request>"
  "hiro_control/GotoGoalRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GotoGoal-request)))
  "Returns string type for a service object of type 'GotoGoal-request"
  "hiro_control/GotoGoalRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GotoGoal-request>)))
  "Returns md5sum for a message object of type '<GotoGoal-request>"
  "4fe5af303955c287688e7347e9b00278")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GotoGoal-request)))
  "Returns md5sum for a message object of type 'GotoGoal-request"
  "4fe5af303955c287688e7347e9b00278")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GotoGoal-request>)))
  "Returns full string definition for message of type '<GotoGoal-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GotoGoal-request)))
  "Returns full string definition for message of type 'GotoGoal-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GotoGoal-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GotoGoal-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GotoGoal-request
))
;//! \htmlinclude GotoGoal-response.msg.html

(cl:defclass <GotoGoal-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:string
    :initform ""))
)

(cl:defclass GotoGoal-response (<GotoGoal-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GotoGoal-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GotoGoal-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hiro_control-srv:<GotoGoal-response> is deprecated: use hiro_control-srv:GotoGoal-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <GotoGoal-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hiro_control-srv:status-val is deprecated.  Use hiro_control-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GotoGoal-response>) ostream)
  "Serializes a message object of type '<GotoGoal-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GotoGoal-response>) istream)
  "Deserializes a message object of type '<GotoGoal-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'status) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GotoGoal-response>)))
  "Returns string type for a service object of type '<GotoGoal-response>"
  "hiro_control/GotoGoalResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GotoGoal-response)))
  "Returns string type for a service object of type 'GotoGoal-response"
  "hiro_control/GotoGoalResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GotoGoal-response>)))
  "Returns md5sum for a message object of type '<GotoGoal-response>"
  "4fe5af303955c287688e7347e9b00278")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GotoGoal-response)))
  "Returns md5sum for a message object of type 'GotoGoal-response"
  "4fe5af303955c287688e7347e9b00278")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GotoGoal-response>)))
  "Returns full string definition for message of type '<GotoGoal-response>"
  (cl:format cl:nil "string status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GotoGoal-response)))
  "Returns full string definition for message of type 'GotoGoal-response"
  (cl:format cl:nil "string status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GotoGoal-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'status))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GotoGoal-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GotoGoal-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GotoGoal)))
  'GotoGoal-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GotoGoal)))
  'GotoGoal-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GotoGoal)))
  "Returns string type for a service object of type '<GotoGoal>"
  "hiro_control/GotoGoal")