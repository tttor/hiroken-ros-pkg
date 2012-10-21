; Auto-generated. Do not edit!


(cl:in-package hiro_control-srv)


;//! \htmlinclude CommitMovingArm-request.msg.html

(cl:defclass <CommitMovingArm-request> (roslisp-msg-protocol:ros-message)
  ((signal
    :reader signal
    :initarg :signal
    :type cl:fixnum
    :initform 0))
)

(cl:defclass CommitMovingArm-request (<CommitMovingArm-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CommitMovingArm-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CommitMovingArm-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hiro_control-srv:<CommitMovingArm-request> is deprecated: use hiro_control-srv:CommitMovingArm-request instead.")))

(cl:ensure-generic-function 'signal-val :lambda-list '(m))
(cl:defmethod signal-val ((m <CommitMovingArm-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hiro_control-srv:signal-val is deprecated.  Use hiro_control-srv:signal instead.")
  (signal m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CommitMovingArm-request>) ostream)
  "Serializes a message object of type '<CommitMovingArm-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'signal)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CommitMovingArm-request>) istream)
  "Deserializes a message object of type '<CommitMovingArm-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'signal)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CommitMovingArm-request>)))
  "Returns string type for a service object of type '<CommitMovingArm-request>"
  "hiro_control/CommitMovingArmRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CommitMovingArm-request)))
  "Returns string type for a service object of type 'CommitMovingArm-request"
  "hiro_control/CommitMovingArmRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CommitMovingArm-request>)))
  "Returns md5sum for a message object of type '<CommitMovingArm-request>"
  "8b0c8c802c21c92bce78aaf2cfb5bdb6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CommitMovingArm-request)))
  "Returns md5sum for a message object of type 'CommitMovingArm-request"
  "8b0c8c802c21c92bce78aaf2cfb5bdb6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CommitMovingArm-request>)))
  "Returns full string definition for message of type '<CommitMovingArm-request>"
  (cl:format cl:nil "uint8 signal~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CommitMovingArm-request)))
  "Returns full string definition for message of type 'CommitMovingArm-request"
  (cl:format cl:nil "uint8 signal~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CommitMovingArm-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CommitMovingArm-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CommitMovingArm-request
    (cl:cons ':signal (signal msg))
))
;//! \htmlinclude CommitMovingArm-response.msg.html

(cl:defclass <CommitMovingArm-response> (roslisp-msg-protocol:ros-message)
  ((msg
    :reader msg
    :initarg :msg
    :type cl:string
    :initform ""))
)

(cl:defclass CommitMovingArm-response (<CommitMovingArm-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CommitMovingArm-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CommitMovingArm-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hiro_control-srv:<CommitMovingArm-response> is deprecated: use hiro_control-srv:CommitMovingArm-response instead.")))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <CommitMovingArm-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hiro_control-srv:msg-val is deprecated.  Use hiro_control-srv:msg instead.")
  (msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CommitMovingArm-response>) ostream)
  "Serializes a message object of type '<CommitMovingArm-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'msg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'msg))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CommitMovingArm-response>) istream)
  "Deserializes a message object of type '<CommitMovingArm-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'msg) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'msg) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CommitMovingArm-response>)))
  "Returns string type for a service object of type '<CommitMovingArm-response>"
  "hiro_control/CommitMovingArmResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CommitMovingArm-response)))
  "Returns string type for a service object of type 'CommitMovingArm-response"
  "hiro_control/CommitMovingArmResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CommitMovingArm-response>)))
  "Returns md5sum for a message object of type '<CommitMovingArm-response>"
  "8b0c8c802c21c92bce78aaf2cfb5bdb6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CommitMovingArm-response)))
  "Returns md5sum for a message object of type 'CommitMovingArm-response"
  "8b0c8c802c21c92bce78aaf2cfb5bdb6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CommitMovingArm-response>)))
  "Returns full string definition for message of type '<CommitMovingArm-response>"
  (cl:format cl:nil "string msg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CommitMovingArm-response)))
  "Returns full string definition for message of type 'CommitMovingArm-response"
  (cl:format cl:nil "string msg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CommitMovingArm-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CommitMovingArm-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CommitMovingArm-response
    (cl:cons ':msg (msg msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CommitMovingArm)))
  'CommitMovingArm-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CommitMovingArm)))
  'CommitMovingArm-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CommitMovingArm)))
  "Returns string type for a service object of type '<CommitMovingArm>"
  "hiro_control/CommitMovingArm")