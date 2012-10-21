; Auto-generated. Do not edit!


(cl:in-package hiro_control-srv)


;//! \htmlinclude ControlHand-request.msg.html

(cl:defclass <ControlHand-request> (roslisp-msg-protocol:ros-message)
  ((cmd
    :reader cmd
    :initarg :cmd
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ControlHand-request (<ControlHand-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlHand-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlHand-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hiro_control-srv:<ControlHand-request> is deprecated: use hiro_control-srv:ControlHand-request instead.")))

(cl:ensure-generic-function 'cmd-val :lambda-list '(m))
(cl:defmethod cmd-val ((m <ControlHand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hiro_control-srv:cmd-val is deprecated.  Use hiro_control-srv:cmd instead.")
  (cmd m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlHand-request>) ostream)
  "Serializes a message object of type '<ControlHand-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cmd)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlHand-request>) istream)
  "Deserializes a message object of type '<ControlHand-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cmd)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlHand-request>)))
  "Returns string type for a service object of type '<ControlHand-request>"
  "hiro_control/ControlHandRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlHand-request)))
  "Returns string type for a service object of type 'ControlHand-request"
  "hiro_control/ControlHandRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlHand-request>)))
  "Returns md5sum for a message object of type '<ControlHand-request>"
  "e652cc4a461c3bb431c9317bbc33fd57")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlHand-request)))
  "Returns md5sum for a message object of type 'ControlHand-request"
  "e652cc4a461c3bb431c9317bbc33fd57")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlHand-request>)))
  "Returns full string definition for message of type '<ControlHand-request>"
  (cl:format cl:nil "uint8 cmd~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlHand-request)))
  "Returns full string definition for message of type 'ControlHand-request"
  (cl:format cl:nil "uint8 cmd~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlHand-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlHand-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlHand-request
    (cl:cons ':cmd (cmd msg))
))
;//! \htmlinclude ControlHand-response.msg.html

(cl:defclass <ControlHand-response> (roslisp-msg-protocol:ros-message)
  ((msg
    :reader msg
    :initarg :msg
    :type cl:string
    :initform ""))
)

(cl:defclass ControlHand-response (<ControlHand-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlHand-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlHand-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hiro_control-srv:<ControlHand-response> is deprecated: use hiro_control-srv:ControlHand-response instead.")))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <ControlHand-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hiro_control-srv:msg-val is deprecated.  Use hiro_control-srv:msg instead.")
  (msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlHand-response>) ostream)
  "Serializes a message object of type '<ControlHand-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'msg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'msg))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlHand-response>) istream)
  "Deserializes a message object of type '<ControlHand-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlHand-response>)))
  "Returns string type for a service object of type '<ControlHand-response>"
  "hiro_control/ControlHandResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlHand-response)))
  "Returns string type for a service object of type 'ControlHand-response"
  "hiro_control/ControlHandResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlHand-response>)))
  "Returns md5sum for a message object of type '<ControlHand-response>"
  "e652cc4a461c3bb431c9317bbc33fd57")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlHand-response)))
  "Returns md5sum for a message object of type 'ControlHand-response"
  "e652cc4a461c3bb431c9317bbc33fd57")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlHand-response>)))
  "Returns full string definition for message of type '<ControlHand-response>"
  (cl:format cl:nil "string msg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlHand-response)))
  "Returns full string definition for message of type 'ControlHand-response"
  (cl:format cl:nil "string msg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlHand-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlHand-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlHand-response
    (cl:cons ':msg (msg msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ControlHand)))
  'ControlHand-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ControlHand)))
  'ControlHand-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlHand)))
  "Returns string type for a service object of type '<ControlHand>"
  "hiro_control/ControlHand")