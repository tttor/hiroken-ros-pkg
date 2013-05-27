; Auto-generated. Do not edit!


(cl:in-package learning_machine-srv)


;//! \htmlinclude CreateMetadata-request.msg.html

(cl:defclass <CreateMetadata-request> (roslisp-msg-protocol:ros-message)
  ((n_obj
    :reader n_obj
    :initarg :n_obj
    :type cl:fixnum
    :initform 0))
)

(cl:defclass CreateMetadata-request (<CreateMetadata-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CreateMetadata-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CreateMetadata-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name learning_machine-srv:<CreateMetadata-request> is deprecated: use learning_machine-srv:CreateMetadata-request instead.")))

(cl:ensure-generic-function 'n_obj-val :lambda-list '(m))
(cl:defmethod n_obj-val ((m <CreateMetadata-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader learning_machine-srv:n_obj-val is deprecated.  Use learning_machine-srv:n_obj instead.")
  (n_obj m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CreateMetadata-request>) ostream)
  "Serializes a message object of type '<CreateMetadata-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'n_obj)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'n_obj)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CreateMetadata-request>) istream)
  "Deserializes a message object of type '<CreateMetadata-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'n_obj)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'n_obj)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CreateMetadata-request>)))
  "Returns string type for a service object of type '<CreateMetadata-request>"
  "learning_machine/CreateMetadataRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CreateMetadata-request)))
  "Returns string type for a service object of type 'CreateMetadata-request"
  "learning_machine/CreateMetadataRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CreateMetadata-request>)))
  "Returns md5sum for a message object of type '<CreateMetadata-request>"
  "220592dc9786e0034a883b2dab270692")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CreateMetadata-request)))
  "Returns md5sum for a message object of type 'CreateMetadata-request"
  "220592dc9786e0034a883b2dab270692")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CreateMetadata-request>)))
  "Returns full string definition for message of type '<CreateMetadata-request>"
  (cl:format cl:nil "uint16 n_obj~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CreateMetadata-request)))
  "Returns full string definition for message of type 'CreateMetadata-request"
  (cl:format cl:nil "uint16 n_obj~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CreateMetadata-request>))
  (cl:+ 0
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CreateMetadata-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CreateMetadata-request
    (cl:cons ':n_obj (n_obj msg))
))
;//! \htmlinclude CreateMetadata-response.msg.html

(cl:defclass <CreateMetadata-response> (roslisp-msg-protocol:ros-message)
  ((msg
    :reader msg
    :initarg :msg
    :type cl:string
    :initform ""))
)

(cl:defclass CreateMetadata-response (<CreateMetadata-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CreateMetadata-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CreateMetadata-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name learning_machine-srv:<CreateMetadata-response> is deprecated: use learning_machine-srv:CreateMetadata-response instead.")))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <CreateMetadata-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader learning_machine-srv:msg-val is deprecated.  Use learning_machine-srv:msg instead.")
  (msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CreateMetadata-response>) ostream)
  "Serializes a message object of type '<CreateMetadata-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'msg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'msg))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CreateMetadata-response>) istream)
  "Deserializes a message object of type '<CreateMetadata-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CreateMetadata-response>)))
  "Returns string type for a service object of type '<CreateMetadata-response>"
  "learning_machine/CreateMetadataResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CreateMetadata-response)))
  "Returns string type for a service object of type 'CreateMetadata-response"
  "learning_machine/CreateMetadataResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CreateMetadata-response>)))
  "Returns md5sum for a message object of type '<CreateMetadata-response>"
  "220592dc9786e0034a883b2dab270692")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CreateMetadata-response)))
  "Returns md5sum for a message object of type 'CreateMetadata-response"
  "220592dc9786e0034a883b2dab270692")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CreateMetadata-response>)))
  "Returns full string definition for message of type '<CreateMetadata-response>"
  (cl:format cl:nil "string msg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CreateMetadata-response)))
  "Returns full string definition for message of type 'CreateMetadata-response"
  (cl:format cl:nil "string msg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CreateMetadata-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CreateMetadata-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CreateMetadata-response
    (cl:cons ':msg (msg msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CreateMetadata)))
  'CreateMetadata-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CreateMetadata)))
  'CreateMetadata-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CreateMetadata)))
  "Returns string type for a service object of type '<CreateMetadata>"
  "learning_machine/CreateMetadata")