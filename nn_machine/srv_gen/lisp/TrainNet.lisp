; Auto-generated. Do not edit!


(cl:in-package nn_machine-srv)


;//! \htmlinclude TrainNet-request.msg.html

(cl:defclass <TrainNet-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass TrainNet-request (<TrainNet-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrainNet-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrainNet-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nn_machine-srv:<TrainNet-request> is deprecated: use nn_machine-srv:TrainNet-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrainNet-request>) ostream)
  "Serializes a message object of type '<TrainNet-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrainNet-request>) istream)
  "Deserializes a message object of type '<TrainNet-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrainNet-request>)))
  "Returns string type for a service object of type '<TrainNet-request>"
  "nn_machine/TrainNetRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrainNet-request)))
  "Returns string type for a service object of type 'TrainNet-request"
  "nn_machine/TrainNetRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrainNet-request>)))
  "Returns md5sum for a message object of type '<TrainNet-request>"
  "7d96ed730776804754140b85e64c862e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrainNet-request)))
  "Returns md5sum for a message object of type 'TrainNet-request"
  "7d96ed730776804754140b85e64c862e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrainNet-request>)))
  "Returns full string definition for message of type '<TrainNet-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrainNet-request)))
  "Returns full string definition for message of type 'TrainNet-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrainNet-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrainNet-request>))
  "Converts a ROS message object to a list"
  (cl:list 'TrainNet-request
))
;//! \htmlinclude TrainNet-response.msg.html

(cl:defclass <TrainNet-response> (roslisp-msg-protocol:ros-message)
  ((msg
    :reader msg
    :initarg :msg
    :type cl:string
    :initform ""))
)

(cl:defclass TrainNet-response (<TrainNet-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrainNet-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrainNet-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nn_machine-srv:<TrainNet-response> is deprecated: use nn_machine-srv:TrainNet-response instead.")))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <TrainNet-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nn_machine-srv:msg-val is deprecated.  Use nn_machine-srv:msg instead.")
  (msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrainNet-response>) ostream)
  "Serializes a message object of type '<TrainNet-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'msg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'msg))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrainNet-response>) istream)
  "Deserializes a message object of type '<TrainNet-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrainNet-response>)))
  "Returns string type for a service object of type '<TrainNet-response>"
  "nn_machine/TrainNetResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrainNet-response)))
  "Returns string type for a service object of type 'TrainNet-response"
  "nn_machine/TrainNetResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrainNet-response>)))
  "Returns md5sum for a message object of type '<TrainNet-response>"
  "7d96ed730776804754140b85e64c862e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrainNet-response)))
  "Returns md5sum for a message object of type 'TrainNet-response"
  "7d96ed730776804754140b85e64c862e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrainNet-response>)))
  "Returns full string definition for message of type '<TrainNet-response>"
  (cl:format cl:nil "string msg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrainNet-response)))
  "Returns full string definition for message of type 'TrainNet-response"
  (cl:format cl:nil "string msg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrainNet-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrainNet-response>))
  "Converts a ROS message object to a list"
  (cl:list 'TrainNet-response
    (cl:cons ':msg (msg msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'TrainNet)))
  'TrainNet-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'TrainNet)))
  'TrainNet-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrainNet)))
  "Returns string type for a service object of type '<TrainNet>"
  "nn_machine/TrainNet")