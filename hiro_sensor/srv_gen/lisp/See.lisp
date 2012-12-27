; Auto-generated. Do not edit!


(cl:in-package hiro_sensor-srv)


;//! \htmlinclude See-request.msg.html

(cl:defclass <See-request> (roslisp-msg-protocol:ros-message)
  ((random
    :reader random
    :initarg :random
    :type cl:boolean
    :initform cl:nil)
   (n
    :reader n
    :initarg :n
    :type cl:fixnum
    :initform 0)
   (path
    :reader path
    :initarg :path
    :type cl:string
    :initform ""))
)

(cl:defclass See-request (<See-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <See-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'See-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hiro_sensor-srv:<See-request> is deprecated: use hiro_sensor-srv:See-request instead.")))

(cl:ensure-generic-function 'random-val :lambda-list '(m))
(cl:defmethod random-val ((m <See-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hiro_sensor-srv:random-val is deprecated.  Use hiro_sensor-srv:random instead.")
  (random m))

(cl:ensure-generic-function 'n-val :lambda-list '(m))
(cl:defmethod n-val ((m <See-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hiro_sensor-srv:n-val is deprecated.  Use hiro_sensor-srv:n instead.")
  (n m))

(cl:ensure-generic-function 'path-val :lambda-list '(m))
(cl:defmethod path-val ((m <See-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hiro_sensor-srv:path-val is deprecated.  Use hiro_sensor-srv:path instead.")
  (path m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <See-request>) ostream)
  "Serializes a message object of type '<See-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'random) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'n)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'n)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'path))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'path))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <See-request>) istream)
  "Deserializes a message object of type '<See-request>"
    (cl:setf (cl:slot-value msg 'random) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'n)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'n)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'path) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'path) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<See-request>)))
  "Returns string type for a service object of type '<See-request>"
  "hiro_sensor/SeeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'See-request)))
  "Returns string type for a service object of type 'See-request"
  "hiro_sensor/SeeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<See-request>)))
  "Returns md5sum for a message object of type '<See-request>"
  "2a0590a7ed91113b12ac2f7d51f68332")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'See-request)))
  "Returns md5sum for a message object of type 'See-request"
  "2a0590a7ed91113b12ac2f7d51f68332")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<See-request>)))
  "Returns full string definition for message of type '<See-request>"
  (cl:format cl:nil "bool random~%uint16 n~%string path~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'See-request)))
  "Returns full string definition for message of type 'See-request"
  (cl:format cl:nil "bool random~%uint16 n~%string path~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <See-request>))
  (cl:+ 0
     1
     2
     4 (cl:length (cl:slot-value msg 'path))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <See-request>))
  "Converts a ROS message object to a list"
  (cl:list 'See-request
    (cl:cons ':random (random msg))
    (cl:cons ':n (n msg))
    (cl:cons ':path (path msg))
))
;//! \htmlinclude See-response.msg.html

(cl:defclass <See-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass See-response (<See-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <See-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'See-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hiro_sensor-srv:<See-response> is deprecated: use hiro_sensor-srv:See-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <See-response>) ostream)
  "Serializes a message object of type '<See-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <See-response>) istream)
  "Deserializes a message object of type '<See-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<See-response>)))
  "Returns string type for a service object of type '<See-response>"
  "hiro_sensor/SeeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'See-response)))
  "Returns string type for a service object of type 'See-response"
  "hiro_sensor/SeeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<See-response>)))
  "Returns md5sum for a message object of type '<See-response>"
  "2a0590a7ed91113b12ac2f7d51f68332")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'See-response)))
  "Returns md5sum for a message object of type 'See-response"
  "2a0590a7ed91113b12ac2f7d51f68332")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<See-response>)))
  "Returns full string definition for message of type '<See-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'See-response)))
  "Returns full string definition for message of type 'See-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <See-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <See-response>))
  "Converts a ROS message object to a list"
  (cl:list 'See-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'See)))
  'See-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'See)))
  'See-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'See)))
  "Returns string type for a service object of type '<See>"
  "hiro_sensor/See")