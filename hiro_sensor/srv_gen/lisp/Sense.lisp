; Auto-generated. Do not edit!


(cl:in-package hiro_sensor-srv)


;//! \htmlinclude Sense-request.msg.html

(cl:defclass <Sense-request> (roslisp-msg-protocol:ros-message)
  ((sensor_type
    :reader sensor_type
    :initarg :sensor_type
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Sense-request (<Sense-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Sense-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Sense-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hiro_sensor-srv:<Sense-request> is deprecated: use hiro_sensor-srv:Sense-request instead.")))

(cl:ensure-generic-function 'sensor_type-val :lambda-list '(m))
(cl:defmethod sensor_type-val ((m <Sense-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hiro_sensor-srv:sensor_type-val is deprecated.  Use hiro_sensor-srv:sensor_type instead.")
  (sensor_type m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Sense-request>) ostream)
  "Serializes a message object of type '<Sense-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sensor_type)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Sense-request>) istream)
  "Deserializes a message object of type '<Sense-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sensor_type)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Sense-request>)))
  "Returns string type for a service object of type '<Sense-request>"
  "hiro_sensor/SenseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Sense-request)))
  "Returns string type for a service object of type 'Sense-request"
  "hiro_sensor/SenseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Sense-request>)))
  "Returns md5sum for a message object of type '<Sense-request>"
  "b8ffb43dc566d47d2b50541d67ff03ac")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Sense-request)))
  "Returns md5sum for a message object of type 'Sense-request"
  "b8ffb43dc566d47d2b50541d67ff03ac")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Sense-request>)))
  "Returns full string definition for message of type '<Sense-request>"
  (cl:format cl:nil "uint8 sensor_type~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Sense-request)))
  "Returns full string definition for message of type 'Sense-request"
  (cl:format cl:nil "uint8 sensor_type~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Sense-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Sense-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Sense-request
    (cl:cons ':sensor_type (sensor_type msg))
))
;//! \htmlinclude Sense-response.msg.html

(cl:defclass <Sense-response> (roslisp-msg-protocol:ros-message)
  ((msg
    :reader msg
    :initarg :msg
    :type cl:string
    :initform ""))
)

(cl:defclass Sense-response (<Sense-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Sense-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Sense-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hiro_sensor-srv:<Sense-response> is deprecated: use hiro_sensor-srv:Sense-response instead.")))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <Sense-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hiro_sensor-srv:msg-val is deprecated.  Use hiro_sensor-srv:msg instead.")
  (msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Sense-response>) ostream)
  "Serializes a message object of type '<Sense-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'msg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'msg))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Sense-response>) istream)
  "Deserializes a message object of type '<Sense-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Sense-response>)))
  "Returns string type for a service object of type '<Sense-response>"
  "hiro_sensor/SenseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Sense-response)))
  "Returns string type for a service object of type 'Sense-response"
  "hiro_sensor/SenseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Sense-response>)))
  "Returns md5sum for a message object of type '<Sense-response>"
  "b8ffb43dc566d47d2b50541d67ff03ac")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Sense-response)))
  "Returns md5sum for a message object of type 'Sense-response"
  "b8ffb43dc566d47d2b50541d67ff03ac")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Sense-response>)))
  "Returns full string definition for message of type '<Sense-response>"
  (cl:format cl:nil "string msg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Sense-response)))
  "Returns full string definition for message of type 'Sense-response"
  (cl:format cl:nil "string msg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Sense-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Sense-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Sense-response
    (cl:cons ':msg (msg msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Sense)))
  'Sense-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Sense)))
  'Sense-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Sense)))
  "Returns string type for a service object of type '<Sense>"
  "hiro_sensor/Sense")