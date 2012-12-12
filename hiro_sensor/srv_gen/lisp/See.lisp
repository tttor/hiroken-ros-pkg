; Auto-generated. Do not edit!


(cl:in-package hiro_sensor-srv)


;//! \htmlinclude See-request.msg.html

(cl:defclass <See-request> (roslisp-msg-protocol:ros-message)
  ((n
    :reader n
    :initarg :n
    :type cl:fixnum
    :initform 0)
   (random
    :reader random
    :initarg :random
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass See-request (<See-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <See-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'See-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hiro_sensor-srv:<See-request> is deprecated: use hiro_sensor-srv:See-request instead.")))

(cl:ensure-generic-function 'n-val :lambda-list '(m))
(cl:defmethod n-val ((m <See-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hiro_sensor-srv:n-val is deprecated.  Use hiro_sensor-srv:n instead.")
  (n m))

(cl:ensure-generic-function 'random-val :lambda-list '(m))
(cl:defmethod random-val ((m <See-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hiro_sensor-srv:random-val is deprecated.  Use hiro_sensor-srv:random instead.")
  (random m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <See-request>) ostream)
  "Serializes a message object of type '<See-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'n)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'n)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'random) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <See-request>) istream)
  "Deserializes a message object of type '<See-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'n)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'n)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'random) (cl:not (cl:zerop (cl:read-byte istream))))
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
  "9803dc42d7ec8be72013e5dbb53b26e1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'See-request)))
  "Returns md5sum for a message object of type 'See-request"
  "9803dc42d7ec8be72013e5dbb53b26e1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<See-request>)))
  "Returns full string definition for message of type '<See-request>"
  (cl:format cl:nil "uint16 n~%bool random~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'See-request)))
  "Returns full string definition for message of type 'See-request"
  (cl:format cl:nil "uint16 n~%bool random~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <See-request>))
  (cl:+ 0
     2
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <See-request>))
  "Converts a ROS message object to a list"
  (cl:list 'See-request
    (cl:cons ':n (n msg))
    (cl:cons ':random (random msg))
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
  "9803dc42d7ec8be72013e5dbb53b26e1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'See-response)))
  "Returns md5sum for a message object of type 'See-response"
  "9803dc42d7ec8be72013e5dbb53b26e1")
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