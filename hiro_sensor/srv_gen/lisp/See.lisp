; Auto-generated. Do not edit!


(cl:in-package hiro_sensor-srv)


;//! \htmlinclude See-request.msg.html

(cl:defclass <See-request> (roslisp-msg-protocol:ros-message)
  ((rerun
    :reader rerun
    :initarg :rerun
    :type cl:boolean
    :initform cl:nil)
   (path
    :reader path
    :initarg :path
    :type cl:string
    :initform "")
   (n_movable_object
    :reader n_movable_object
    :initarg :n_movable_object
    :type cl:fixnum
    :initform 0)
   (n_vase
    :reader n_vase
    :initarg :n_vase
    :type cl:fixnum
    :initform 0)
   (randomized_vase
    :reader randomized_vase
    :initarg :randomized_vase
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass See-request (<See-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <See-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'See-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hiro_sensor-srv:<See-request> is deprecated: use hiro_sensor-srv:See-request instead.")))

(cl:ensure-generic-function 'rerun-val :lambda-list '(m))
(cl:defmethod rerun-val ((m <See-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hiro_sensor-srv:rerun-val is deprecated.  Use hiro_sensor-srv:rerun instead.")
  (rerun m))

(cl:ensure-generic-function 'path-val :lambda-list '(m))
(cl:defmethod path-val ((m <See-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hiro_sensor-srv:path-val is deprecated.  Use hiro_sensor-srv:path instead.")
  (path m))

(cl:ensure-generic-function 'n_movable_object-val :lambda-list '(m))
(cl:defmethod n_movable_object-val ((m <See-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hiro_sensor-srv:n_movable_object-val is deprecated.  Use hiro_sensor-srv:n_movable_object instead.")
  (n_movable_object m))

(cl:ensure-generic-function 'n_vase-val :lambda-list '(m))
(cl:defmethod n_vase-val ((m <See-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hiro_sensor-srv:n_vase-val is deprecated.  Use hiro_sensor-srv:n_vase instead.")
  (n_vase m))

(cl:ensure-generic-function 'randomized_vase-val :lambda-list '(m))
(cl:defmethod randomized_vase-val ((m <See-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hiro_sensor-srv:randomized_vase-val is deprecated.  Use hiro_sensor-srv:randomized_vase instead.")
  (randomized_vase m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <See-request>) ostream)
  "Serializes a message object of type '<See-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'rerun) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'path))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'path))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'n_movable_object)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'n_movable_object)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'n_vase)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'n_vase)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'randomized_vase) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <See-request>) istream)
  "Deserializes a message object of type '<See-request>"
    (cl:setf (cl:slot-value msg 'rerun) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'path) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'path) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'n_movable_object)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'n_movable_object)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'n_vase)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'n_vase)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'randomized_vase) (cl:not (cl:zerop (cl:read-byte istream))))
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
  "bbe84daac54043cac37fe08e055733cf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'See-request)))
  "Returns md5sum for a message object of type 'See-request"
  "bbe84daac54043cac37fe08e055733cf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<See-request>)))
  "Returns full string definition for message of type '<See-request>"
  (cl:format cl:nil "bool rerun~%string path~%uint16 n_movable_object~%uint16 n_vase~%bool randomized_vase~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'See-request)))
  "Returns full string definition for message of type 'See-request"
  (cl:format cl:nil "bool rerun~%string path~%uint16 n_movable_object~%uint16 n_vase~%bool randomized_vase~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <See-request>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'path))
     2
     2
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <See-request>))
  "Converts a ROS message object to a list"
  (cl:list 'See-request
    (cl:cons ':rerun (rerun msg))
    (cl:cons ':path (path msg))
    (cl:cons ':n_movable_object (n_movable_object msg))
    (cl:cons ':n_vase (n_vase msg))
    (cl:cons ':randomized_vase (randomized_vase msg))
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
  "bbe84daac54043cac37fe08e055733cf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'See-response)))
  "Returns md5sum for a message object of type 'See-response"
  "bbe84daac54043cac37fe08e055733cf")
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