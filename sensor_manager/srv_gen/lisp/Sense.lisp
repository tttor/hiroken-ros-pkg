; Auto-generated. Do not edit!


(cl:in-package sensor_manager-srv)


;//! \htmlinclude Sense-request.msg.html

(cl:defclass <Sense-request> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0)
   (uint_args
    :reader uint_args
    :initarg :uint_args
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (string_args
    :reader string_args
    :initarg :string_args
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass Sense-request (<Sense-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Sense-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Sense-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sensor_manager-srv:<Sense-request> is deprecated: use sensor_manager-srv:Sense-request instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <Sense-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensor_manager-srv:id-val is deprecated.  Use sensor_manager-srv:id instead.")
  (id m))

(cl:ensure-generic-function 'uint_args-val :lambda-list '(m))
(cl:defmethod uint_args-val ((m <Sense-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensor_manager-srv:uint_args-val is deprecated.  Use sensor_manager-srv:uint_args instead.")
  (uint_args m))

(cl:ensure-generic-function 'string_args-val :lambda-list '(m))
(cl:defmethod string_args-val ((m <Sense-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensor_manager-srv:string_args-val is deprecated.  Use sensor_manager-srv:string_args instead.")
  (string_args m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Sense-request>) ostream)
  "Serializes a message object of type '<Sense-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'uint_args))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'uint_args))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'string_args))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'string_args))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Sense-request>) istream)
  "Deserializes a message object of type '<Sense-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'uint_args) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'uint_args)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'string_args) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'string_args)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Sense-request>)))
  "Returns string type for a service object of type '<Sense-request>"
  "sensor_manager/SenseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Sense-request)))
  "Returns string type for a service object of type 'Sense-request"
  "sensor_manager/SenseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Sense-request>)))
  "Returns md5sum for a message object of type '<Sense-request>"
  "321a9d12d35a991430b703db88e8fea8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Sense-request)))
  "Returns md5sum for a message object of type 'Sense-request"
  "321a9d12d35a991430b703db88e8fea8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Sense-request>)))
  "Returns full string definition for message of type '<Sense-request>"
  (cl:format cl:nil "uint16 id~%uint16[] uint_args~%string[] string_args~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Sense-request)))
  "Returns full string definition for message of type 'Sense-request"
  (cl:format cl:nil "uint16 id~%uint16[] uint_args~%string[] string_args~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Sense-request>))
  (cl:+ 0
     2
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'uint_args) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'string_args) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Sense-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Sense-request
    (cl:cons ':id (id msg))
    (cl:cons ':uint_args (uint_args msg))
    (cl:cons ':string_args (string_args msg))
))
;//! \htmlinclude Sense-response.msg.html

(cl:defclass <Sense-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Sense-response (<Sense-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Sense-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Sense-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sensor_manager-srv:<Sense-response> is deprecated: use sensor_manager-srv:Sense-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Sense-response>) ostream)
  "Serializes a message object of type '<Sense-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Sense-response>) istream)
  "Deserializes a message object of type '<Sense-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Sense-response>)))
  "Returns string type for a service object of type '<Sense-response>"
  "sensor_manager/SenseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Sense-response)))
  "Returns string type for a service object of type 'Sense-response"
  "sensor_manager/SenseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Sense-response>)))
  "Returns md5sum for a message object of type '<Sense-response>"
  "321a9d12d35a991430b703db88e8fea8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Sense-response)))
  "Returns md5sum for a message object of type 'Sense-response"
  "321a9d12d35a991430b703db88e8fea8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Sense-response>)))
  "Returns full string definition for message of type '<Sense-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Sense-response)))
  "Returns full string definition for message of type 'Sense-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Sense-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Sense-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Sense-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Sense)))
  'Sense-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Sense)))
  'Sense-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Sense)))
  "Returns string type for a service object of type '<Sense>"
  "sensor_manager/Sense")