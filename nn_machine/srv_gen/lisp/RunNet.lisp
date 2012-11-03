; Auto-generated. Do not edit!


(cl:in-package nn_machine-srv)


;//! \htmlinclude RunNet-request.msg.html

(cl:defclass <RunNet-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type (cl:vector nn_machine-msg:Feature)
   :initform (cl:make-array 0 :element-type 'nn_machine-msg:Feature :initial-element (cl:make-instance 'nn_machine-msg:Feature))))
)

(cl:defclass RunNet-request (<RunNet-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RunNet-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RunNet-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nn_machine-srv:<RunNet-request> is deprecated: use nn_machine-srv:RunNet-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <RunNet-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nn_machine-srv:input-val is deprecated.  Use nn_machine-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RunNet-request>) ostream)
  "Serializes a message object of type '<RunNet-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'input))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'input))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RunNet-request>) istream)
  "Deserializes a message object of type '<RunNet-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'input) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'input)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'nn_machine-msg:Feature))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RunNet-request>)))
  "Returns string type for a service object of type '<RunNet-request>"
  "nn_machine/RunNetRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RunNet-request)))
  "Returns string type for a service object of type 'RunNet-request"
  "nn_machine/RunNetRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RunNet-request>)))
  "Returns md5sum for a message object of type '<RunNet-request>"
  "8ab244ff7e826903d7f666bab8196504")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RunNet-request)))
  "Returns md5sum for a message object of type 'RunNet-request"
  "8ab244ff7e826903d7f666bab8196504")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RunNet-request>)))
  "Returns full string definition for message of type '<RunNet-request>"
  (cl:format cl:nil "nn_machine/Feature[] input~%~%================================================================================~%MSG: nn_machine/Feature~%string key~%float64 val~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RunNet-request)))
  "Returns full string definition for message of type 'RunNet-request"
  (cl:format cl:nil "nn_machine/Feature[] input~%~%================================================================================~%MSG: nn_machine/Feature~%string key~%float64 val~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RunNet-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'input) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RunNet-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RunNet-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude RunNet-response.msg.html

(cl:defclass <RunNet-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type cl:float
    :initform 0.0))
)

(cl:defclass RunNet-response (<RunNet-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RunNet-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RunNet-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nn_machine-srv:<RunNet-response> is deprecated: use nn_machine-srv:RunNet-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <RunNet-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nn_machine-srv:output-val is deprecated.  Use nn_machine-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RunNet-response>) ostream)
  "Serializes a message object of type '<RunNet-response>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'output))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RunNet-response>) istream)
  "Deserializes a message object of type '<RunNet-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'output) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RunNet-response>)))
  "Returns string type for a service object of type '<RunNet-response>"
  "nn_machine/RunNetResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RunNet-response)))
  "Returns string type for a service object of type 'RunNet-response"
  "nn_machine/RunNetResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RunNet-response>)))
  "Returns md5sum for a message object of type '<RunNet-response>"
  "8ab244ff7e826903d7f666bab8196504")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RunNet-response)))
  "Returns md5sum for a message object of type 'RunNet-response"
  "8ab244ff7e826903d7f666bab8196504")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RunNet-response>)))
  "Returns full string definition for message of type '<RunNet-response>"
  (cl:format cl:nil "float64 output~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RunNet-response)))
  "Returns full string definition for message of type 'RunNet-response"
  (cl:format cl:nil "float64 output~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RunNet-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RunNet-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RunNet-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RunNet)))
  'RunNet-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RunNet)))
  'RunNet-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RunNet)))
  "Returns string type for a service object of type '<RunNet>"
  "nn_machine/RunNet")