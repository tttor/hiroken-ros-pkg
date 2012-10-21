; Auto-generated. Do not edit!


(cl:in-package hiro_common-msg)


;//! \htmlinclude PathQuality.msg.html

(cl:defclass <PathQuality> (roslisp-msg-protocol:ros-message)
  ((length
    :reader length
    :initarg :length
    :type cl:float
    :initform 0.0)
   (smoothness
    :reader smoothness
    :initarg :smoothness
    :type cl:float
    :initform 0.0))
)

(cl:defclass PathQuality (<PathQuality>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PathQuality>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PathQuality)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hiro_common-msg:<PathQuality> is deprecated: use hiro_common-msg:PathQuality instead.")))

(cl:ensure-generic-function 'length-val :lambda-list '(m))
(cl:defmethod length-val ((m <PathQuality>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hiro_common-msg:length-val is deprecated.  Use hiro_common-msg:length instead.")
  (length m))

(cl:ensure-generic-function 'smoothness-val :lambda-list '(m))
(cl:defmethod smoothness-val ((m <PathQuality>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hiro_common-msg:smoothness-val is deprecated.  Use hiro_common-msg:smoothness instead.")
  (smoothness m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PathQuality>) ostream)
  "Serializes a message object of type '<PathQuality>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'length))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'smoothness))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PathQuality>) istream)
  "Deserializes a message object of type '<PathQuality>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'length) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'smoothness) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PathQuality>)))
  "Returns string type for a message object of type '<PathQuality>"
  "hiro_common/PathQuality")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PathQuality)))
  "Returns string type for a message object of type 'PathQuality"
  "hiro_common/PathQuality")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PathQuality>)))
  "Returns md5sum for a message object of type '<PathQuality>"
  "75a4d7ec806b8bbee52e2ce9b2d1ee1e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PathQuality)))
  "Returns md5sum for a message object of type 'PathQuality"
  "75a4d7ec806b8bbee52e2ce9b2d1ee1e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PathQuality>)))
  "Returns full string definition for message of type '<PathQuality>"
  (cl:format cl:nil "float64 length~%float64 smoothness~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PathQuality)))
  "Returns full string definition for message of type 'PathQuality"
  (cl:format cl:nil "float64 length~%float64 smoothness~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PathQuality>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PathQuality>))
  "Converts a ROS message object to a list"
  (cl:list 'PathQuality
    (cl:cons ':length (length msg))
    (cl:cons ':smoothness (smoothness msg))
))
