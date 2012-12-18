; Auto-generated. Do not edit!


(cl:in-package learning_machine-srv)


;//! \htmlinclude Train-request.msg.html

(cl:defclass <Train-request> (roslisp-msg-protocol:ros-message)
  ((tmm_paths
    :reader tmm_paths
    :initarg :tmm_paths
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass Train-request (<Train-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Train-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Train-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name learning_machine-srv:<Train-request> is deprecated: use learning_machine-srv:Train-request instead.")))

(cl:ensure-generic-function 'tmm_paths-val :lambda-list '(m))
(cl:defmethod tmm_paths-val ((m <Train-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader learning_machine-srv:tmm_paths-val is deprecated.  Use learning_machine-srv:tmm_paths instead.")
  (tmm_paths m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Train-request>) ostream)
  "Serializes a message object of type '<Train-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'tmm_paths))))
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
   (cl:slot-value msg 'tmm_paths))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Train-request>) istream)
  "Deserializes a message object of type '<Train-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'tmm_paths) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'tmm_paths)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Train-request>)))
  "Returns string type for a service object of type '<Train-request>"
  "learning_machine/TrainRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Train-request)))
  "Returns string type for a service object of type 'Train-request"
  "learning_machine/TrainRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Train-request>)))
  "Returns md5sum for a message object of type '<Train-request>"
  "a4e82f149495f2916fe2470fb0fe41f8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Train-request)))
  "Returns md5sum for a message object of type 'Train-request"
  "a4e82f149495f2916fe2470fb0fe41f8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Train-request>)))
  "Returns full string definition for message of type '<Train-request>"
  (cl:format cl:nil "string[] tmm_paths~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Train-request)))
  "Returns full string definition for message of type 'Train-request"
  (cl:format cl:nil "string[] tmm_paths~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Train-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'tmm_paths) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Train-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Train-request
    (cl:cons ':tmm_paths (tmm_paths msg))
))
;//! \htmlinclude Train-response.msg.html

(cl:defclass <Train-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Train-response (<Train-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Train-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Train-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name learning_machine-srv:<Train-response> is deprecated: use learning_machine-srv:Train-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Train-response>) ostream)
  "Serializes a message object of type '<Train-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Train-response>) istream)
  "Deserializes a message object of type '<Train-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Train-response>)))
  "Returns string type for a service object of type '<Train-response>"
  "learning_machine/TrainResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Train-response)))
  "Returns string type for a service object of type 'Train-response"
  "learning_machine/TrainResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Train-response>)))
  "Returns md5sum for a message object of type '<Train-response>"
  "a4e82f149495f2916fe2470fb0fe41f8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Train-response)))
  "Returns md5sum for a message object of type 'Train-response"
  "a4e82f149495f2916fe2470fb0fe41f8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Train-response>)))
  "Returns full string definition for message of type '<Train-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Train-response)))
  "Returns full string definition for message of type 'Train-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Train-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Train-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Train-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Train)))
  'Train-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Train)))
  'Train-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Train)))
  "Returns string type for a service object of type '<Train>"
  "learning_machine/Train")