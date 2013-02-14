; Auto-generated. Do not edit!


(cl:in-package ws_anal-srv)


;//! \htmlinclude EvalWS-request.msg.html

(cl:defclass <EvalWS-request> (roslisp-msg-protocol:ros-message)
  ((rbt_id
    :reader rbt_id
    :initarg :rbt_id
    :type cl:string
    :initform "")
   (jspace
    :reader jspace
    :initarg :jspace
    :type cl:string
    :initform "")
   (path
    :reader path
    :initarg :path
    :type cl:string
    :initform ""))
)

(cl:defclass EvalWS-request (<EvalWS-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EvalWS-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EvalWS-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ws_anal-srv:<EvalWS-request> is deprecated: use ws_anal-srv:EvalWS-request instead.")))

(cl:ensure-generic-function 'rbt_id-val :lambda-list '(m))
(cl:defmethod rbt_id-val ((m <EvalWS-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ws_anal-srv:rbt_id-val is deprecated.  Use ws_anal-srv:rbt_id instead.")
  (rbt_id m))

(cl:ensure-generic-function 'jspace-val :lambda-list '(m))
(cl:defmethod jspace-val ((m <EvalWS-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ws_anal-srv:jspace-val is deprecated.  Use ws_anal-srv:jspace instead.")
  (jspace m))

(cl:ensure-generic-function 'path-val :lambda-list '(m))
(cl:defmethod path-val ((m <EvalWS-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ws_anal-srv:path-val is deprecated.  Use ws_anal-srv:path instead.")
  (path m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EvalWS-request>) ostream)
  "Serializes a message object of type '<EvalWS-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'rbt_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'rbt_id))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'jspace))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'jspace))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'path))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'path))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EvalWS-request>) istream)
  "Deserializes a message object of type '<EvalWS-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'rbt_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'rbt_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'jspace) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'jspace) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EvalWS-request>)))
  "Returns string type for a service object of type '<EvalWS-request>"
  "ws_anal/EvalWSRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EvalWS-request)))
  "Returns string type for a service object of type 'EvalWS-request"
  "ws_anal/EvalWSRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EvalWS-request>)))
  "Returns md5sum for a message object of type '<EvalWS-request>"
  "a329fccc88163dbeb79aa97ba7444f4c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EvalWS-request)))
  "Returns md5sum for a message object of type 'EvalWS-request"
  "a329fccc88163dbeb79aa97ba7444f4c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EvalWS-request>)))
  "Returns full string definition for message of type '<EvalWS-request>"
  (cl:format cl:nil "string rbt_id~%string jspace~%string path~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EvalWS-request)))
  "Returns full string definition for message of type 'EvalWS-request"
  (cl:format cl:nil "string rbt_id~%string jspace~%string path~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EvalWS-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'rbt_id))
     4 (cl:length (cl:slot-value msg 'jspace))
     4 (cl:length (cl:slot-value msg 'path))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EvalWS-request>))
  "Converts a ROS message object to a list"
  (cl:list 'EvalWS-request
    (cl:cons ':rbt_id (rbt_id msg))
    (cl:cons ':jspace (jspace msg))
    (cl:cons ':path (path msg))
))
;//! \htmlinclude EvalWS-response.msg.html

(cl:defclass <EvalWS-response> (roslisp-msg-protocol:ros-message)
  ((msg
    :reader msg
    :initarg :msg
    :type cl:string
    :initform ""))
)

(cl:defclass EvalWS-response (<EvalWS-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EvalWS-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EvalWS-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ws_anal-srv:<EvalWS-response> is deprecated: use ws_anal-srv:EvalWS-response instead.")))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <EvalWS-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ws_anal-srv:msg-val is deprecated.  Use ws_anal-srv:msg instead.")
  (msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EvalWS-response>) ostream)
  "Serializes a message object of type '<EvalWS-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'msg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'msg))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EvalWS-response>) istream)
  "Deserializes a message object of type '<EvalWS-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EvalWS-response>)))
  "Returns string type for a service object of type '<EvalWS-response>"
  "ws_anal/EvalWSResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EvalWS-response)))
  "Returns string type for a service object of type 'EvalWS-response"
  "ws_anal/EvalWSResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EvalWS-response>)))
  "Returns md5sum for a message object of type '<EvalWS-response>"
  "a329fccc88163dbeb79aa97ba7444f4c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EvalWS-response)))
  "Returns md5sum for a message object of type 'EvalWS-response"
  "a329fccc88163dbeb79aa97ba7444f4c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EvalWS-response>)))
  "Returns full string definition for message of type '<EvalWS-response>"
  (cl:format cl:nil "string msg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EvalWS-response)))
  "Returns full string definition for message of type 'EvalWS-response"
  (cl:format cl:nil "string msg~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EvalWS-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EvalWS-response>))
  "Converts a ROS message object to a list"
  (cl:list 'EvalWS-response
    (cl:cons ':msg (msg msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'EvalWS)))
  'EvalWS-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'EvalWS)))
  'EvalWS-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EvalWS)))
  "Returns string type for a service object of type '<EvalWS>"
  "ws_anal/EvalWS")