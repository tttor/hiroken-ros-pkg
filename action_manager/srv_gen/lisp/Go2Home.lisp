; Auto-generated. Do not edit!


(cl:in-package action_manager-srv)


;//! \htmlinclude Go2Home-request.msg.html

(cl:defclass <Go2Home-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Go2Home-request (<Go2Home-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Go2Home-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Go2Home-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name action_manager-srv:<Go2Home-request> is deprecated: use action_manager-srv:Go2Home-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Go2Home-request>) ostream)
  "Serializes a message object of type '<Go2Home-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Go2Home-request>) istream)
  "Deserializes a message object of type '<Go2Home-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Go2Home-request>)))
  "Returns string type for a service object of type '<Go2Home-request>"
  "action_manager/Go2HomeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Go2Home-request)))
  "Returns string type for a service object of type 'Go2Home-request"
  "action_manager/Go2HomeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Go2Home-request>)))
  "Returns md5sum for a message object of type '<Go2Home-request>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Go2Home-request)))
  "Returns md5sum for a message object of type 'Go2Home-request"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Go2Home-request>)))
  "Returns full string definition for message of type '<Go2Home-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Go2Home-request)))
  "Returns full string definition for message of type 'Go2Home-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Go2Home-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Go2Home-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Go2Home-request
))
;//! \htmlinclude Go2Home-response.msg.html

(cl:defclass <Go2Home-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Go2Home-response (<Go2Home-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Go2Home-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Go2Home-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name action_manager-srv:<Go2Home-response> is deprecated: use action_manager-srv:Go2Home-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Go2Home-response>) ostream)
  "Serializes a message object of type '<Go2Home-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Go2Home-response>) istream)
  "Deserializes a message object of type '<Go2Home-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Go2Home-response>)))
  "Returns string type for a service object of type '<Go2Home-response>"
  "action_manager/Go2HomeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Go2Home-response)))
  "Returns string type for a service object of type 'Go2Home-response"
  "action_manager/Go2HomeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Go2Home-response>)))
  "Returns md5sum for a message object of type '<Go2Home-response>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Go2Home-response)))
  "Returns md5sum for a message object of type 'Go2Home-response"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Go2Home-response>)))
  "Returns full string definition for message of type '<Go2Home-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Go2Home-response)))
  "Returns full string definition for message of type 'Go2Home-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Go2Home-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Go2Home-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Go2Home-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Go2Home)))
  'Go2Home-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Go2Home)))
  'Go2Home-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Go2Home)))
  "Returns string type for a service object of type '<Go2Home>"
  "action_manager/Go2Home")