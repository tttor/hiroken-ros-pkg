; Auto-generated. Do not edit!


(cl:in-package action_manager-srv)


;//! \htmlinclude Commit-request.msg.html

(cl:defclass <Commit-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Commit-request (<Commit-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Commit-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Commit-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name action_manager-srv:<Commit-request> is deprecated: use action_manager-srv:Commit-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Commit-request>) ostream)
  "Serializes a message object of type '<Commit-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Commit-request>) istream)
  "Deserializes a message object of type '<Commit-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Commit-request>)))
  "Returns string type for a service object of type '<Commit-request>"
  "action_manager/CommitRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Commit-request)))
  "Returns string type for a service object of type 'Commit-request"
  "action_manager/CommitRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Commit-request>)))
  "Returns md5sum for a message object of type '<Commit-request>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Commit-request)))
  "Returns md5sum for a message object of type 'Commit-request"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Commit-request>)))
  "Returns full string definition for message of type '<Commit-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Commit-request)))
  "Returns full string definition for message of type 'Commit-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Commit-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Commit-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Commit-request
))
;//! \htmlinclude Commit-response.msg.html

(cl:defclass <Commit-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Commit-response (<Commit-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Commit-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Commit-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name action_manager-srv:<Commit-response> is deprecated: use action_manager-srv:Commit-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Commit-response>) ostream)
  "Serializes a message object of type '<Commit-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Commit-response>) istream)
  "Deserializes a message object of type '<Commit-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Commit-response>)))
  "Returns string type for a service object of type '<Commit-response>"
  "action_manager/CommitResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Commit-response)))
  "Returns string type for a service object of type 'Commit-response"
  "action_manager/CommitResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Commit-response>)))
  "Returns md5sum for a message object of type '<Commit-response>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Commit-response)))
  "Returns md5sum for a message object of type 'Commit-response"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Commit-response>)))
  "Returns full string definition for message of type '<Commit-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Commit-response)))
  "Returns full string definition for message of type 'Commit-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Commit-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Commit-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Commit-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Commit)))
  'Commit-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Commit)))
  'Commit-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Commit)))
  "Returns string type for a service object of type '<Commit>"
  "action_manager/Commit")