; Auto-generated. Do not edit!


(cl:in-package learning_machine-srv)


;//! \htmlinclude CreateMetadata-request.msg.html

(cl:defclass <CreateMetadata-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass CreateMetadata-request (<CreateMetadata-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CreateMetadata-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CreateMetadata-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name learning_machine-srv:<CreateMetadata-request> is deprecated: use learning_machine-srv:CreateMetadata-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CreateMetadata-request>) ostream)
  "Serializes a message object of type '<CreateMetadata-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CreateMetadata-request>) istream)
  "Deserializes a message object of type '<CreateMetadata-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CreateMetadata-request>)))
  "Returns string type for a service object of type '<CreateMetadata-request>"
  "learning_machine/CreateMetadataRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CreateMetadata-request)))
  "Returns string type for a service object of type 'CreateMetadata-request"
  "learning_machine/CreateMetadataRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CreateMetadata-request>)))
  "Returns md5sum for a message object of type '<CreateMetadata-request>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CreateMetadata-request)))
  "Returns md5sum for a message object of type 'CreateMetadata-request"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CreateMetadata-request>)))
  "Returns full string definition for message of type '<CreateMetadata-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CreateMetadata-request)))
  "Returns full string definition for message of type 'CreateMetadata-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CreateMetadata-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CreateMetadata-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CreateMetadata-request
))
;//! \htmlinclude CreateMetadata-response.msg.html

(cl:defclass <CreateMetadata-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass CreateMetadata-response (<CreateMetadata-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CreateMetadata-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CreateMetadata-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name learning_machine-srv:<CreateMetadata-response> is deprecated: use learning_machine-srv:CreateMetadata-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CreateMetadata-response>) ostream)
  "Serializes a message object of type '<CreateMetadata-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CreateMetadata-response>) istream)
  "Deserializes a message object of type '<CreateMetadata-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CreateMetadata-response>)))
  "Returns string type for a service object of type '<CreateMetadata-response>"
  "learning_machine/CreateMetadataResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CreateMetadata-response)))
  "Returns string type for a service object of type 'CreateMetadata-response"
  "learning_machine/CreateMetadataResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CreateMetadata-response>)))
  "Returns md5sum for a message object of type '<CreateMetadata-response>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CreateMetadata-response)))
  "Returns md5sum for a message object of type 'CreateMetadata-response"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CreateMetadata-response>)))
  "Returns full string definition for message of type '<CreateMetadata-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CreateMetadata-response)))
  "Returns full string definition for message of type 'CreateMetadata-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CreateMetadata-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CreateMetadata-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CreateMetadata-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CreateMetadata)))
  'CreateMetadata-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CreateMetadata)))
  'CreateMetadata-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CreateMetadata)))
  "Returns string type for a service object of type '<CreateMetadata>"
  "learning_machine/CreateMetadata")