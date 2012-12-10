; Auto-generated. Do not edit!


(cl:in-package learning_machine-srv)


;//! \htmlinclude Test-request.msg.html

(cl:defclass <Test-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Test-request (<Test-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Test-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Test-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name learning_machine-srv:<Test-request> is deprecated: use learning_machine-srv:Test-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Test-request>) ostream)
  "Serializes a message object of type '<Test-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Test-request>) istream)
  "Deserializes a message object of type '<Test-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Test-request>)))
  "Returns string type for a service object of type '<Test-request>"
  "learning_machine/TestRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Test-request)))
  "Returns string type for a service object of type 'Test-request"
  "learning_machine/TestRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Test-request>)))
  "Returns md5sum for a message object of type '<Test-request>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Test-request)))
  "Returns md5sum for a message object of type 'Test-request"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Test-request>)))
  "Returns full string definition for message of type '<Test-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Test-request)))
  "Returns full string definition for message of type 'Test-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Test-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Test-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Test-request
))
;//! \htmlinclude Test-response.msg.html

(cl:defclass <Test-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Test-response (<Test-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Test-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Test-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name learning_machine-srv:<Test-response> is deprecated: use learning_machine-srv:Test-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Test-response>) ostream)
  "Serializes a message object of type '<Test-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Test-response>) istream)
  "Deserializes a message object of type '<Test-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Test-response>)))
  "Returns string type for a service object of type '<Test-response>"
  "learning_machine/TestResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Test-response)))
  "Returns string type for a service object of type 'Test-response"
  "learning_machine/TestResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Test-response>)))
  "Returns md5sum for a message object of type '<Test-response>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Test-response)))
  "Returns md5sum for a message object of type 'Test-response"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Test-response>)))
  "Returns full string definition for message of type '<Test-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Test-response)))
  "Returns full string definition for message of type 'Test-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Test-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Test-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Test-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Test)))
  'Test-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Test)))
  'Test-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Test)))
  "Returns string type for a service object of type '<Test>"
  "learning_machine/Test")