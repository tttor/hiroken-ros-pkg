; Auto-generated. Do not edit!


(cl:in-package learning_machine-srv)


;//! \htmlinclude Train-request.msg.html

(cl:defclass <Train-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Train-request (<Train-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Train-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Train-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name learning_machine-srv:<Train-request> is deprecated: use learning_machine-srv:Train-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Train-request>) ostream)
  "Serializes a message object of type '<Train-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Train-request>) istream)
  "Deserializes a message object of type '<Train-request>"
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
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Train-request)))
  "Returns md5sum for a message object of type 'Train-request"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Train-request>)))
  "Returns full string definition for message of type '<Train-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Train-request)))
  "Returns full string definition for message of type 'Train-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Train-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Train-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Train-request
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
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Train-response)))
  "Returns md5sum for a message object of type 'Train-response"
  "d41d8cd98f00b204e9800998ecf8427e")
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