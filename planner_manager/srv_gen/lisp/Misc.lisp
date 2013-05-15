; Auto-generated. Do not edit!


(cl:in-package planner_manager-srv)


;//! \htmlinclude Misc-request.msg.html

(cl:defclass <Misc-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Misc-request (<Misc-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Misc-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Misc-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name planner_manager-srv:<Misc-request> is deprecated: use planner_manager-srv:Misc-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Misc-request>) ostream)
  "Serializes a message object of type '<Misc-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Misc-request>) istream)
  "Deserializes a message object of type '<Misc-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Misc-request>)))
  "Returns string type for a service object of type '<Misc-request>"
  "planner_manager/MiscRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Misc-request)))
  "Returns string type for a service object of type 'Misc-request"
  "planner_manager/MiscRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Misc-request>)))
  "Returns md5sum for a message object of type '<Misc-request>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Misc-request)))
  "Returns md5sum for a message object of type 'Misc-request"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Misc-request>)))
  "Returns full string definition for message of type '<Misc-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Misc-request)))
  "Returns full string definition for message of type 'Misc-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Misc-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Misc-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Misc-request
))
;//! \htmlinclude Misc-response.msg.html

(cl:defclass <Misc-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Misc-response (<Misc-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Misc-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Misc-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name planner_manager-srv:<Misc-response> is deprecated: use planner_manager-srv:Misc-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Misc-response>) ostream)
  "Serializes a message object of type '<Misc-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Misc-response>) istream)
  "Deserializes a message object of type '<Misc-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Misc-response>)))
  "Returns string type for a service object of type '<Misc-response>"
  "planner_manager/MiscResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Misc-response)))
  "Returns string type for a service object of type 'Misc-response"
  "planner_manager/MiscResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Misc-response>)))
  "Returns md5sum for a message object of type '<Misc-response>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Misc-response)))
  "Returns md5sum for a message object of type 'Misc-response"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Misc-response>)))
  "Returns full string definition for message of type '<Misc-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Misc-response)))
  "Returns full string definition for message of type 'Misc-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Misc-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Misc-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Misc-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Misc)))
  'Misc-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Misc)))
  'Misc-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Misc)))
  "Returns string type for a service object of type '<Misc>"
  "planner_manager/Misc")