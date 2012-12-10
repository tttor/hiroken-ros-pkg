; Auto-generated. Do not edit!


(cl:in-package planner_manager-srv)


;//! \htmlinclude Plan-request.msg.html

(cl:defclass <Plan-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Plan-request (<Plan-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Plan-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Plan-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name planner_manager-srv:<Plan-request> is deprecated: use planner_manager-srv:Plan-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Plan-request>) ostream)
  "Serializes a message object of type '<Plan-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Plan-request>) istream)
  "Deserializes a message object of type '<Plan-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Plan-request>)))
  "Returns string type for a service object of type '<Plan-request>"
  "planner_manager/PlanRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Plan-request)))
  "Returns string type for a service object of type 'Plan-request"
  "planner_manager/PlanRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Plan-request>)))
  "Returns md5sum for a message object of type '<Plan-request>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Plan-request)))
  "Returns md5sum for a message object of type 'Plan-request"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Plan-request>)))
  "Returns full string definition for message of type '<Plan-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Plan-request)))
  "Returns full string definition for message of type 'Plan-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Plan-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Plan-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Plan-request
))
;//! \htmlinclude Plan-response.msg.html

(cl:defclass <Plan-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Plan-response (<Plan-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Plan-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Plan-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name planner_manager-srv:<Plan-response> is deprecated: use planner_manager-srv:Plan-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Plan-response>) ostream)
  "Serializes a message object of type '<Plan-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Plan-response>) istream)
  "Deserializes a message object of type '<Plan-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Plan-response>)))
  "Returns string type for a service object of type '<Plan-response>"
  "planner_manager/PlanResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Plan-response)))
  "Returns string type for a service object of type 'Plan-response"
  "planner_manager/PlanResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Plan-response>)))
  "Returns md5sum for a message object of type '<Plan-response>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Plan-response)))
  "Returns md5sum for a message object of type 'Plan-response"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Plan-response>)))
  "Returns full string definition for message of type '<Plan-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Plan-response)))
  "Returns full string definition for message of type 'Plan-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Plan-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Plan-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Plan-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Plan)))
  'Plan-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Plan)))
  'Plan-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Plan)))
  "Returns string type for a service object of type '<Plan>"
  "planner_manager/Plan")