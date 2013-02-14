; Auto-generated. Do not edit!


(cl:in-package grasp_planner-srv)


;//! \htmlinclude PlanGrasp-request.msg.html

(cl:defclass <PlanGrasp-request> (roslisp-msg-protocol:ros-message)
  ((object
    :reader object
    :initarg :object
    :type arm_navigation_msgs-msg:CollisionObject
    :initform (cl:make-instance 'arm_navigation_msgs-msg:CollisionObject))
   (rbt_id
    :reader rbt_id
    :initarg :rbt_id
    :type cl:string
    :initform "")
   (jspace
    :reader jspace
    :initarg :jspace
    :type cl:string
    :initform ""))
)

(cl:defclass PlanGrasp-request (<PlanGrasp-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlanGrasp-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlanGrasp-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name grasp_planner-srv:<PlanGrasp-request> is deprecated: use grasp_planner-srv:PlanGrasp-request instead.")))

(cl:ensure-generic-function 'object-val :lambda-list '(m))
(cl:defmethod object-val ((m <PlanGrasp-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader grasp_planner-srv:object-val is deprecated.  Use grasp_planner-srv:object instead.")
  (object m))

(cl:ensure-generic-function 'rbt_id-val :lambda-list '(m))
(cl:defmethod rbt_id-val ((m <PlanGrasp-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader grasp_planner-srv:rbt_id-val is deprecated.  Use grasp_planner-srv:rbt_id instead.")
  (rbt_id m))

(cl:ensure-generic-function 'jspace-val :lambda-list '(m))
(cl:defmethod jspace-val ((m <PlanGrasp-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader grasp_planner-srv:jspace-val is deprecated.  Use grasp_planner-srv:jspace instead.")
  (jspace m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlanGrasp-request>) ostream)
  "Serializes a message object of type '<PlanGrasp-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'object) ostream)
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlanGrasp-request>) istream)
  "Deserializes a message object of type '<PlanGrasp-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'object) istream)
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlanGrasp-request>)))
  "Returns string type for a service object of type '<PlanGrasp-request>"
  "grasp_planner/PlanGraspRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlanGrasp-request)))
  "Returns string type for a service object of type 'PlanGrasp-request"
  "grasp_planner/PlanGraspRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlanGrasp-request>)))
  "Returns md5sum for a message object of type '<PlanGrasp-request>"
  "c3b61166dd2ad999b94bf9a333f32d7b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlanGrasp-request)))
  "Returns md5sum for a message object of type 'PlanGrasp-request"
  "c3b61166dd2ad999b94bf9a333f32d7b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlanGrasp-request>)))
  "Returns full string definition for message of type '<PlanGrasp-request>"
  (cl:format cl:nil "~%arm_navigation_msgs/CollisionObject object~%string rbt_id~%string jspace~%~%================================================================================~%MSG: arm_navigation_msgs/CollisionObject~%# a header, used for interpreting the poses~%Header header~%~%# the id of the object~%string id~%~%# The padding used for filtering points near the object.~%# This does not affect collision checking for the object.  ~%# Set to negative to get zero padding.~%float32 padding~%~%#This contains what is to be done with the object~%CollisionObjectOperation operation~%~%#the shapes associated with the object~%arm_navigation_msgs/Shape[] shapes~%~%#the poses associated with the shapes - will be transformed using the header~%geometry_msgs/Pose[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: arm_navigation_msgs/CollisionObjectOperation~%#Puts the object into the environment~%#or updates the object if already added~%byte ADD=0~%~%#Removes the object from the environment entirely~%byte REMOVE=1~%~%#Only valid within the context of a CollisionAttachedObject message~%#Will be ignored if sent with an CollisionObject message~%#Takes an attached object, detaches from the attached link~%#But adds back in as regular object~%byte DETACH_AND_ADD_AS_OBJECT=2~%~%#Only valid within the context of a CollisionAttachedObject message~%#Will be ignored if sent with an CollisionObject message~%#Takes current object in the environment and removes it as~%#a regular object~%byte ATTACH_AND_REMOVE_AS_OBJECT=3~%~%# Byte code for operation~%byte operation~%~%================================================================================~%MSG: arm_navigation_msgs/Shape~%byte SPHERE=0~%byte BOX=1~%byte CYLINDER=2~%byte MESH=3~%~%byte type~%~%~%#### define sphere, box, cylinder ####~%# the origin of each shape is considered at the shape's center~%~%# for sphere~%# radius := dimensions[0]~%~%# for cylinder~%# radius := dimensions[0]~%# length := dimensions[1]~%# the length is along the Z axis~%~%# for box~%# size_x := dimensions[0]~%# size_y := dimensions[1]~%# size_z := dimensions[2]~%float64[] dimensions~%~%~%#### define mesh ####~%~%# list of triangles; triangle k is defined by tre vertices located~%# at indices triangles[3k], triangles[3k+1], triangles[3k+2]~%int32[] triangles~%geometry_msgs/Point[] vertices~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlanGrasp-request)))
  "Returns full string definition for message of type 'PlanGrasp-request"
  (cl:format cl:nil "~%arm_navigation_msgs/CollisionObject object~%string rbt_id~%string jspace~%~%================================================================================~%MSG: arm_navigation_msgs/CollisionObject~%# a header, used for interpreting the poses~%Header header~%~%# the id of the object~%string id~%~%# The padding used for filtering points near the object.~%# This does not affect collision checking for the object.  ~%# Set to negative to get zero padding.~%float32 padding~%~%#This contains what is to be done with the object~%CollisionObjectOperation operation~%~%#the shapes associated with the object~%arm_navigation_msgs/Shape[] shapes~%~%#the poses associated with the shapes - will be transformed using the header~%geometry_msgs/Pose[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: arm_navigation_msgs/CollisionObjectOperation~%#Puts the object into the environment~%#or updates the object if already added~%byte ADD=0~%~%#Removes the object from the environment entirely~%byte REMOVE=1~%~%#Only valid within the context of a CollisionAttachedObject message~%#Will be ignored if sent with an CollisionObject message~%#Takes an attached object, detaches from the attached link~%#But adds back in as regular object~%byte DETACH_AND_ADD_AS_OBJECT=2~%~%#Only valid within the context of a CollisionAttachedObject message~%#Will be ignored if sent with an CollisionObject message~%#Takes current object in the environment and removes it as~%#a regular object~%byte ATTACH_AND_REMOVE_AS_OBJECT=3~%~%# Byte code for operation~%byte operation~%~%================================================================================~%MSG: arm_navigation_msgs/Shape~%byte SPHERE=0~%byte BOX=1~%byte CYLINDER=2~%byte MESH=3~%~%byte type~%~%~%#### define sphere, box, cylinder ####~%# the origin of each shape is considered at the shape's center~%~%# for sphere~%# radius := dimensions[0]~%~%# for cylinder~%# radius := dimensions[0]~%# length := dimensions[1]~%# the length is along the Z axis~%~%# for box~%# size_x := dimensions[0]~%# size_y := dimensions[1]~%# size_z := dimensions[2]~%float64[] dimensions~%~%~%#### define mesh ####~%~%# list of triangles; triangle k is defined by tre vertices located~%# at indices triangles[3k], triangles[3k+1], triangles[3k+2]~%int32[] triangles~%geometry_msgs/Point[] vertices~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlanGrasp-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'object))
     4 (cl:length (cl:slot-value msg 'rbt_id))
     4 (cl:length (cl:slot-value msg 'jspace))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlanGrasp-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PlanGrasp-request
    (cl:cons ':object (object msg))
    (cl:cons ':rbt_id (rbt_id msg))
    (cl:cons ':jspace (jspace msg))
))
;//! \htmlinclude PlanGrasp-response.msg.html

(cl:defclass <PlanGrasp-response> (roslisp-msg-protocol:ros-message)
  ((grasp_plans
    :reader grasp_plans
    :initarg :grasp_plans
    :type (cl:vector sensor_msgs-msg:JointState)
   :initform (cl:make-array 0 :element-type 'sensor_msgs-msg:JointState :initial-element (cl:make-instance 'sensor_msgs-msg:JointState)))
   (process_cost
    :reader process_cost
    :initarg :process_cost
    :type cl:float
    :initform 0.0))
)

(cl:defclass PlanGrasp-response (<PlanGrasp-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlanGrasp-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlanGrasp-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name grasp_planner-srv:<PlanGrasp-response> is deprecated: use grasp_planner-srv:PlanGrasp-response instead.")))

(cl:ensure-generic-function 'grasp_plans-val :lambda-list '(m))
(cl:defmethod grasp_plans-val ((m <PlanGrasp-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader grasp_planner-srv:grasp_plans-val is deprecated.  Use grasp_planner-srv:grasp_plans instead.")
  (grasp_plans m))

(cl:ensure-generic-function 'process_cost-val :lambda-list '(m))
(cl:defmethod process_cost-val ((m <PlanGrasp-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader grasp_planner-srv:process_cost-val is deprecated.  Use grasp_planner-srv:process_cost instead.")
  (process_cost m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlanGrasp-response>) ostream)
  "Serializes a message object of type '<PlanGrasp-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'grasp_plans))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'grasp_plans))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'process_cost))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlanGrasp-response>) istream)
  "Deserializes a message object of type '<PlanGrasp-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'grasp_plans) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'grasp_plans)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'sensor_msgs-msg:JointState))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'process_cost) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlanGrasp-response>)))
  "Returns string type for a service object of type '<PlanGrasp-response>"
  "grasp_planner/PlanGraspResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlanGrasp-response)))
  "Returns string type for a service object of type 'PlanGrasp-response"
  "grasp_planner/PlanGraspResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlanGrasp-response>)))
  "Returns md5sum for a message object of type '<PlanGrasp-response>"
  "c3b61166dd2ad999b94bf9a333f32d7b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlanGrasp-response)))
  "Returns md5sum for a message object of type 'PlanGrasp-response"
  "c3b61166dd2ad999b94bf9a333f32d7b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlanGrasp-response>)))
  "Returns full string definition for message of type '<PlanGrasp-response>"
  (cl:format cl:nil "~%sensor_msgs/JointState[] grasp_plans~%float64 process_cost~%~%~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlanGrasp-response)))
  "Returns full string definition for message of type 'PlanGrasp-response"
  (cl:format cl:nil "~%sensor_msgs/JointState[] grasp_plans~%float64 process_cost~%~%~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlanGrasp-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'grasp_plans) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlanGrasp-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PlanGrasp-response
    (cl:cons ':grasp_plans (grasp_plans msg))
    (cl:cons ':process_cost (process_cost msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PlanGrasp)))
  'PlanGrasp-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PlanGrasp)))
  'PlanGrasp-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlanGrasp)))
  "Returns string type for a service object of type '<PlanGrasp>"
  "grasp_planner/PlanGrasp")