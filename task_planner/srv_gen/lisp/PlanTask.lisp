; Auto-generated. Do not edit!


(cl:in-package task_planner-srv)


;//! \htmlinclude PlanTask-request.msg.html

(cl:defclass <PlanTask-request> (roslisp-msg-protocol:ros-message)
  ((objects
    :reader objects
    :initarg :objects
    :type (cl:vector arm_navigation_msgs-msg:CollisionObject)
   :initform (cl:make-array 0 :element-type 'arm_navigation_msgs-msg:CollisionObject :initial-element (cl:make-instance 'arm_navigation_msgs-msg:CollisionObject))))
)

(cl:defclass PlanTask-request (<PlanTask-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlanTask-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlanTask-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name task_planner-srv:<PlanTask-request> is deprecated: use task_planner-srv:PlanTask-request instead.")))

(cl:ensure-generic-function 'objects-val :lambda-list '(m))
(cl:defmethod objects-val ((m <PlanTask-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader task_planner-srv:objects-val is deprecated.  Use task_planner-srv:objects instead.")
  (objects m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlanTask-request>) ostream)
  "Serializes a message object of type '<PlanTask-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'objects))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'objects))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlanTask-request>) istream)
  "Deserializes a message object of type '<PlanTask-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'objects) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'objects)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'arm_navigation_msgs-msg:CollisionObject))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlanTask-request>)))
  "Returns string type for a service object of type '<PlanTask-request>"
  "task_planner/PlanTaskRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlanTask-request)))
  "Returns string type for a service object of type 'PlanTask-request"
  "task_planner/PlanTaskRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlanTask-request>)))
  "Returns md5sum for a message object of type '<PlanTask-request>"
  "e14aed90eedaddbd0822c0e675600537")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlanTask-request)))
  "Returns md5sum for a message object of type 'PlanTask-request"
  "e14aed90eedaddbd0822c0e675600537")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlanTask-request>)))
  "Returns full string definition for message of type '<PlanTask-request>"
  (cl:format cl:nil "~%arm_navigation_msgs/CollisionObject[] objects~%~%================================================================================~%MSG: arm_navigation_msgs/CollisionObject~%# a header, used for interpreting the poses~%Header header~%~%# the id of the object~%string id~%~%# The padding used for filtering points near the object.~%# This does not affect collision checking for the object.  ~%# Set to negative to get zero padding.~%float32 padding~%~%#This contains what is to be done with the object~%CollisionObjectOperation operation~%~%#the shapes associated with the object~%arm_navigation_msgs/Shape[] shapes~%~%#the poses associated with the shapes - will be transformed using the header~%geometry_msgs/Pose[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: arm_navigation_msgs/CollisionObjectOperation~%#Puts the object into the environment~%#or updates the object if already added~%byte ADD=0~%~%#Removes the object from the environment entirely~%byte REMOVE=1~%~%#Only valid within the context of a CollisionAttachedObject message~%#Will be ignored if sent with an CollisionObject message~%#Takes an attached object, detaches from the attached link~%#But adds back in as regular object~%byte DETACH_AND_ADD_AS_OBJECT=2~%~%#Only valid within the context of a CollisionAttachedObject message~%#Will be ignored if sent with an CollisionObject message~%#Takes current object in the environment and removes it as~%#a regular object~%byte ATTACH_AND_REMOVE_AS_OBJECT=3~%~%# Byte code for operation~%byte operation~%~%================================================================================~%MSG: arm_navigation_msgs/Shape~%byte SPHERE=0~%byte BOX=1~%byte CYLINDER=2~%byte MESH=3~%~%byte type~%~%~%#### define sphere, box, cylinder ####~%# the origin of each shape is considered at the shape's center~%~%# for sphere~%# radius := dimensions[0]~%~%# for cylinder~%# radius := dimensions[0]~%# length := dimensions[1]~%# the length is along the Z axis~%~%# for box~%# size_x := dimensions[0]~%# size_y := dimensions[1]~%# size_z := dimensions[2]~%float64[] dimensions~%~%~%#### define mesh ####~%~%# list of triangles; triangle k is defined by tre vertices located~%# at indices triangles[3k], triangles[3k+1], triangles[3k+2]~%int32[] triangles~%geometry_msgs/Point[] vertices~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlanTask-request)))
  "Returns full string definition for message of type 'PlanTask-request"
  (cl:format cl:nil "~%arm_navigation_msgs/CollisionObject[] objects~%~%================================================================================~%MSG: arm_navigation_msgs/CollisionObject~%# a header, used for interpreting the poses~%Header header~%~%# the id of the object~%string id~%~%# The padding used for filtering points near the object.~%# This does not affect collision checking for the object.  ~%# Set to negative to get zero padding.~%float32 padding~%~%#This contains what is to be done with the object~%CollisionObjectOperation operation~%~%#the shapes associated with the object~%arm_navigation_msgs/Shape[] shapes~%~%#the poses associated with the shapes - will be transformed using the header~%geometry_msgs/Pose[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: arm_navigation_msgs/CollisionObjectOperation~%#Puts the object into the environment~%#or updates the object if already added~%byte ADD=0~%~%#Removes the object from the environment entirely~%byte REMOVE=1~%~%#Only valid within the context of a CollisionAttachedObject message~%#Will be ignored if sent with an CollisionObject message~%#Takes an attached object, detaches from the attached link~%#But adds back in as regular object~%byte DETACH_AND_ADD_AS_OBJECT=2~%~%#Only valid within the context of a CollisionAttachedObject message~%#Will be ignored if sent with an CollisionObject message~%#Takes current object in the environment and removes it as~%#a regular object~%byte ATTACH_AND_REMOVE_AS_OBJECT=3~%~%# Byte code for operation~%byte operation~%~%================================================================================~%MSG: arm_navigation_msgs/Shape~%byte SPHERE=0~%byte BOX=1~%byte CYLINDER=2~%byte MESH=3~%~%byte type~%~%~%#### define sphere, box, cylinder ####~%# the origin of each shape is considered at the shape's center~%~%# for sphere~%# radius := dimensions[0]~%~%# for cylinder~%# radius := dimensions[0]~%# length := dimensions[1]~%# the length is along the Z axis~%~%# for box~%# size_x := dimensions[0]~%# size_y := dimensions[1]~%# size_z := dimensions[2]~%float64[] dimensions~%~%~%#### define mesh ####~%~%# list of triangles; triangle k is defined by tre vertices located~%# at indices triangles[3k], triangles[3k+1], triangles[3k+2]~%int32[] triangles~%geometry_msgs/Point[] vertices~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlanTask-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'objects) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlanTask-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PlanTask-request
    (cl:cons ':objects (objects msg))
))
;//! \htmlinclude PlanTask-response.msg.html

(cl:defclass <PlanTask-response> (roslisp-msg-protocol:ros-message)
  ((plans
    :reader plans
    :initarg :plans
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass PlanTask-response (<PlanTask-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlanTask-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlanTask-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name task_planner-srv:<PlanTask-response> is deprecated: use task_planner-srv:PlanTask-response instead.")))

(cl:ensure-generic-function 'plans-val :lambda-list '(m))
(cl:defmethod plans-val ((m <PlanTask-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader task_planner-srv:plans-val is deprecated.  Use task_planner-srv:plans instead.")
  (plans m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlanTask-response>) ostream)
  "Serializes a message object of type '<PlanTask-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'plans))))
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
   (cl:slot-value msg 'plans))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlanTask-response>) istream)
  "Deserializes a message object of type '<PlanTask-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'plans) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'plans)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlanTask-response>)))
  "Returns string type for a service object of type '<PlanTask-response>"
  "task_planner/PlanTaskResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlanTask-response)))
  "Returns string type for a service object of type 'PlanTask-response"
  "task_planner/PlanTaskResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlanTask-response>)))
  "Returns md5sum for a message object of type '<PlanTask-response>"
  "e14aed90eedaddbd0822c0e675600537")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlanTask-response)))
  "Returns md5sum for a message object of type 'PlanTask-response"
  "e14aed90eedaddbd0822c0e675600537")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlanTask-response>)))
  "Returns full string definition for message of type '<PlanTask-response>"
  (cl:format cl:nil "string[] plans~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlanTask-response)))
  "Returns full string definition for message of type 'PlanTask-response"
  (cl:format cl:nil "string[] plans~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlanTask-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'plans) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlanTask-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PlanTask-response
    (cl:cons ':plans (plans msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PlanTask)))
  'PlanTask-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PlanTask)))
  'PlanTask-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlanTask)))
  "Returns string type for a service object of type '<PlanTask>"
  "task_planner/PlanTask")