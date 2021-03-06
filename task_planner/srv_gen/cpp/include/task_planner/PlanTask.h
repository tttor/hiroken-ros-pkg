/* Auto-generated by genmsg_cpp for file /home/vektor/hiroken-ros-pkg/task_planner/srv/PlanTask.srv */
#ifndef TASK_PLANNER_SERVICE_PLANTASK_H
#define TASK_PLANNER_SERVICE_PLANTASK_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "ros/service_traits.h"

#include "arm_navigation_msgs/CollisionObject.h"



namespace task_planner
{
template <class ContainerAllocator>
struct PlanTaskRequest_ {
  typedef PlanTaskRequest_<ContainerAllocator> Type;

  PlanTaskRequest_()
  : objects()
  {
  }

  PlanTaskRequest_(const ContainerAllocator& _alloc)
  : objects(_alloc)
  {
  }

  typedef std::vector< ::arm_navigation_msgs::CollisionObject_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::arm_navigation_msgs::CollisionObject_<ContainerAllocator> >::other >  _objects_type;
  std::vector< ::arm_navigation_msgs::CollisionObject_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::arm_navigation_msgs::CollisionObject_<ContainerAllocator> >::other >  objects;


  typedef boost::shared_ptr< ::task_planner::PlanTaskRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::task_planner::PlanTaskRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct PlanTaskRequest
typedef  ::task_planner::PlanTaskRequest_<std::allocator<void> > PlanTaskRequest;

typedef boost::shared_ptr< ::task_planner::PlanTaskRequest> PlanTaskRequestPtr;
typedef boost::shared_ptr< ::task_planner::PlanTaskRequest const> PlanTaskRequestConstPtr;


template <class ContainerAllocator>
struct PlanTaskResponse_ {
  typedef PlanTaskResponse_<ContainerAllocator> Type;

  PlanTaskResponse_()
  : plans()
  {
  }

  PlanTaskResponse_(const ContainerAllocator& _alloc)
  : plans(_alloc)
  {
  }

  typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _plans_type;
  std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  plans;


  typedef boost::shared_ptr< ::task_planner::PlanTaskResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::task_planner::PlanTaskResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct PlanTaskResponse
typedef  ::task_planner::PlanTaskResponse_<std::allocator<void> > PlanTaskResponse;

typedef boost::shared_ptr< ::task_planner::PlanTaskResponse> PlanTaskResponsePtr;
typedef boost::shared_ptr< ::task_planner::PlanTaskResponse const> PlanTaskResponseConstPtr;

struct PlanTask
{

typedef PlanTaskRequest Request;
typedef PlanTaskResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct PlanTask
} // namespace task_planner

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::task_planner::PlanTaskRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::task_planner::PlanTaskRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::task_planner::PlanTaskRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "99170af098dfcb0ae7b06f35caecb510";
  }

  static const char* value(const  ::task_planner::PlanTaskRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x99170af098dfcb0aULL;
  static const uint64_t static_value2 = 0xe7b06f35caecb510ULL;
};

template<class ContainerAllocator>
struct DataType< ::task_planner::PlanTaskRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "task_planner/PlanTaskRequest";
  }

  static const char* value(const  ::task_planner::PlanTaskRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::task_planner::PlanTaskRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
arm_navigation_msgs/CollisionObject[] objects\n\
\n\
================================================================================\n\
MSG: arm_navigation_msgs/CollisionObject\n\
# a header, used for interpreting the poses\n\
Header header\n\
\n\
# the id of the object\n\
string id\n\
\n\
# The padding used for filtering points near the object.\n\
# This does not affect collision checking for the object.  \n\
# Set to negative to get zero padding.\n\
float32 padding\n\
\n\
#This contains what is to be done with the object\n\
CollisionObjectOperation operation\n\
\n\
#the shapes associated with the object\n\
arm_navigation_msgs/Shape[] shapes\n\
\n\
#the poses associated with the shapes - will be transformed using the header\n\
geometry_msgs/Pose[] poses\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: arm_navigation_msgs/CollisionObjectOperation\n\
#Puts the object into the environment\n\
#or updates the object if already added\n\
byte ADD=0\n\
\n\
#Removes the object from the environment entirely\n\
byte REMOVE=1\n\
\n\
#Only valid within the context of a CollisionAttachedObject message\n\
#Will be ignored if sent with an CollisionObject message\n\
#Takes an attached object, detaches from the attached link\n\
#But adds back in as regular object\n\
byte DETACH_AND_ADD_AS_OBJECT=2\n\
\n\
#Only valid within the context of a CollisionAttachedObject message\n\
#Will be ignored if sent with an CollisionObject message\n\
#Takes current object in the environment and removes it as\n\
#a regular object\n\
byte ATTACH_AND_REMOVE_AS_OBJECT=3\n\
\n\
# Byte code for operation\n\
byte operation\n\
\n\
================================================================================\n\
MSG: arm_navigation_msgs/Shape\n\
byte SPHERE=0\n\
byte BOX=1\n\
byte CYLINDER=2\n\
byte MESH=3\n\
\n\
byte type\n\
\n\
\n\
#### define sphere, box, cylinder ####\n\
# the origin of each shape is considered at the shape's center\n\
\n\
# for sphere\n\
# radius := dimensions[0]\n\
\n\
# for cylinder\n\
# radius := dimensions[0]\n\
# length := dimensions[1]\n\
# the length is along the Z axis\n\
\n\
# for box\n\
# size_x := dimensions[0]\n\
# size_y := dimensions[1]\n\
# size_z := dimensions[2]\n\
float64[] dimensions\n\
\n\
\n\
#### define mesh ####\n\
\n\
# list of triangles; triangle k is defined by tre vertices located\n\
# at indices triangles[3k], triangles[3k+1], triangles[3k+2]\n\
int32[] triangles\n\
geometry_msgs/Point[] vertices\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of postion and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
";
  }

  static const char* value(const  ::task_planner::PlanTaskRequest_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::task_planner::PlanTaskResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::task_planner::PlanTaskResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::task_planner::PlanTaskResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ce5b451fa3baf09328c52bff9c283adf";
  }

  static const char* value(const  ::task_planner::PlanTaskResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xce5b451fa3baf093ULL;
  static const uint64_t static_value2 = 0x28c52bff9c283adfULL;
};

template<class ContainerAllocator>
struct DataType< ::task_planner::PlanTaskResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "task_planner/PlanTaskResponse";
  }

  static const char* value(const  ::task_planner::PlanTaskResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::task_planner::PlanTaskResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "string[] plans\n\
\n\
\n\
";
  }

  static const char* value(const  ::task_planner::PlanTaskResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::task_planner::PlanTaskRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.objects);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct PlanTaskRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::task_planner::PlanTaskResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.plans);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct PlanTaskResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<task_planner::PlanTask> {
  static const char* value() 
  {
    return "e14aed90eedaddbd0822c0e675600537";
  }

  static const char* value(const task_planner::PlanTask&) { return value(); } 
};

template<>
struct DataType<task_planner::PlanTask> {
  static const char* value() 
  {
    return "task_planner/PlanTask";
  }

  static const char* value(const task_planner::PlanTask&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<task_planner::PlanTaskRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "e14aed90eedaddbd0822c0e675600537";
  }

  static const char* value(const task_planner::PlanTaskRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<task_planner::PlanTaskRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "task_planner/PlanTask";
  }

  static const char* value(const task_planner::PlanTaskRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<task_planner::PlanTaskResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "e14aed90eedaddbd0822c0e675600537";
  }

  static const char* value(const task_planner::PlanTaskResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<task_planner::PlanTaskResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "task_planner/PlanTask";
  }

  static const char* value(const task_planner::PlanTaskResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // TASK_PLANNER_SERVICE_PLANTASK_H

