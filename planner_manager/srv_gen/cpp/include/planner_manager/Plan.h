/* Auto-generated by genmsg_cpp for file /home/vektor/hiroken-ros-pkg/planner_manager/srv/Plan.srv */
#ifndef PLANNER_MANAGER_SERVICE_PLAN_H
#define PLANNER_MANAGER_SERVICE_PLAN_H
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



#include "trajectory_msgs/JointTrajectory.h"

namespace planner_manager
{
template <class ContainerAllocator>
struct PlanRequest_ {
  typedef PlanRequest_<ContainerAllocator> Type;

  PlanRequest_()
  : ml_mode(0)
  , log_path()
  , rerun(false)
  {
  }

  PlanRequest_(const ContainerAllocator& _alloc)
  : ml_mode(0)
  , log_path(_alloc)
  , rerun(false)
  {
  }

  typedef uint8_t _ml_mode_type;
  uint8_t ml_mode;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _log_path_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  log_path;

  typedef uint8_t _rerun_type;
  uint8_t rerun;


  typedef boost::shared_ptr< ::planner_manager::PlanRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::planner_manager::PlanRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct PlanRequest
typedef  ::planner_manager::PlanRequest_<std::allocator<void> > PlanRequest;

typedef boost::shared_ptr< ::planner_manager::PlanRequest> PlanRequestPtr;
typedef boost::shared_ptr< ::planner_manager::PlanRequest const> PlanRequestConstPtr;


template <class ContainerAllocator>
struct PlanResponse_ {
  typedef PlanResponse_<ContainerAllocator> Type;

  PlanResponse_()
  : ctamp_sol()
  , ctamp_log()
  {
  }

  PlanResponse_(const ContainerAllocator& _alloc)
  : ctamp_sol(_alloc)
  , ctamp_log(_alloc)
  {
  }

  typedef std::vector< ::trajectory_msgs::JointTrajectory_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::trajectory_msgs::JointTrajectory_<ContainerAllocator> >::other >  _ctamp_sol_type;
  std::vector< ::trajectory_msgs::JointTrajectory_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::trajectory_msgs::JointTrajectory_<ContainerAllocator> >::other >  ctamp_sol;

  typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _ctamp_log_type;
  std::vector<double, typename ContainerAllocator::template rebind<double>::other >  ctamp_log;


  typedef boost::shared_ptr< ::planner_manager::PlanResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::planner_manager::PlanResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct PlanResponse
typedef  ::planner_manager::PlanResponse_<std::allocator<void> > PlanResponse;

typedef boost::shared_ptr< ::planner_manager::PlanResponse> PlanResponsePtr;
typedef boost::shared_ptr< ::planner_manager::PlanResponse const> PlanResponseConstPtr;

struct Plan
{

typedef PlanRequest Request;
typedef PlanResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct Plan
} // namespace planner_manager

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::planner_manager::PlanRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::planner_manager::PlanRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::planner_manager::PlanRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "dd4eecefbba533c08853534a8083b7c5";
  }

  static const char* value(const  ::planner_manager::PlanRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xdd4eecefbba533c0ULL;
  static const uint64_t static_value2 = 0x8853534a8083b7c5ULL;
};

template<class ContainerAllocator>
struct DataType< ::planner_manager::PlanRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "planner_manager/PlanRequest";
  }

  static const char* value(const  ::planner_manager::PlanRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::planner_manager::PlanRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "uint8 ml_mode\n\
string log_path\n\
bool rerun\n\
\n\
";
  }

  static const char* value(const  ::planner_manager::PlanRequest_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::planner_manager::PlanResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::planner_manager::PlanResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::planner_manager::PlanResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "700e06dcc5e788860bef7920b6810e7b";
  }

  static const char* value(const  ::planner_manager::PlanResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x700e06dcc5e78886ULL;
  static const uint64_t static_value2 = 0x0bef7920b6810e7bULL;
};

template<class ContainerAllocator>
struct DataType< ::planner_manager::PlanResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "planner_manager/PlanResponse";
  }

  static const char* value(const  ::planner_manager::PlanResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::planner_manager::PlanResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "trajectory_msgs/JointTrajectory[] ctamp_sol\n\
float64[] ctamp_log\n\
\n\
\n\
================================================================================\n\
MSG: trajectory_msgs/JointTrajectory\n\
Header header\n\
string[] joint_names\n\
JointTrajectoryPoint[] points\n\
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
MSG: trajectory_msgs/JointTrajectoryPoint\n\
float64[] positions\n\
float64[] velocities\n\
float64[] accelerations\n\
duration time_from_start\n\
";
  }

  static const char* value(const  ::planner_manager::PlanResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::planner_manager::PlanRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.ml_mode);
    stream.next(m.log_path);
    stream.next(m.rerun);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct PlanRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::planner_manager::PlanResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.ctamp_sol);
    stream.next(m.ctamp_log);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct PlanResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<planner_manager::Plan> {
  static const char* value() 
  {
    return "88ec01b41a01a569deb9eb80adb4ff64";
  }

  static const char* value(const planner_manager::Plan&) { return value(); } 
};

template<>
struct DataType<planner_manager::Plan> {
  static const char* value() 
  {
    return "planner_manager/Plan";
  }

  static const char* value(const planner_manager::Plan&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<planner_manager::PlanRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "88ec01b41a01a569deb9eb80adb4ff64";
  }

  static const char* value(const planner_manager::PlanRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<planner_manager::PlanRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "planner_manager/Plan";
  }

  static const char* value(const planner_manager::PlanRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<planner_manager::PlanResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "88ec01b41a01a569deb9eb80adb4ff64";
  }

  static const char* value(const planner_manager::PlanResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<planner_manager::PlanResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "planner_manager/Plan";
  }

  static const char* value(const planner_manager::PlanResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // PLANNER_MANAGER_SERVICE_PLAN_H

