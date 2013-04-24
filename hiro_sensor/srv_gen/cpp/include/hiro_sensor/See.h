/* Auto-generated by genmsg_cpp for file /home/vektor/hiroken-ros-pkg/hiro_sensor/srv/See.srv */
#ifndef HIRO_SENSOR_SERVICE_SEE_H
#define HIRO_SENSOR_SERVICE_SEE_H
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




namespace hiro_sensor
{
template <class ContainerAllocator>
struct SeeRequest_ {
  typedef SeeRequest_<ContainerAllocator> Type;

  SeeRequest_()
  : rerun(false)
  , path()
  , n_movable_object(0)
  , n_vase(0)
  , randomized_vase(false)
  {
  }

  SeeRequest_(const ContainerAllocator& _alloc)
  : rerun(false)
  , path(_alloc)
  , n_movable_object(0)
  , n_vase(0)
  , randomized_vase(false)
  {
  }

  typedef uint8_t _rerun_type;
  uint8_t rerun;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _path_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  path;

  typedef uint16_t _n_movable_object_type;
  uint16_t n_movable_object;

  typedef uint16_t _n_vase_type;
  uint16_t n_vase;

  typedef uint8_t _randomized_vase_type;
  uint8_t randomized_vase;


  typedef boost::shared_ptr< ::hiro_sensor::SeeRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hiro_sensor::SeeRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct SeeRequest
typedef  ::hiro_sensor::SeeRequest_<std::allocator<void> > SeeRequest;

typedef boost::shared_ptr< ::hiro_sensor::SeeRequest> SeeRequestPtr;
typedef boost::shared_ptr< ::hiro_sensor::SeeRequest const> SeeRequestConstPtr;


template <class ContainerAllocator>
struct SeeResponse_ {
  typedef SeeResponse_<ContainerAllocator> Type;

  SeeResponse_()
  {
  }

  SeeResponse_(const ContainerAllocator& _alloc)
  {
  }


  typedef boost::shared_ptr< ::hiro_sensor::SeeResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hiro_sensor::SeeResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct SeeResponse
typedef  ::hiro_sensor::SeeResponse_<std::allocator<void> > SeeResponse;

typedef boost::shared_ptr< ::hiro_sensor::SeeResponse> SeeResponsePtr;
typedef boost::shared_ptr< ::hiro_sensor::SeeResponse const> SeeResponseConstPtr;

struct See
{

typedef SeeRequest Request;
typedef SeeResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct See
} // namespace hiro_sensor

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::hiro_sensor::SeeRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::hiro_sensor::SeeRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::hiro_sensor::SeeRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bbe84daac54043cac37fe08e055733cf";
  }

  static const char* value(const  ::hiro_sensor::SeeRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xbbe84daac54043caULL;
  static const uint64_t static_value2 = 0xc37fe08e055733cfULL;
};

template<class ContainerAllocator>
struct DataType< ::hiro_sensor::SeeRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "hiro_sensor/SeeRequest";
  }

  static const char* value(const  ::hiro_sensor::SeeRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::hiro_sensor::SeeRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool rerun\n\
string path\n\
uint16 n_movable_object\n\
uint16 n_vase\n\
bool randomized_vase\n\
\n\
";
  }

  static const char* value(const  ::hiro_sensor::SeeRequest_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::hiro_sensor::SeeResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::hiro_sensor::SeeResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::hiro_sensor::SeeResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const  ::hiro_sensor::SeeResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::hiro_sensor::SeeResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "hiro_sensor/SeeResponse";
  }

  static const char* value(const  ::hiro_sensor::SeeResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::hiro_sensor::SeeResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
";
  }

  static const char* value(const  ::hiro_sensor::SeeResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::hiro_sensor::SeeResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::hiro_sensor::SeeRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.rerun);
    stream.next(m.path);
    stream.next(m.n_movable_object);
    stream.next(m.n_vase);
    stream.next(m.randomized_vase);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct SeeRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::hiro_sensor::SeeResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct SeeResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<hiro_sensor::See> {
  static const char* value() 
  {
    return "bbe84daac54043cac37fe08e055733cf";
  }

  static const char* value(const hiro_sensor::See&) { return value(); } 
};

template<>
struct DataType<hiro_sensor::See> {
  static const char* value() 
  {
    return "hiro_sensor/See";
  }

  static const char* value(const hiro_sensor::See&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<hiro_sensor::SeeRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bbe84daac54043cac37fe08e055733cf";
  }

  static const char* value(const hiro_sensor::SeeRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<hiro_sensor::SeeRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "hiro_sensor/See";
  }

  static const char* value(const hiro_sensor::SeeRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<hiro_sensor::SeeResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bbe84daac54043cac37fe08e055733cf";
  }

  static const char* value(const hiro_sensor::SeeResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<hiro_sensor::SeeResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "hiro_sensor/See";
  }

  static const char* value(const hiro_sensor::SeeResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // HIRO_SENSOR_SERVICE_SEE_H

