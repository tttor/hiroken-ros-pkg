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
  : n(0)
  , random(false)
  {
  }

  SeeRequest_(const ContainerAllocator& _alloc)
  : n(0)
  , random(false)
  {
  }

  typedef uint16_t _n_type;
  uint16_t n;

  typedef uint8_t _random_type;
  uint8_t random;


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
    return "9803dc42d7ec8be72013e5dbb53b26e1";
  }

  static const char* value(const  ::hiro_sensor::SeeRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x9803dc42d7ec8be7ULL;
  static const uint64_t static_value2 = 0x2013e5dbb53b26e1ULL;
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
    return "uint16 n\n\
bool random\n\
\n\
";
  }

  static const char* value(const  ::hiro_sensor::SeeRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::hiro_sensor::SeeRequest_<ContainerAllocator> > : public TrueType {};
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
    stream.next(m.n);
    stream.next(m.random);
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
    return "9803dc42d7ec8be72013e5dbb53b26e1";
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
    return "9803dc42d7ec8be72013e5dbb53b26e1";
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
    return "9803dc42d7ec8be72013e5dbb53b26e1";
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

