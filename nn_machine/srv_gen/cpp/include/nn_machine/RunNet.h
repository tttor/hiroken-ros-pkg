/* Auto-generated by genmsg_cpp for file /home/vektor/hiroken-ros-pkg/nn_machine/srv/RunNet.srv */
#ifndef NN_MACHINE_SERVICE_RUNNET_H
#define NN_MACHINE_SERVICE_RUNNET_H
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

#include "nn_machine/Feature.h"



namespace nn_machine
{
template <class ContainerAllocator>
struct RunNetRequest_ {
  typedef RunNetRequest_<ContainerAllocator> Type;

  RunNetRequest_()
  : input()
  {
  }

  RunNetRequest_(const ContainerAllocator& _alloc)
  : input(_alloc)
  {
  }

  typedef std::vector< ::nn_machine::Feature_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::nn_machine::Feature_<ContainerAllocator> >::other >  _input_type;
  std::vector< ::nn_machine::Feature_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::nn_machine::Feature_<ContainerAllocator> >::other >  input;


  typedef boost::shared_ptr< ::nn_machine::RunNetRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nn_machine::RunNetRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct RunNetRequest
typedef  ::nn_machine::RunNetRequest_<std::allocator<void> > RunNetRequest;

typedef boost::shared_ptr< ::nn_machine::RunNetRequest> RunNetRequestPtr;
typedef boost::shared_ptr< ::nn_machine::RunNetRequest const> RunNetRequestConstPtr;


template <class ContainerAllocator>
struct RunNetResponse_ {
  typedef RunNetResponse_<ContainerAllocator> Type;

  RunNetResponse_()
  : output(0.0)
  {
  }

  RunNetResponse_(const ContainerAllocator& _alloc)
  : output(0.0)
  {
  }

  typedef double _output_type;
  double output;


  typedef boost::shared_ptr< ::nn_machine::RunNetResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nn_machine::RunNetResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct RunNetResponse
typedef  ::nn_machine::RunNetResponse_<std::allocator<void> > RunNetResponse;

typedef boost::shared_ptr< ::nn_machine::RunNetResponse> RunNetResponsePtr;
typedef boost::shared_ptr< ::nn_machine::RunNetResponse const> RunNetResponseConstPtr;

struct RunNet
{

typedef RunNetRequest Request;
typedef RunNetResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct RunNet
} // namespace nn_machine

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::nn_machine::RunNetRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::nn_machine::RunNetRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::nn_machine::RunNetRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d615c3a6b45b80516358d2ed6c0cad21";
  }

  static const char* value(const  ::nn_machine::RunNetRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd615c3a6b45b8051ULL;
  static const uint64_t static_value2 = 0x6358d2ed6c0cad21ULL;
};

template<class ContainerAllocator>
struct DataType< ::nn_machine::RunNetRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nn_machine/RunNetRequest";
  }

  static const char* value(const  ::nn_machine::RunNetRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::nn_machine::RunNetRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nn_machine/Feature[] input\n\
\n\
================================================================================\n\
MSG: nn_machine/Feature\n\
string key\n\
float64 val\n\
\n\
";
  }

  static const char* value(const  ::nn_machine::RunNetRequest_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::nn_machine::RunNetResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::nn_machine::RunNetResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::nn_machine::RunNetResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "5dd87a43ba76105996c6c8cafb738498";
  }

  static const char* value(const  ::nn_machine::RunNetResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x5dd87a43ba761059ULL;
  static const uint64_t static_value2 = 0x96c6c8cafb738498ULL;
};

template<class ContainerAllocator>
struct DataType< ::nn_machine::RunNetResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nn_machine/RunNetResponse";
  }

  static const char* value(const  ::nn_machine::RunNetResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::nn_machine::RunNetResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float64 output\n\
\n\
\n\
";
  }

  static const char* value(const  ::nn_machine::RunNetResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::nn_machine::RunNetResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::nn_machine::RunNetRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.input);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct RunNetRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::nn_machine::RunNetResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.output);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct RunNetResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<nn_machine::RunNet> {
  static const char* value() 
  {
    return "8ab244ff7e826903d7f666bab8196504";
  }

  static const char* value(const nn_machine::RunNet&) { return value(); } 
};

template<>
struct DataType<nn_machine::RunNet> {
  static const char* value() 
  {
    return "nn_machine/RunNet";
  }

  static const char* value(const nn_machine::RunNet&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<nn_machine::RunNetRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "8ab244ff7e826903d7f666bab8196504";
  }

  static const char* value(const nn_machine::RunNetRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<nn_machine::RunNetRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nn_machine/RunNet";
  }

  static const char* value(const nn_machine::RunNetRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<nn_machine::RunNetResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "8ab244ff7e826903d7f666bab8196504";
  }

  static const char* value(const nn_machine::RunNetResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<nn_machine::RunNetResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nn_machine/RunNet";
  }

  static const char* value(const nn_machine::RunNetResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // NN_MACHINE_SERVICE_RUNNET_H
