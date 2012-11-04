/* Auto-generated by genmsg_cpp for file /home/vektor/hiroken-ros-pkg/nn_machine/msg/Feature.msg */
#ifndef NN_MACHINE_MESSAGE_FEATURE_H
#define NN_MACHINE_MESSAGE_FEATURE_H
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


namespace nn_machine
{
template <class ContainerAllocator>
struct Feature_ {
  typedef Feature_<ContainerAllocator> Type;

  Feature_()
  : key()
  , val(0.0)
  {
  }

  Feature_(const ContainerAllocator& _alloc)
  : key(_alloc)
  , val(0.0)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _key_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  key;

  typedef double _val_type;
  double val;


  typedef boost::shared_ptr< ::nn_machine::Feature_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nn_machine::Feature_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct Feature
typedef  ::nn_machine::Feature_<std::allocator<void> > Feature;

typedef boost::shared_ptr< ::nn_machine::Feature> FeaturePtr;
typedef boost::shared_ptr< ::nn_machine::Feature const> FeatureConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::nn_machine::Feature_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::nn_machine::Feature_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace nn_machine

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::nn_machine::Feature_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::nn_machine::Feature_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::nn_machine::Feature_<ContainerAllocator> > {
  static const char* value() 
  {
    return "1a0d2c6e9c8537750f0d8dd7b9661bfc";
  }

  static const char* value(const  ::nn_machine::Feature_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x1a0d2c6e9c853775ULL;
  static const uint64_t static_value2 = 0x0f0d8dd7b9661bfcULL;
};

template<class ContainerAllocator>
struct DataType< ::nn_machine::Feature_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nn_machine/Feature";
  }

  static const char* value(const  ::nn_machine::Feature_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::nn_machine::Feature_<ContainerAllocator> > {
  static const char* value() 
  {
    return "string key\n\
float64 val\n\
\n\
";
  }

  static const char* value(const  ::nn_machine::Feature_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::nn_machine::Feature_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.key);
    stream.next(m.val);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct Feature_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::nn_machine::Feature_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::nn_machine::Feature_<ContainerAllocator> & v) 
  {
    s << indent << "key: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.key);
    s << indent << "val: ";
    Printer<double>::stream(s, indent + "  ", v.val);
  }
};


} // namespace message_operations
} // namespace ros

#endif // NN_MACHINE_MESSAGE_FEATURE_H
