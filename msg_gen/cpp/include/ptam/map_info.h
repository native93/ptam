/* Auto-generated by genmsg_cpp for file /home/nazrul/fuerte_workspace/ptam/msg/map_info.msg */
#ifndef PTAM_MESSAGE_MAP_INFO_H
#define PTAM_MESSAGE_MAP_INFO_H
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


namespace ptam
{
template <class ContainerAllocator>
struct map_info_ {
  typedef map_info_<ContainerAllocator> Type;

  map_info_()
  : data()
  , index(0)
  {
  }

  map_info_(const ContainerAllocator& _alloc)
  : data(_alloc)
  , index(0)
  {
  }

  typedef std::vector<int8_t, typename ContainerAllocator::template rebind<int8_t>::other >  _data_type;
  std::vector<int8_t, typename ContainerAllocator::template rebind<int8_t>::other >  data;

  typedef int8_t _index_type;
  int8_t index;


  typedef boost::shared_ptr< ::ptam::map_info_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ptam::map_info_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct map_info
typedef  ::ptam::map_info_<std::allocator<void> > map_info;

typedef boost::shared_ptr< ::ptam::map_info> map_infoPtr;
typedef boost::shared_ptr< ::ptam::map_info const> map_infoConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::ptam::map_info_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::ptam::map_info_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace ptam

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::ptam::map_info_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::ptam::map_info_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::ptam::map_info_<ContainerAllocator> > {
  static const char* value() 
  {
    return "f14d254c9aa1e06009f4a43da7832db0";
  }

  static const char* value(const  ::ptam::map_info_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xf14d254c9aa1e060ULL;
  static const uint64_t static_value2 = 0x09f4a43da7832db0ULL;
};

template<class ContainerAllocator>
struct DataType< ::ptam::map_info_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ptam/map_info";
  }

  static const char* value(const  ::ptam::map_info_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::ptam::map_info_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int8[] data\n\
int8 index\n\
\n\
";
  }

  static const char* value(const  ::ptam::map_info_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::ptam::map_info_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.data);
    stream.next(m.index);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct map_info_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ptam::map_info_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::ptam::map_info_<ContainerAllocator> & v) 
  {
    s << indent << "data[]" << std::endl;
    for (size_t i = 0; i < v.data.size(); ++i)
    {
      s << indent << "  data[" << i << "]: ";
      Printer<int8_t>::stream(s, indent + "  ", v.data[i]);
    }
    s << indent << "index: ";
    Printer<int8_t>::stream(s, indent + "  ", v.index);
  }
};


} // namespace message_operations
} // namespace ros

#endif // PTAM_MESSAGE_MAP_INFO_H

