/* Auto-generated by genmsg_cpp for file /home/nazrul/fuerte_workspace/ptam/srv/VisibilityCalculator.srv */
#ifndef PTAM_SERVICE_VISIBILITYCALCULATOR_H
#define PTAM_SERVICE_VISIBILITYCALCULATOR_H
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

#include "geometry_msgs/PoseArray.h"



namespace ptam
{
template <class ContainerAllocator>
struct VisibilityCalculatorRequest_ {
  typedef VisibilityCalculatorRequest_<ContainerAllocator> Type;

  VisibilityCalculatorRequest_()
  : ugv_frontiers()
  {
  }

  VisibilityCalculatorRequest_(const ContainerAllocator& _alloc)
  : ugv_frontiers(_alloc)
  {
  }

  typedef  ::geometry_msgs::PoseArray_<ContainerAllocator>  _ugv_frontiers_type;
   ::geometry_msgs::PoseArray_<ContainerAllocator>  ugv_frontiers;


  typedef boost::shared_ptr< ::ptam::VisibilityCalculatorRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ptam::VisibilityCalculatorRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct VisibilityCalculatorRequest
typedef  ::ptam::VisibilityCalculatorRequest_<std::allocator<void> > VisibilityCalculatorRequest;

typedef boost::shared_ptr< ::ptam::VisibilityCalculatorRequest> VisibilityCalculatorRequestPtr;
typedef boost::shared_ptr< ::ptam::VisibilityCalculatorRequest const> VisibilityCalculatorRequestConstPtr;


template <class ContainerAllocator>
struct VisibilityCalculatorResponse_ {
  typedef VisibilityCalculatorResponse_<ContainerAllocator> Type;

  VisibilityCalculatorResponse_()
  : ugv_frontier_visibility()
  {
  }

  VisibilityCalculatorResponse_(const ContainerAllocator& _alloc)
  : ugv_frontier_visibility(_alloc)
  {
  }

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _ugv_frontier_visibility_type;
  std::vector<float, typename ContainerAllocator::template rebind<float>::other >  ugv_frontier_visibility;


  typedef boost::shared_ptr< ::ptam::VisibilityCalculatorResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ptam::VisibilityCalculatorResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct VisibilityCalculatorResponse
typedef  ::ptam::VisibilityCalculatorResponse_<std::allocator<void> > VisibilityCalculatorResponse;

typedef boost::shared_ptr< ::ptam::VisibilityCalculatorResponse> VisibilityCalculatorResponsePtr;
typedef boost::shared_ptr< ::ptam::VisibilityCalculatorResponse const> VisibilityCalculatorResponseConstPtr;

struct VisibilityCalculator
{

typedef VisibilityCalculatorRequest Request;
typedef VisibilityCalculatorResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct VisibilityCalculator
} // namespace ptam

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::ptam::VisibilityCalculatorRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::ptam::VisibilityCalculatorRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::ptam::VisibilityCalculatorRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "c2e6d9601a2ecc523c450bbf904b63b1";
  }

  static const char* value(const  ::ptam::VisibilityCalculatorRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xc2e6d9601a2ecc52ULL;
  static const uint64_t static_value2 = 0x3c450bbf904b63b1ULL;
};

template<class ContainerAllocator>
struct DataType< ::ptam::VisibilityCalculatorRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ptam/VisibilityCalculatorRequest";
  }

  static const char* value(const  ::ptam::VisibilityCalculatorRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::ptam::VisibilityCalculatorRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "geometry_msgs/PoseArray ugv_frontiers\n\
\n\
================================================================================\n\
MSG: geometry_msgs/PoseArray\n\
# An array of poses with a header for global reference.\n\
\n\
Header header\n\
\n\
Pose[] poses\n\
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
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of postion and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
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

  static const char* value(const  ::ptam::VisibilityCalculatorRequest_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::ptam::VisibilityCalculatorResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::ptam::VisibilityCalculatorResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::ptam::VisibilityCalculatorResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "7b072e248eda809ae07916f535294b55";
  }

  static const char* value(const  ::ptam::VisibilityCalculatorResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x7b072e248eda809aULL;
  static const uint64_t static_value2 = 0xe07916f535294b55ULL;
};

template<class ContainerAllocator>
struct DataType< ::ptam::VisibilityCalculatorResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ptam/VisibilityCalculatorResponse";
  }

  static const char* value(const  ::ptam::VisibilityCalculatorResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::ptam::VisibilityCalculatorResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float32[] ugv_frontier_visibility\n\
\n\
\n\
";
  }

  static const char* value(const  ::ptam::VisibilityCalculatorResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::ptam::VisibilityCalculatorRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.ugv_frontiers);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct VisibilityCalculatorRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::ptam::VisibilityCalculatorResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.ugv_frontier_visibility);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct VisibilityCalculatorResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<ptam::VisibilityCalculator> {
  static const char* value() 
  {
    return "3a44e6eb897dd41c3de9eb553ef7c91f";
  }

  static const char* value(const ptam::VisibilityCalculator&) { return value(); } 
};

template<>
struct DataType<ptam::VisibilityCalculator> {
  static const char* value() 
  {
    return "ptam/VisibilityCalculator";
  }

  static const char* value(const ptam::VisibilityCalculator&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<ptam::VisibilityCalculatorRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "3a44e6eb897dd41c3de9eb553ef7c91f";
  }

  static const char* value(const ptam::VisibilityCalculatorRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<ptam::VisibilityCalculatorRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ptam/VisibilityCalculator";
  }

  static const char* value(const ptam::VisibilityCalculatorRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<ptam::VisibilityCalculatorResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "3a44e6eb897dd41c3de9eb553ef7c91f";
  }

  static const char* value(const ptam::VisibilityCalculatorResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<ptam::VisibilityCalculatorResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ptam/VisibilityCalculator";
  }

  static const char* value(const ptam::VisibilityCalculatorResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // PTAM_SERVICE_VISIBILITYCALCULATOR_H
