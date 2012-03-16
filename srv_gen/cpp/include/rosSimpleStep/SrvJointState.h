/* Auto-generated by genmsg_cpp for file /home/ssafarik/git/rosSimpleStep/srv/SrvJointState.srv */
#ifndef ROSSIMPLESTEP_SERVICE_SRVJOINTSTATE_H
#define ROSSIMPLESTEP_SERVICE_SRVJOINTSTATE_H
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

#include "std_msgs/Header.h"


#include "std_msgs/Header.h"

namespace rosSimpleStep
{
template <class ContainerAllocator>
struct SrvJointStateRequest_ {
  typedef SrvJointStateRequest_<ContainerAllocator> Type;

  SrvJointStateRequest_()
  : header()
  , position(0.0)
  , velocity(0.0)
  {
  }

  SrvJointStateRequest_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , position(0.0)
  , velocity(0.0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef double _position_type;
  double position;

  typedef double _velocity_type;
  double velocity;


private:
  static const char* __s_getDataType_() { return "rosSimpleStep/SrvJointStateRequest"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "cd1b1b329f46363cad43f272d53d52e8"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "05d70090e2d6b93bb2c3f7a7de104786"; }
public:
  ROS_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROS_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "Header header\n\
float64 position\n\
float64 velocity\n\
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
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, header);
    ros::serialization::serialize(stream, position);
    ros::serialization::serialize(stream, velocity);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, header);
    ros::serialization::deserialize(stream, position);
    ros::serialization::deserialize(stream, velocity);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(header);
    size += ros::serialization::serializationLength(position);
    size += ros::serialization::serializationLength(velocity);
    return size;
  }

  typedef boost::shared_ptr< ::rosSimpleStep::SrvJointStateRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rosSimpleStep::SrvJointStateRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct SrvJointStateRequest
typedef  ::rosSimpleStep::SrvJointStateRequest_<std::allocator<void> > SrvJointStateRequest;

typedef boost::shared_ptr< ::rosSimpleStep::SrvJointStateRequest> SrvJointStateRequestPtr;
typedef boost::shared_ptr< ::rosSimpleStep::SrvJointStateRequest const> SrvJointStateRequestConstPtr;


template <class ContainerAllocator>
struct SrvJointStateResponse_ {
  typedef SrvJointStateResponse_<ContainerAllocator> Type;

  SrvJointStateResponse_()
  : header()
  , position(0.0)
  , velocity(0.0)
  {
  }

  SrvJointStateResponse_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , position(0.0)
  , velocity(0.0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef double _position_type;
  double position;

  typedef double _velocity_type;
  double velocity;


private:
  static const char* __s_getDataType_() { return "rosSimpleStep/SrvJointStateResponse"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "cd1b1b329f46363cad43f272d53d52e8"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "05d70090e2d6b93bb2c3f7a7de104786"; }
public:
  ROS_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROS_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "Header header\n\
float64 position\n\
float64 velocity\n\
\n\
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
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, header);
    ros::serialization::serialize(stream, position);
    ros::serialization::serialize(stream, velocity);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, header);
    ros::serialization::deserialize(stream, position);
    ros::serialization::deserialize(stream, velocity);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(header);
    size += ros::serialization::serializationLength(position);
    size += ros::serialization::serializationLength(velocity);
    return size;
  }

  typedef boost::shared_ptr< ::rosSimpleStep::SrvJointStateResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rosSimpleStep::SrvJointStateResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct SrvJointStateResponse
typedef  ::rosSimpleStep::SrvJointStateResponse_<std::allocator<void> > SrvJointStateResponse;

typedef boost::shared_ptr< ::rosSimpleStep::SrvJointStateResponse> SrvJointStateResponsePtr;
typedef boost::shared_ptr< ::rosSimpleStep::SrvJointStateResponse const> SrvJointStateResponseConstPtr;

struct SrvJointState
{

typedef SrvJointStateRequest Request;
typedef SrvJointStateResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct SrvJointState
} // namespace rosSimpleStep

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::rosSimpleStep::SrvJointStateRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::rosSimpleStep::SrvJointStateRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::rosSimpleStep::SrvJointStateRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "cd1b1b329f46363cad43f272d53d52e8";
  }

  static const char* value(const  ::rosSimpleStep::SrvJointStateRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xcd1b1b329f46363cULL;
  static const uint64_t static_value2 = 0xad43f272d53d52e8ULL;
};

template<class ContainerAllocator>
struct DataType< ::rosSimpleStep::SrvJointStateRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "rosSimpleStep/SrvJointStateRequest";
  }

  static const char* value(const  ::rosSimpleStep::SrvJointStateRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::rosSimpleStep::SrvJointStateRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
float64 position\n\
float64 velocity\n\
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
";
  }

  static const char* value(const  ::rosSimpleStep::SrvJointStateRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::rosSimpleStep::SrvJointStateRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::rosSimpleStep::SrvJointStateRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::rosSimpleStep::SrvJointStateResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::rosSimpleStep::SrvJointStateResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::rosSimpleStep::SrvJointStateResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "cd1b1b329f46363cad43f272d53d52e8";
  }

  static const char* value(const  ::rosSimpleStep::SrvJointStateResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xcd1b1b329f46363cULL;
  static const uint64_t static_value2 = 0xad43f272d53d52e8ULL;
};

template<class ContainerAllocator>
struct DataType< ::rosSimpleStep::SrvJointStateResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "rosSimpleStep/SrvJointStateResponse";
  }

  static const char* value(const  ::rosSimpleStep::SrvJointStateResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::rosSimpleStep::SrvJointStateResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
float64 position\n\
float64 velocity\n\
\n\
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
";
  }

  static const char* value(const  ::rosSimpleStep::SrvJointStateResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::rosSimpleStep::SrvJointStateResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::rosSimpleStep::SrvJointStateResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::rosSimpleStep::SrvJointStateRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.position);
    stream.next(m.velocity);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct SrvJointStateRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::rosSimpleStep::SrvJointStateResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.position);
    stream.next(m.velocity);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct SrvJointStateResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<rosSimpleStep::SrvJointState> {
  static const char* value() 
  {
    return "05d70090e2d6b93bb2c3f7a7de104786";
  }

  static const char* value(const rosSimpleStep::SrvJointState&) { return value(); } 
};

template<>
struct DataType<rosSimpleStep::SrvJointState> {
  static const char* value() 
  {
    return "rosSimpleStep/SrvJointState";
  }

  static const char* value(const rosSimpleStep::SrvJointState&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<rosSimpleStep::SrvJointStateRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "05d70090e2d6b93bb2c3f7a7de104786";
  }

  static const char* value(const rosSimpleStep::SrvJointStateRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<rosSimpleStep::SrvJointStateRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "rosSimpleStep/SrvJointState";
  }

  static const char* value(const rosSimpleStep::SrvJointStateRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<rosSimpleStep::SrvJointStateResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "05d70090e2d6b93bb2c3f7a7de104786";
  }

  static const char* value(const rosSimpleStep::SrvJointStateResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<rosSimpleStep::SrvJointStateResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "rosSimpleStep/SrvJointState";
  }

  static const char* value(const rosSimpleStep::SrvJointStateResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // ROSSIMPLESTEP_SERVICE_SRVJOINTSTATE_H

