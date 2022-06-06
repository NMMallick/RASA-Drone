// Generated by gencpp from file offboard/offboardMode.msg
// DO NOT EDIT!


#ifndef OFFBOARD_MESSAGE_OFFBOARDMODE_H
#define OFFBOARD_MESSAGE_OFFBOARDMODE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace offboard
{
template <class ContainerAllocator>
struct offboardMode_
{
  typedef offboardMode_<ContainerAllocator> Type;

  offboardMode_()
    : state()  {
    }
  offboardMode_(const ContainerAllocator& _alloc)
    : state(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _state_type;
  _state_type state;





  typedef boost::shared_ptr< ::offboard::offboardMode_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::offboard::offboardMode_<ContainerAllocator> const> ConstPtr;

}; // struct offboardMode_

typedef ::offboard::offboardMode_<std::allocator<void> > offboardMode;

typedef boost::shared_ptr< ::offboard::offboardMode > offboardModePtr;
typedef boost::shared_ptr< ::offboard::offboardMode const> offboardModeConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::offboard::offboardMode_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::offboard::offboardMode_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::offboard::offboardMode_<ContainerAllocator1> & lhs, const ::offboard::offboardMode_<ContainerAllocator2> & rhs)
{
  return lhs.state == rhs.state;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::offboard::offboardMode_<ContainerAllocator1> & lhs, const ::offboard::offboardMode_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace offboard

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::offboard::offboardMode_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::offboard::offboardMode_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::offboard::offboardMode_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::offboard::offboardMode_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::offboard::offboardMode_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::offboard::offboardMode_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::offboard::offboardMode_<ContainerAllocator> >
{
  static const char* value()
  {
    return "af6d3a99f0fbeb66d3248fa4b3e675fb";
  }

  static const char* value(const ::offboard::offboardMode_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xaf6d3a99f0fbeb66ULL;
  static const uint64_t static_value2 = 0xd3248fa4b3e675fbULL;
};

template<class ContainerAllocator>
struct DataType< ::offboard::offboardMode_<ContainerAllocator> >
{
  static const char* value()
  {
    return "offboard/offboardMode";
  }

  static const char* value(const ::offboard::offboardMode_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::offboard::offboardMode_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string state\n"
;
  }

  static const char* value(const ::offboard::offboardMode_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::offboard::offboardMode_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.state);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct offboardMode_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::offboard::offboardMode_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::offboard::offboardMode_<ContainerAllocator>& v)
  {
    s << indent << "state: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.state);
  }
};

} // namespace message_operations
} // namespace ros

#endif // OFFBOARD_MESSAGE_OFFBOARDMODE_H
