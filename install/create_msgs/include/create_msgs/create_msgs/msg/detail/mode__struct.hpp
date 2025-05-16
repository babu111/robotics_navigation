// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from create_msgs:msg/Mode.idl
// generated code does not contain a copyright notice

#ifndef CREATE_MSGS__MSG__DETAIL__MODE__STRUCT_HPP_
#define CREATE_MSGS__MSG__DETAIL__MODE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__create_msgs__msg__Mode __attribute__((deprecated))
#else
# define DEPRECATED__create_msgs__msg__Mode __declspec(deprecated)
#endif

namespace create_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Mode_
{
  using Type = Mode_<ContainerAllocator>;

  explicit Mode_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
    }
  }

  explicit Mode_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _mode_type =
    uint8_t;
  _mode_type mode;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__mode(
    const uint8_t & _arg)
  {
    this->mode = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t MODE_OFF =
    0u;
  static constexpr uint8_t MODE_PASSIVE =
    1u;
  static constexpr uint8_t MODE_SAFE =
    2u;
  static constexpr uint8_t MODE_FULL =
    3u;

  // pointer types
  using RawPtr =
    create_msgs::msg::Mode_<ContainerAllocator> *;
  using ConstRawPtr =
    const create_msgs::msg::Mode_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<create_msgs::msg::Mode_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<create_msgs::msg::Mode_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      create_msgs::msg::Mode_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<create_msgs::msg::Mode_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      create_msgs::msg::Mode_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<create_msgs::msg::Mode_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<create_msgs::msg::Mode_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<create_msgs::msg::Mode_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__create_msgs__msg__Mode
    std::shared_ptr<create_msgs::msg::Mode_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__create_msgs__msg__Mode
    std::shared_ptr<create_msgs::msg::Mode_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Mode_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->mode != other.mode) {
      return false;
    }
    return true;
  }
  bool operator!=(const Mode_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Mode_

// alias to use template instance with default allocator
using Mode =
  create_msgs::msg::Mode_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Mode_<ContainerAllocator>::MODE_OFF;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Mode_<ContainerAllocator>::MODE_PASSIVE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Mode_<ContainerAllocator>::MODE_SAFE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Mode_<ContainerAllocator>::MODE_FULL;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace create_msgs

#endif  // CREATE_MSGS__MSG__DETAIL__MODE__STRUCT_HPP_
