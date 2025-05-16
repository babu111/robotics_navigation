// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from create_msgs:msg/ChargingState.idl
// generated code does not contain a copyright notice

#ifndef CREATE_MSGS__MSG__DETAIL__CHARGING_STATE__STRUCT_HPP_
#define CREATE_MSGS__MSG__DETAIL__CHARGING_STATE__STRUCT_HPP_

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
# define DEPRECATED__create_msgs__msg__ChargingState __attribute__((deprecated))
#else
# define DEPRECATED__create_msgs__msg__ChargingState __declspec(deprecated)
#endif

namespace create_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ChargingState_
{
  using Type = ChargingState_<ContainerAllocator>;

  explicit ChargingState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0;
    }
  }

  explicit ChargingState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _state_type =
    uint8_t;
  _state_type state;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__state(
    const uint8_t & _arg)
  {
    this->state = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t CHARGE_NONE =
    0u;
  static constexpr uint8_t CHARGE_RECONDITION =
    1u;
  static constexpr uint8_t CHARGE_FULL =
    2u;
  static constexpr uint8_t CHARGE_TRICKLE =
    3u;
  static constexpr uint8_t CHARGE_WAITING =
    4u;
  static constexpr uint8_t CHARGE_FAULT =
    5u;

  // pointer types
  using RawPtr =
    create_msgs::msg::ChargingState_<ContainerAllocator> *;
  using ConstRawPtr =
    const create_msgs::msg::ChargingState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<create_msgs::msg::ChargingState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<create_msgs::msg::ChargingState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      create_msgs::msg::ChargingState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<create_msgs::msg::ChargingState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      create_msgs::msg::ChargingState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<create_msgs::msg::ChargingState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<create_msgs::msg::ChargingState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<create_msgs::msg::ChargingState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__create_msgs__msg__ChargingState
    std::shared_ptr<create_msgs::msg::ChargingState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__create_msgs__msg__ChargingState
    std::shared_ptr<create_msgs::msg::ChargingState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ChargingState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    return true;
  }
  bool operator!=(const ChargingState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ChargingState_

// alias to use template instance with default allocator
using ChargingState =
  create_msgs::msg::ChargingState_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ChargingState_<ContainerAllocator>::CHARGE_NONE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ChargingState_<ContainerAllocator>::CHARGE_RECONDITION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ChargingState_<ContainerAllocator>::CHARGE_FULL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ChargingState_<ContainerAllocator>::CHARGE_TRICKLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ChargingState_<ContainerAllocator>::CHARGE_WAITING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ChargingState_<ContainerAllocator>::CHARGE_FAULT;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace create_msgs

#endif  // CREATE_MSGS__MSG__DETAIL__CHARGING_STATE__STRUCT_HPP_
