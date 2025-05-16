// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from create_msgs:msg/Bumper.idl
// generated code does not contain a copyright notice

#ifndef CREATE_MSGS__MSG__DETAIL__BUMPER__STRUCT_HPP_
#define CREATE_MSGS__MSG__DETAIL__BUMPER__STRUCT_HPP_

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
# define DEPRECATED__create_msgs__msg__Bumper __attribute__((deprecated))
#else
# define DEPRECATED__create_msgs__msg__Bumper __declspec(deprecated)
#endif

namespace create_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Bumper_
{
  using Type = Bumper_<ContainerAllocator>;

  explicit Bumper_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_left_pressed = false;
      this->is_right_pressed = false;
      this->is_light_left = false;
      this->is_light_front_left = false;
      this->is_light_center_left = false;
      this->is_light_center_right = false;
      this->is_light_front_right = false;
      this->is_light_right = false;
      this->light_signal_left = 0;
      this->light_signal_front_left = 0;
      this->light_signal_center_left = 0;
      this->light_signal_center_right = 0;
      this->light_signal_front_right = 0;
      this->light_signal_right = 0;
    }
  }

  explicit Bumper_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_left_pressed = false;
      this->is_right_pressed = false;
      this->is_light_left = false;
      this->is_light_front_left = false;
      this->is_light_center_left = false;
      this->is_light_center_right = false;
      this->is_light_front_right = false;
      this->is_light_right = false;
      this->light_signal_left = 0;
      this->light_signal_front_left = 0;
      this->light_signal_center_left = 0;
      this->light_signal_center_right = 0;
      this->light_signal_front_right = 0;
      this->light_signal_right = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _is_left_pressed_type =
    bool;
  _is_left_pressed_type is_left_pressed;
  using _is_right_pressed_type =
    bool;
  _is_right_pressed_type is_right_pressed;
  using _is_light_left_type =
    bool;
  _is_light_left_type is_light_left;
  using _is_light_front_left_type =
    bool;
  _is_light_front_left_type is_light_front_left;
  using _is_light_center_left_type =
    bool;
  _is_light_center_left_type is_light_center_left;
  using _is_light_center_right_type =
    bool;
  _is_light_center_right_type is_light_center_right;
  using _is_light_front_right_type =
    bool;
  _is_light_front_right_type is_light_front_right;
  using _is_light_right_type =
    bool;
  _is_light_right_type is_light_right;
  using _light_signal_left_type =
    uint16_t;
  _light_signal_left_type light_signal_left;
  using _light_signal_front_left_type =
    uint16_t;
  _light_signal_front_left_type light_signal_front_left;
  using _light_signal_center_left_type =
    uint16_t;
  _light_signal_center_left_type light_signal_center_left;
  using _light_signal_center_right_type =
    uint16_t;
  _light_signal_center_right_type light_signal_center_right;
  using _light_signal_front_right_type =
    uint16_t;
  _light_signal_front_right_type light_signal_front_right;
  using _light_signal_right_type =
    uint16_t;
  _light_signal_right_type light_signal_right;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__is_left_pressed(
    const bool & _arg)
  {
    this->is_left_pressed = _arg;
    return *this;
  }
  Type & set__is_right_pressed(
    const bool & _arg)
  {
    this->is_right_pressed = _arg;
    return *this;
  }
  Type & set__is_light_left(
    const bool & _arg)
  {
    this->is_light_left = _arg;
    return *this;
  }
  Type & set__is_light_front_left(
    const bool & _arg)
  {
    this->is_light_front_left = _arg;
    return *this;
  }
  Type & set__is_light_center_left(
    const bool & _arg)
  {
    this->is_light_center_left = _arg;
    return *this;
  }
  Type & set__is_light_center_right(
    const bool & _arg)
  {
    this->is_light_center_right = _arg;
    return *this;
  }
  Type & set__is_light_front_right(
    const bool & _arg)
  {
    this->is_light_front_right = _arg;
    return *this;
  }
  Type & set__is_light_right(
    const bool & _arg)
  {
    this->is_light_right = _arg;
    return *this;
  }
  Type & set__light_signal_left(
    const uint16_t & _arg)
  {
    this->light_signal_left = _arg;
    return *this;
  }
  Type & set__light_signal_front_left(
    const uint16_t & _arg)
  {
    this->light_signal_front_left = _arg;
    return *this;
  }
  Type & set__light_signal_center_left(
    const uint16_t & _arg)
  {
    this->light_signal_center_left = _arg;
    return *this;
  }
  Type & set__light_signal_center_right(
    const uint16_t & _arg)
  {
    this->light_signal_center_right = _arg;
    return *this;
  }
  Type & set__light_signal_front_right(
    const uint16_t & _arg)
  {
    this->light_signal_front_right = _arg;
    return *this;
  }
  Type & set__light_signal_right(
    const uint16_t & _arg)
  {
    this->light_signal_right = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    create_msgs::msg::Bumper_<ContainerAllocator> *;
  using ConstRawPtr =
    const create_msgs::msg::Bumper_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<create_msgs::msg::Bumper_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<create_msgs::msg::Bumper_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      create_msgs::msg::Bumper_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<create_msgs::msg::Bumper_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      create_msgs::msg::Bumper_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<create_msgs::msg::Bumper_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<create_msgs::msg::Bumper_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<create_msgs::msg::Bumper_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__create_msgs__msg__Bumper
    std::shared_ptr<create_msgs::msg::Bumper_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__create_msgs__msg__Bumper
    std::shared_ptr<create_msgs::msg::Bumper_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Bumper_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->is_left_pressed != other.is_left_pressed) {
      return false;
    }
    if (this->is_right_pressed != other.is_right_pressed) {
      return false;
    }
    if (this->is_light_left != other.is_light_left) {
      return false;
    }
    if (this->is_light_front_left != other.is_light_front_left) {
      return false;
    }
    if (this->is_light_center_left != other.is_light_center_left) {
      return false;
    }
    if (this->is_light_center_right != other.is_light_center_right) {
      return false;
    }
    if (this->is_light_front_right != other.is_light_front_right) {
      return false;
    }
    if (this->is_light_right != other.is_light_right) {
      return false;
    }
    if (this->light_signal_left != other.light_signal_left) {
      return false;
    }
    if (this->light_signal_front_left != other.light_signal_front_left) {
      return false;
    }
    if (this->light_signal_center_left != other.light_signal_center_left) {
      return false;
    }
    if (this->light_signal_center_right != other.light_signal_center_right) {
      return false;
    }
    if (this->light_signal_front_right != other.light_signal_front_right) {
      return false;
    }
    if (this->light_signal_right != other.light_signal_right) {
      return false;
    }
    return true;
  }
  bool operator!=(const Bumper_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Bumper_

// alias to use template instance with default allocator
using Bumper =
  create_msgs::msg::Bumper_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace create_msgs

#endif  // CREATE_MSGS__MSG__DETAIL__BUMPER__STRUCT_HPP_
