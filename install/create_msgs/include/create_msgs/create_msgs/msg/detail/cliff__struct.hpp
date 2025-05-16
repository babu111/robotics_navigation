// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from create_msgs:msg/Cliff.idl
// generated code does not contain a copyright notice

#ifndef CREATE_MSGS__MSG__DETAIL__CLIFF__STRUCT_HPP_
#define CREATE_MSGS__MSG__DETAIL__CLIFF__STRUCT_HPP_

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
# define DEPRECATED__create_msgs__msg__Cliff __attribute__((deprecated))
#else
# define DEPRECATED__create_msgs__msg__Cliff __declspec(deprecated)
#endif

namespace create_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Cliff_
{
  using Type = Cliff_<ContainerAllocator>;

  explicit Cliff_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_cliff_left = false;
      this->is_cliff_front_left = false;
      this->is_cliff_right = false;
      this->is_cliff_front_right = false;
    }
  }

  explicit Cliff_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_cliff_left = false;
      this->is_cliff_front_left = false;
      this->is_cliff_right = false;
      this->is_cliff_front_right = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _is_cliff_left_type =
    bool;
  _is_cliff_left_type is_cliff_left;
  using _is_cliff_front_left_type =
    bool;
  _is_cliff_front_left_type is_cliff_front_left;
  using _is_cliff_right_type =
    bool;
  _is_cliff_right_type is_cliff_right;
  using _is_cliff_front_right_type =
    bool;
  _is_cliff_front_right_type is_cliff_front_right;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__is_cliff_left(
    const bool & _arg)
  {
    this->is_cliff_left = _arg;
    return *this;
  }
  Type & set__is_cliff_front_left(
    const bool & _arg)
  {
    this->is_cliff_front_left = _arg;
    return *this;
  }
  Type & set__is_cliff_right(
    const bool & _arg)
  {
    this->is_cliff_right = _arg;
    return *this;
  }
  Type & set__is_cliff_front_right(
    const bool & _arg)
  {
    this->is_cliff_front_right = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    create_msgs::msg::Cliff_<ContainerAllocator> *;
  using ConstRawPtr =
    const create_msgs::msg::Cliff_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<create_msgs::msg::Cliff_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<create_msgs::msg::Cliff_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      create_msgs::msg::Cliff_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<create_msgs::msg::Cliff_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      create_msgs::msg::Cliff_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<create_msgs::msg::Cliff_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<create_msgs::msg::Cliff_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<create_msgs::msg::Cliff_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__create_msgs__msg__Cliff
    std::shared_ptr<create_msgs::msg::Cliff_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__create_msgs__msg__Cliff
    std::shared_ptr<create_msgs::msg::Cliff_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Cliff_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->is_cliff_left != other.is_cliff_left) {
      return false;
    }
    if (this->is_cliff_front_left != other.is_cliff_front_left) {
      return false;
    }
    if (this->is_cliff_right != other.is_cliff_right) {
      return false;
    }
    if (this->is_cliff_front_right != other.is_cliff_front_right) {
      return false;
    }
    return true;
  }
  bool operator!=(const Cliff_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Cliff_

// alias to use template instance with default allocator
using Cliff =
  create_msgs::msg::Cliff_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace create_msgs

#endif  // CREATE_MSGS__MSG__DETAIL__CLIFF__STRUCT_HPP_
