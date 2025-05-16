// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from create_msgs:msg/MotorSetpoint.idl
// generated code does not contain a copyright notice

#ifndef CREATE_MSGS__MSG__DETAIL__MOTOR_SETPOINT__STRUCT_HPP_
#define CREATE_MSGS__MSG__DETAIL__MOTOR_SETPOINT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__create_msgs__msg__MotorSetpoint __attribute__((deprecated))
#else
# define DEPRECATED__create_msgs__msg__MotorSetpoint __declspec(deprecated)
#endif

namespace create_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotorSetpoint_
{
  using Type = MotorSetpoint_<ContainerAllocator>;

  explicit MotorSetpoint_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->duty_cycle = 0.0f;
    }
  }

  explicit MotorSetpoint_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->duty_cycle = 0.0f;
    }
  }

  // field types and members
  using _duty_cycle_type =
    float;
  _duty_cycle_type duty_cycle;

  // setters for named parameter idiom
  Type & set__duty_cycle(
    const float & _arg)
  {
    this->duty_cycle = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    create_msgs::msg::MotorSetpoint_<ContainerAllocator> *;
  using ConstRawPtr =
    const create_msgs::msg::MotorSetpoint_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<create_msgs::msg::MotorSetpoint_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<create_msgs::msg::MotorSetpoint_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      create_msgs::msg::MotorSetpoint_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<create_msgs::msg::MotorSetpoint_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      create_msgs::msg::MotorSetpoint_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<create_msgs::msg::MotorSetpoint_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<create_msgs::msg::MotorSetpoint_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<create_msgs::msg::MotorSetpoint_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__create_msgs__msg__MotorSetpoint
    std::shared_ptr<create_msgs::msg::MotorSetpoint_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__create_msgs__msg__MotorSetpoint
    std::shared_ptr<create_msgs::msg::MotorSetpoint_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorSetpoint_ & other) const
  {
    if (this->duty_cycle != other.duty_cycle) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorSetpoint_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorSetpoint_

// alias to use template instance with default allocator
using MotorSetpoint =
  create_msgs::msg::MotorSetpoint_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace create_msgs

#endif  // CREATE_MSGS__MSG__DETAIL__MOTOR_SETPOINT__STRUCT_HPP_
