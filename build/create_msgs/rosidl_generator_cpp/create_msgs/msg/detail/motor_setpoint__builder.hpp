// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from create_msgs:msg/MotorSetpoint.idl
// generated code does not contain a copyright notice

#ifndef CREATE_MSGS__MSG__DETAIL__MOTOR_SETPOINT__BUILDER_HPP_
#define CREATE_MSGS__MSG__DETAIL__MOTOR_SETPOINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "create_msgs/msg/detail/motor_setpoint__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace create_msgs
{

namespace msg
{

namespace builder
{

class Init_MotorSetpoint_duty_cycle
{
public:
  Init_MotorSetpoint_duty_cycle()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::create_msgs::msg::MotorSetpoint duty_cycle(::create_msgs::msg::MotorSetpoint::_duty_cycle_type arg)
  {
    msg_.duty_cycle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::create_msgs::msg::MotorSetpoint msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::create_msgs::msg::MotorSetpoint>()
{
  return create_msgs::msg::builder::Init_MotorSetpoint_duty_cycle();
}

}  // namespace create_msgs

#endif  // CREATE_MSGS__MSG__DETAIL__MOTOR_SETPOINT__BUILDER_HPP_
