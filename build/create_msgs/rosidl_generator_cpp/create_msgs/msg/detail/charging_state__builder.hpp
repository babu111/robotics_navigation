// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from create_msgs:msg/ChargingState.idl
// generated code does not contain a copyright notice

#ifndef CREATE_MSGS__MSG__DETAIL__CHARGING_STATE__BUILDER_HPP_
#define CREATE_MSGS__MSG__DETAIL__CHARGING_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "create_msgs/msg/detail/charging_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace create_msgs
{

namespace msg
{

namespace builder
{

class Init_ChargingState_state
{
public:
  explicit Init_ChargingState_state(::create_msgs::msg::ChargingState & msg)
  : msg_(msg)
  {}
  ::create_msgs::msg::ChargingState state(::create_msgs::msg::ChargingState::_state_type arg)
  {
    msg_.state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::create_msgs::msg::ChargingState msg_;
};

class Init_ChargingState_header
{
public:
  Init_ChargingState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ChargingState_state header(::create_msgs::msg::ChargingState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ChargingState_state(msg_);
  }

private:
  ::create_msgs::msg::ChargingState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::create_msgs::msg::ChargingState>()
{
  return create_msgs::msg::builder::Init_ChargingState_header();
}

}  // namespace create_msgs

#endif  // CREATE_MSGS__MSG__DETAIL__CHARGING_STATE__BUILDER_HPP_
