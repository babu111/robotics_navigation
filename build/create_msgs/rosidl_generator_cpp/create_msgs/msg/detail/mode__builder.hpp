// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from create_msgs:msg/Mode.idl
// generated code does not contain a copyright notice

#ifndef CREATE_MSGS__MSG__DETAIL__MODE__BUILDER_HPP_
#define CREATE_MSGS__MSG__DETAIL__MODE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "create_msgs/msg/detail/mode__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace create_msgs
{

namespace msg
{

namespace builder
{

class Init_Mode_mode
{
public:
  explicit Init_Mode_mode(::create_msgs::msg::Mode & msg)
  : msg_(msg)
  {}
  ::create_msgs::msg::Mode mode(::create_msgs::msg::Mode::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return std::move(msg_);
  }

private:
  ::create_msgs::msg::Mode msg_;
};

class Init_Mode_header
{
public:
  Init_Mode_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Mode_mode header(::create_msgs::msg::Mode::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Mode_mode(msg_);
  }

private:
  ::create_msgs::msg::Mode msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::create_msgs::msg::Mode>()
{
  return create_msgs::msg::builder::Init_Mode_header();
}

}  // namespace create_msgs

#endif  // CREATE_MSGS__MSG__DETAIL__MODE__BUILDER_HPP_
