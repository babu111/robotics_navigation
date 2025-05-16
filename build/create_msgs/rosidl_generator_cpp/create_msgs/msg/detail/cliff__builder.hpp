// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from create_msgs:msg/Cliff.idl
// generated code does not contain a copyright notice

#ifndef CREATE_MSGS__MSG__DETAIL__CLIFF__BUILDER_HPP_
#define CREATE_MSGS__MSG__DETAIL__CLIFF__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "create_msgs/msg/detail/cliff__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace create_msgs
{

namespace msg
{

namespace builder
{

class Init_Cliff_is_cliff_front_right
{
public:
  explicit Init_Cliff_is_cliff_front_right(::create_msgs::msg::Cliff & msg)
  : msg_(msg)
  {}
  ::create_msgs::msg::Cliff is_cliff_front_right(::create_msgs::msg::Cliff::_is_cliff_front_right_type arg)
  {
    msg_.is_cliff_front_right = std::move(arg);
    return std::move(msg_);
  }

private:
  ::create_msgs::msg::Cliff msg_;
};

class Init_Cliff_is_cliff_right
{
public:
  explicit Init_Cliff_is_cliff_right(::create_msgs::msg::Cliff & msg)
  : msg_(msg)
  {}
  Init_Cliff_is_cliff_front_right is_cliff_right(::create_msgs::msg::Cliff::_is_cliff_right_type arg)
  {
    msg_.is_cliff_right = std::move(arg);
    return Init_Cliff_is_cliff_front_right(msg_);
  }

private:
  ::create_msgs::msg::Cliff msg_;
};

class Init_Cliff_is_cliff_front_left
{
public:
  explicit Init_Cliff_is_cliff_front_left(::create_msgs::msg::Cliff & msg)
  : msg_(msg)
  {}
  Init_Cliff_is_cliff_right is_cliff_front_left(::create_msgs::msg::Cliff::_is_cliff_front_left_type arg)
  {
    msg_.is_cliff_front_left = std::move(arg);
    return Init_Cliff_is_cliff_right(msg_);
  }

private:
  ::create_msgs::msg::Cliff msg_;
};

class Init_Cliff_is_cliff_left
{
public:
  explicit Init_Cliff_is_cliff_left(::create_msgs::msg::Cliff & msg)
  : msg_(msg)
  {}
  Init_Cliff_is_cliff_front_left is_cliff_left(::create_msgs::msg::Cliff::_is_cliff_left_type arg)
  {
    msg_.is_cliff_left = std::move(arg);
    return Init_Cliff_is_cliff_front_left(msg_);
  }

private:
  ::create_msgs::msg::Cliff msg_;
};

class Init_Cliff_header
{
public:
  Init_Cliff_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Cliff_is_cliff_left header(::create_msgs::msg::Cliff::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Cliff_is_cliff_left(msg_);
  }

private:
  ::create_msgs::msg::Cliff msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::create_msgs::msg::Cliff>()
{
  return create_msgs::msg::builder::Init_Cliff_header();
}

}  // namespace create_msgs

#endif  // CREATE_MSGS__MSG__DETAIL__CLIFF__BUILDER_HPP_
