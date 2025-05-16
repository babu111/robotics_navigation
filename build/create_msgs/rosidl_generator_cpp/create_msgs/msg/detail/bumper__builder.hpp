// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from create_msgs:msg/Bumper.idl
// generated code does not contain a copyright notice

#ifndef CREATE_MSGS__MSG__DETAIL__BUMPER__BUILDER_HPP_
#define CREATE_MSGS__MSG__DETAIL__BUMPER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "create_msgs/msg/detail/bumper__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace create_msgs
{

namespace msg
{

namespace builder
{

class Init_Bumper_light_signal_right
{
public:
  explicit Init_Bumper_light_signal_right(::create_msgs::msg::Bumper & msg)
  : msg_(msg)
  {}
  ::create_msgs::msg::Bumper light_signal_right(::create_msgs::msg::Bumper::_light_signal_right_type arg)
  {
    msg_.light_signal_right = std::move(arg);
    return std::move(msg_);
  }

private:
  ::create_msgs::msg::Bumper msg_;
};

class Init_Bumper_light_signal_front_right
{
public:
  explicit Init_Bumper_light_signal_front_right(::create_msgs::msg::Bumper & msg)
  : msg_(msg)
  {}
  Init_Bumper_light_signal_right light_signal_front_right(::create_msgs::msg::Bumper::_light_signal_front_right_type arg)
  {
    msg_.light_signal_front_right = std::move(arg);
    return Init_Bumper_light_signal_right(msg_);
  }

private:
  ::create_msgs::msg::Bumper msg_;
};

class Init_Bumper_light_signal_center_right
{
public:
  explicit Init_Bumper_light_signal_center_right(::create_msgs::msg::Bumper & msg)
  : msg_(msg)
  {}
  Init_Bumper_light_signal_front_right light_signal_center_right(::create_msgs::msg::Bumper::_light_signal_center_right_type arg)
  {
    msg_.light_signal_center_right = std::move(arg);
    return Init_Bumper_light_signal_front_right(msg_);
  }

private:
  ::create_msgs::msg::Bumper msg_;
};

class Init_Bumper_light_signal_center_left
{
public:
  explicit Init_Bumper_light_signal_center_left(::create_msgs::msg::Bumper & msg)
  : msg_(msg)
  {}
  Init_Bumper_light_signal_center_right light_signal_center_left(::create_msgs::msg::Bumper::_light_signal_center_left_type arg)
  {
    msg_.light_signal_center_left = std::move(arg);
    return Init_Bumper_light_signal_center_right(msg_);
  }

private:
  ::create_msgs::msg::Bumper msg_;
};

class Init_Bumper_light_signal_front_left
{
public:
  explicit Init_Bumper_light_signal_front_left(::create_msgs::msg::Bumper & msg)
  : msg_(msg)
  {}
  Init_Bumper_light_signal_center_left light_signal_front_left(::create_msgs::msg::Bumper::_light_signal_front_left_type arg)
  {
    msg_.light_signal_front_left = std::move(arg);
    return Init_Bumper_light_signal_center_left(msg_);
  }

private:
  ::create_msgs::msg::Bumper msg_;
};

class Init_Bumper_light_signal_left
{
public:
  explicit Init_Bumper_light_signal_left(::create_msgs::msg::Bumper & msg)
  : msg_(msg)
  {}
  Init_Bumper_light_signal_front_left light_signal_left(::create_msgs::msg::Bumper::_light_signal_left_type arg)
  {
    msg_.light_signal_left = std::move(arg);
    return Init_Bumper_light_signal_front_left(msg_);
  }

private:
  ::create_msgs::msg::Bumper msg_;
};

class Init_Bumper_is_light_right
{
public:
  explicit Init_Bumper_is_light_right(::create_msgs::msg::Bumper & msg)
  : msg_(msg)
  {}
  Init_Bumper_light_signal_left is_light_right(::create_msgs::msg::Bumper::_is_light_right_type arg)
  {
    msg_.is_light_right = std::move(arg);
    return Init_Bumper_light_signal_left(msg_);
  }

private:
  ::create_msgs::msg::Bumper msg_;
};

class Init_Bumper_is_light_front_right
{
public:
  explicit Init_Bumper_is_light_front_right(::create_msgs::msg::Bumper & msg)
  : msg_(msg)
  {}
  Init_Bumper_is_light_right is_light_front_right(::create_msgs::msg::Bumper::_is_light_front_right_type arg)
  {
    msg_.is_light_front_right = std::move(arg);
    return Init_Bumper_is_light_right(msg_);
  }

private:
  ::create_msgs::msg::Bumper msg_;
};

class Init_Bumper_is_light_center_right
{
public:
  explicit Init_Bumper_is_light_center_right(::create_msgs::msg::Bumper & msg)
  : msg_(msg)
  {}
  Init_Bumper_is_light_front_right is_light_center_right(::create_msgs::msg::Bumper::_is_light_center_right_type arg)
  {
    msg_.is_light_center_right = std::move(arg);
    return Init_Bumper_is_light_front_right(msg_);
  }

private:
  ::create_msgs::msg::Bumper msg_;
};

class Init_Bumper_is_light_center_left
{
public:
  explicit Init_Bumper_is_light_center_left(::create_msgs::msg::Bumper & msg)
  : msg_(msg)
  {}
  Init_Bumper_is_light_center_right is_light_center_left(::create_msgs::msg::Bumper::_is_light_center_left_type arg)
  {
    msg_.is_light_center_left = std::move(arg);
    return Init_Bumper_is_light_center_right(msg_);
  }

private:
  ::create_msgs::msg::Bumper msg_;
};

class Init_Bumper_is_light_front_left
{
public:
  explicit Init_Bumper_is_light_front_left(::create_msgs::msg::Bumper & msg)
  : msg_(msg)
  {}
  Init_Bumper_is_light_center_left is_light_front_left(::create_msgs::msg::Bumper::_is_light_front_left_type arg)
  {
    msg_.is_light_front_left = std::move(arg);
    return Init_Bumper_is_light_center_left(msg_);
  }

private:
  ::create_msgs::msg::Bumper msg_;
};

class Init_Bumper_is_light_left
{
public:
  explicit Init_Bumper_is_light_left(::create_msgs::msg::Bumper & msg)
  : msg_(msg)
  {}
  Init_Bumper_is_light_front_left is_light_left(::create_msgs::msg::Bumper::_is_light_left_type arg)
  {
    msg_.is_light_left = std::move(arg);
    return Init_Bumper_is_light_front_left(msg_);
  }

private:
  ::create_msgs::msg::Bumper msg_;
};

class Init_Bumper_is_right_pressed
{
public:
  explicit Init_Bumper_is_right_pressed(::create_msgs::msg::Bumper & msg)
  : msg_(msg)
  {}
  Init_Bumper_is_light_left is_right_pressed(::create_msgs::msg::Bumper::_is_right_pressed_type arg)
  {
    msg_.is_right_pressed = std::move(arg);
    return Init_Bumper_is_light_left(msg_);
  }

private:
  ::create_msgs::msg::Bumper msg_;
};

class Init_Bumper_is_left_pressed
{
public:
  explicit Init_Bumper_is_left_pressed(::create_msgs::msg::Bumper & msg)
  : msg_(msg)
  {}
  Init_Bumper_is_right_pressed is_left_pressed(::create_msgs::msg::Bumper::_is_left_pressed_type arg)
  {
    msg_.is_left_pressed = std::move(arg);
    return Init_Bumper_is_right_pressed(msg_);
  }

private:
  ::create_msgs::msg::Bumper msg_;
};

class Init_Bumper_header
{
public:
  Init_Bumper_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Bumper_is_left_pressed header(::create_msgs::msg::Bumper::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Bumper_is_left_pressed(msg_);
  }

private:
  ::create_msgs::msg::Bumper msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::create_msgs::msg::Bumper>()
{
  return create_msgs::msg::builder::Init_Bumper_header();
}

}  // namespace create_msgs

#endif  // CREATE_MSGS__MSG__DETAIL__BUMPER__BUILDER_HPP_
