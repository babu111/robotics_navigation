// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from create_msgs:msg/DefineSong.idl
// generated code does not contain a copyright notice

#ifndef CREATE_MSGS__MSG__DETAIL__DEFINE_SONG__BUILDER_HPP_
#define CREATE_MSGS__MSG__DETAIL__DEFINE_SONG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "create_msgs/msg/detail/define_song__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace create_msgs
{

namespace msg
{

namespace builder
{

class Init_DefineSong_durations
{
public:
  explicit Init_DefineSong_durations(::create_msgs::msg::DefineSong & msg)
  : msg_(msg)
  {}
  ::create_msgs::msg::DefineSong durations(::create_msgs::msg::DefineSong::_durations_type arg)
  {
    msg_.durations = std::move(arg);
    return std::move(msg_);
  }

private:
  ::create_msgs::msg::DefineSong msg_;
};

class Init_DefineSong_notes
{
public:
  explicit Init_DefineSong_notes(::create_msgs::msg::DefineSong & msg)
  : msg_(msg)
  {}
  Init_DefineSong_durations notes(::create_msgs::msg::DefineSong::_notes_type arg)
  {
    msg_.notes = std::move(arg);
    return Init_DefineSong_durations(msg_);
  }

private:
  ::create_msgs::msg::DefineSong msg_;
};

class Init_DefineSong_length
{
public:
  explicit Init_DefineSong_length(::create_msgs::msg::DefineSong & msg)
  : msg_(msg)
  {}
  Init_DefineSong_notes length(::create_msgs::msg::DefineSong::_length_type arg)
  {
    msg_.length = std::move(arg);
    return Init_DefineSong_notes(msg_);
  }

private:
  ::create_msgs::msg::DefineSong msg_;
};

class Init_DefineSong_song
{
public:
  Init_DefineSong_song()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DefineSong_length song(::create_msgs::msg::DefineSong::_song_type arg)
  {
    msg_.song = std::move(arg);
    return Init_DefineSong_length(msg_);
  }

private:
  ::create_msgs::msg::DefineSong msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::create_msgs::msg::DefineSong>()
{
  return create_msgs::msg::builder::Init_DefineSong_song();
}

}  // namespace create_msgs

#endif  // CREATE_MSGS__MSG__DETAIL__DEFINE_SONG__BUILDER_HPP_
