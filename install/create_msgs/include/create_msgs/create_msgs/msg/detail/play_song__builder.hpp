// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from create_msgs:msg/PlaySong.idl
// generated code does not contain a copyright notice

#ifndef CREATE_MSGS__MSG__DETAIL__PLAY_SONG__BUILDER_HPP_
#define CREATE_MSGS__MSG__DETAIL__PLAY_SONG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "create_msgs/msg/detail/play_song__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace create_msgs
{

namespace msg
{

namespace builder
{

class Init_PlaySong_song
{
public:
  Init_PlaySong_song()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::create_msgs::msg::PlaySong song(::create_msgs::msg::PlaySong::_song_type arg)
  {
    msg_.song = std::move(arg);
    return std::move(msg_);
  }

private:
  ::create_msgs::msg::PlaySong msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::create_msgs::msg::PlaySong>()
{
  return create_msgs::msg::builder::Init_PlaySong_song();
}

}  // namespace create_msgs

#endif  // CREATE_MSGS__MSG__DETAIL__PLAY_SONG__BUILDER_HPP_
