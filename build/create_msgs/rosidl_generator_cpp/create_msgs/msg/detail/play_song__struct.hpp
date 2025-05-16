// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from create_msgs:msg/PlaySong.idl
// generated code does not contain a copyright notice

#ifndef CREATE_MSGS__MSG__DETAIL__PLAY_SONG__STRUCT_HPP_
#define CREATE_MSGS__MSG__DETAIL__PLAY_SONG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__create_msgs__msg__PlaySong __attribute__((deprecated))
#else
# define DEPRECATED__create_msgs__msg__PlaySong __declspec(deprecated)
#endif

namespace create_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PlaySong_
{
  using Type = PlaySong_<ContainerAllocator>;

  explicit PlaySong_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->song = 0;
    }
  }

  explicit PlaySong_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->song = 0;
    }
  }

  // field types and members
  using _song_type =
    uint8_t;
  _song_type song;

  // setters for named parameter idiom
  Type & set__song(
    const uint8_t & _arg)
  {
    this->song = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    create_msgs::msg::PlaySong_<ContainerAllocator> *;
  using ConstRawPtr =
    const create_msgs::msg::PlaySong_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<create_msgs::msg::PlaySong_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<create_msgs::msg::PlaySong_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      create_msgs::msg::PlaySong_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<create_msgs::msg::PlaySong_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      create_msgs::msg::PlaySong_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<create_msgs::msg::PlaySong_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<create_msgs::msg::PlaySong_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<create_msgs::msg::PlaySong_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__create_msgs__msg__PlaySong
    std::shared_ptr<create_msgs::msg::PlaySong_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__create_msgs__msg__PlaySong
    std::shared_ptr<create_msgs::msg::PlaySong_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlaySong_ & other) const
  {
    if (this->song != other.song) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlaySong_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlaySong_

// alias to use template instance with default allocator
using PlaySong =
  create_msgs::msg::PlaySong_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace create_msgs

#endif  // CREATE_MSGS__MSG__DETAIL__PLAY_SONG__STRUCT_HPP_
