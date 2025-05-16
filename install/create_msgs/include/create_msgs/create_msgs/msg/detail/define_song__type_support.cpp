// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from create_msgs:msg/DefineSong.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "create_msgs/msg/detail/define_song__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace create_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void DefineSong_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) create_msgs::msg::DefineSong(_init);
}

void DefineSong_fini_function(void * message_memory)
{
  auto typed_message = static_cast<create_msgs::msg::DefineSong *>(message_memory);
  typed_message->~DefineSong();
}

size_t size_function__DefineSong__notes(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<uint8_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__DefineSong__notes(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<uint8_t> *>(untyped_member);
  return &member[index];
}

void * get_function__DefineSong__notes(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<uint8_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__DefineSong__notes(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint8_t *>(
    get_const_function__DefineSong__notes(untyped_member, index));
  auto & value = *reinterpret_cast<uint8_t *>(untyped_value);
  value = item;
}

void assign_function__DefineSong__notes(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint8_t *>(
    get_function__DefineSong__notes(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint8_t *>(untyped_value);
  item = value;
}

void resize_function__DefineSong__notes(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<uint8_t> *>(untyped_member);
  member->resize(size);
}

size_t size_function__DefineSong__durations(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__DefineSong__durations(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__DefineSong__durations(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__DefineSong__durations(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__DefineSong__durations(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__DefineSong__durations(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__DefineSong__durations(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__DefineSong__durations(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember DefineSong_message_member_array[4] = {
  {
    "song",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(create_msgs::msg::DefineSong, song),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "length",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(create_msgs::msg::DefineSong, length),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "notes",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(create_msgs::msg::DefineSong, notes),  // bytes offset in struct
    nullptr,  // default value
    size_function__DefineSong__notes,  // size() function pointer
    get_const_function__DefineSong__notes,  // get_const(index) function pointer
    get_function__DefineSong__notes,  // get(index) function pointer
    fetch_function__DefineSong__notes,  // fetch(index, &value) function pointer
    assign_function__DefineSong__notes,  // assign(index, value) function pointer
    resize_function__DefineSong__notes  // resize(index) function pointer
  },
  {
    "durations",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(create_msgs::msg::DefineSong, durations),  // bytes offset in struct
    nullptr,  // default value
    size_function__DefineSong__durations,  // size() function pointer
    get_const_function__DefineSong__durations,  // get_const(index) function pointer
    get_function__DefineSong__durations,  // get(index) function pointer
    fetch_function__DefineSong__durations,  // fetch(index, &value) function pointer
    assign_function__DefineSong__durations,  // assign(index, value) function pointer
    resize_function__DefineSong__durations  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers DefineSong_message_members = {
  "create_msgs::msg",  // message namespace
  "DefineSong",  // message name
  4,  // number of fields
  sizeof(create_msgs::msg::DefineSong),
  DefineSong_message_member_array,  // message members
  DefineSong_init_function,  // function to initialize message memory (memory has to be allocated)
  DefineSong_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t DefineSong_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &DefineSong_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace create_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<create_msgs::msg::DefineSong>()
{
  return &::create_msgs::msg::rosidl_typesupport_introspection_cpp::DefineSong_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, create_msgs, msg, DefineSong)() {
  return &::create_msgs::msg::rosidl_typesupport_introspection_cpp::DefineSong_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
