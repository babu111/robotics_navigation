// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from create_msgs:msg/DefineSong.idl
// generated code does not contain a copyright notice
#include "create_msgs/msg/detail/define_song__rosidl_typesupport_fastrtps_cpp.hpp"
#include "create_msgs/msg/detail/define_song__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace create_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_create_msgs
cdr_serialize(
  const create_msgs::msg::DefineSong & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: song
  cdr << ros_message.song;
  // Member: length
  cdr << ros_message.length;
  // Member: notes
  {
    cdr << ros_message.notes;
  }
  // Member: durations
  {
    cdr << ros_message.durations;
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_create_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  create_msgs::msg::DefineSong & ros_message)
{
  // Member: song
  cdr >> ros_message.song;

  // Member: length
  cdr >> ros_message.length;

  // Member: notes
  {
    cdr >> ros_message.notes;
  }

  // Member: durations
  {
    cdr >> ros_message.durations;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_create_msgs
get_serialized_size(
  const create_msgs::msg::DefineSong & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: song
  {
    size_t item_size = sizeof(ros_message.song);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: length
  {
    size_t item_size = sizeof(ros_message.length);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: notes
  {
    size_t array_size = ros_message.notes.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.notes[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: durations
  {
    size_t array_size = ros_message.durations.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.durations[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_create_msgs
max_serialized_size_DefineSong(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: song
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: length
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: notes
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: durations
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = create_msgs::msg::DefineSong;
    is_plain =
      (
      offsetof(DataType, durations) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _DefineSong__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const create_msgs::msg::DefineSong *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _DefineSong__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<create_msgs::msg::DefineSong *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _DefineSong__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const create_msgs::msg::DefineSong *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _DefineSong__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_DefineSong(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _DefineSong__callbacks = {
  "create_msgs::msg",
  "DefineSong",
  _DefineSong__cdr_serialize,
  _DefineSong__cdr_deserialize,
  _DefineSong__get_serialized_size,
  _DefineSong__max_serialized_size
};

static rosidl_message_type_support_t _DefineSong__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_DefineSong__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace create_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_create_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<create_msgs::msg::DefineSong>()
{
  return &create_msgs::msg::typesupport_fastrtps_cpp::_DefineSong__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, create_msgs, msg, DefineSong)() {
  return &create_msgs::msg::typesupport_fastrtps_cpp::_DefineSong__handle;
}

#ifdef __cplusplus
}
#endif
