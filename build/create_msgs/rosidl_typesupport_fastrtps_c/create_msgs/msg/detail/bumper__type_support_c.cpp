// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from create_msgs:msg/Bumper.idl
// generated code does not contain a copyright notice
#include "create_msgs/msg/detail/bumper__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "create_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "create_msgs/msg/detail/bumper__struct.h"
#include "create_msgs/msg/detail/bumper__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "std_msgs/msg/detail/header__functions.h"  // header

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_create_msgs
size_t get_serialized_size_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_create_msgs
size_t max_serialized_size_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_create_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();


using _Bumper__ros_msg_type = create_msgs__msg__Bumper;

static bool _Bumper__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Bumper__ros_msg_type * ros_message = static_cast<const _Bumper__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->header, cdr))
    {
      return false;
    }
  }

  // Field name: is_left_pressed
  {
    cdr << (ros_message->is_left_pressed ? true : false);
  }

  // Field name: is_right_pressed
  {
    cdr << (ros_message->is_right_pressed ? true : false);
  }

  // Field name: is_light_left
  {
    cdr << (ros_message->is_light_left ? true : false);
  }

  // Field name: is_light_front_left
  {
    cdr << (ros_message->is_light_front_left ? true : false);
  }

  // Field name: is_light_center_left
  {
    cdr << (ros_message->is_light_center_left ? true : false);
  }

  // Field name: is_light_center_right
  {
    cdr << (ros_message->is_light_center_right ? true : false);
  }

  // Field name: is_light_front_right
  {
    cdr << (ros_message->is_light_front_right ? true : false);
  }

  // Field name: is_light_right
  {
    cdr << (ros_message->is_light_right ? true : false);
  }

  // Field name: light_signal_left
  {
    cdr << ros_message->light_signal_left;
  }

  // Field name: light_signal_front_left
  {
    cdr << ros_message->light_signal_front_left;
  }

  // Field name: light_signal_center_left
  {
    cdr << ros_message->light_signal_center_left;
  }

  // Field name: light_signal_center_right
  {
    cdr << ros_message->light_signal_center_right;
  }

  // Field name: light_signal_front_right
  {
    cdr << ros_message->light_signal_front_right;
  }

  // Field name: light_signal_right
  {
    cdr << ros_message->light_signal_right;
  }

  return true;
}

static bool _Bumper__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Bumper__ros_msg_type * ros_message = static_cast<_Bumper__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->header))
    {
      return false;
    }
  }

  // Field name: is_left_pressed
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->is_left_pressed = tmp ? true : false;
  }

  // Field name: is_right_pressed
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->is_right_pressed = tmp ? true : false;
  }

  // Field name: is_light_left
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->is_light_left = tmp ? true : false;
  }

  // Field name: is_light_front_left
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->is_light_front_left = tmp ? true : false;
  }

  // Field name: is_light_center_left
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->is_light_center_left = tmp ? true : false;
  }

  // Field name: is_light_center_right
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->is_light_center_right = tmp ? true : false;
  }

  // Field name: is_light_front_right
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->is_light_front_right = tmp ? true : false;
  }

  // Field name: is_light_right
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->is_light_right = tmp ? true : false;
  }

  // Field name: light_signal_left
  {
    cdr >> ros_message->light_signal_left;
  }

  // Field name: light_signal_front_left
  {
    cdr >> ros_message->light_signal_front_left;
  }

  // Field name: light_signal_center_left
  {
    cdr >> ros_message->light_signal_center_left;
  }

  // Field name: light_signal_center_right
  {
    cdr >> ros_message->light_signal_center_right;
  }

  // Field name: light_signal_front_right
  {
    cdr >> ros_message->light_signal_front_right;
  }

  // Field name: light_signal_right
  {
    cdr >> ros_message->light_signal_right;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_create_msgs
size_t get_serialized_size_create_msgs__msg__Bumper(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Bumper__ros_msg_type * ros_message = static_cast<const _Bumper__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name header

  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);
  // field.name is_left_pressed
  {
    size_t item_size = sizeof(ros_message->is_left_pressed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name is_right_pressed
  {
    size_t item_size = sizeof(ros_message->is_right_pressed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name is_light_left
  {
    size_t item_size = sizeof(ros_message->is_light_left);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name is_light_front_left
  {
    size_t item_size = sizeof(ros_message->is_light_front_left);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name is_light_center_left
  {
    size_t item_size = sizeof(ros_message->is_light_center_left);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name is_light_center_right
  {
    size_t item_size = sizeof(ros_message->is_light_center_right);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name is_light_front_right
  {
    size_t item_size = sizeof(ros_message->is_light_front_right);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name is_light_right
  {
    size_t item_size = sizeof(ros_message->is_light_right);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name light_signal_left
  {
    size_t item_size = sizeof(ros_message->light_signal_left);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name light_signal_front_left
  {
    size_t item_size = sizeof(ros_message->light_signal_front_left);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name light_signal_center_left
  {
    size_t item_size = sizeof(ros_message->light_signal_center_left);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name light_signal_center_right
  {
    size_t item_size = sizeof(ros_message->light_signal_center_right);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name light_signal_front_right
  {
    size_t item_size = sizeof(ros_message->light_signal_front_right);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name light_signal_right
  {
    size_t item_size = sizeof(ros_message->light_signal_right);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _Bumper__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_create_msgs__msg__Bumper(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_create_msgs
size_t max_serialized_size_create_msgs__msg__Bumper(
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

  // member: header
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_std_msgs__msg__Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: is_left_pressed
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: is_right_pressed
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: is_light_left
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: is_light_front_left
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: is_light_center_left
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: is_light_center_right
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: is_light_front_right
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: is_light_right
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: light_signal_left
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: light_signal_front_left
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: light_signal_center_left
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: light_signal_center_right
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: light_signal_front_right
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: light_signal_right
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = create_msgs__msg__Bumper;
    is_plain =
      (
      offsetof(DataType, light_signal_right) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _Bumper__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_create_msgs__msg__Bumper(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Bumper = {
  "create_msgs::msg",
  "Bumper",
  _Bumper__cdr_serialize,
  _Bumper__cdr_deserialize,
  _Bumper__get_serialized_size,
  _Bumper__max_serialized_size
};

static rosidl_message_type_support_t _Bumper__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Bumper,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, create_msgs, msg, Bumper)() {
  return &_Bumper__type_support;
}

#if defined(__cplusplus)
}
#endif
