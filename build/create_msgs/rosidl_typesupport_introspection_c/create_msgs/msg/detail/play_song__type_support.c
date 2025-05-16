// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from create_msgs:msg/PlaySong.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "create_msgs/msg/detail/play_song__rosidl_typesupport_introspection_c.h"
#include "create_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "create_msgs/msg/detail/play_song__functions.h"
#include "create_msgs/msg/detail/play_song__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void create_msgs__msg__PlaySong__rosidl_typesupport_introspection_c__PlaySong_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  create_msgs__msg__PlaySong__init(message_memory);
}

void create_msgs__msg__PlaySong__rosidl_typesupport_introspection_c__PlaySong_fini_function(void * message_memory)
{
  create_msgs__msg__PlaySong__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember create_msgs__msg__PlaySong__rosidl_typesupport_introspection_c__PlaySong_message_member_array[1] = {
  {
    "song",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(create_msgs__msg__PlaySong, song),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers create_msgs__msg__PlaySong__rosidl_typesupport_introspection_c__PlaySong_message_members = {
  "create_msgs__msg",  // message namespace
  "PlaySong",  // message name
  1,  // number of fields
  sizeof(create_msgs__msg__PlaySong),
  create_msgs__msg__PlaySong__rosidl_typesupport_introspection_c__PlaySong_message_member_array,  // message members
  create_msgs__msg__PlaySong__rosidl_typesupport_introspection_c__PlaySong_init_function,  // function to initialize message memory (memory has to be allocated)
  create_msgs__msg__PlaySong__rosidl_typesupport_introspection_c__PlaySong_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t create_msgs__msg__PlaySong__rosidl_typesupport_introspection_c__PlaySong_message_type_support_handle = {
  0,
  &create_msgs__msg__PlaySong__rosidl_typesupport_introspection_c__PlaySong_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_create_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, create_msgs, msg, PlaySong)() {
  if (!create_msgs__msg__PlaySong__rosidl_typesupport_introspection_c__PlaySong_message_type_support_handle.typesupport_identifier) {
    create_msgs__msg__PlaySong__rosidl_typesupport_introspection_c__PlaySong_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &create_msgs__msg__PlaySong__rosidl_typesupport_introspection_c__PlaySong_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
