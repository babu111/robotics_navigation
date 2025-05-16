// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from create_msgs:msg/Mode.idl
// generated code does not contain a copyright notice

#ifndef CREATE_MSGS__MSG__DETAIL__MODE__STRUCT_H_
#define CREATE_MSGS__MSG__DETAIL__MODE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'MODE_OFF'.
enum
{
  create_msgs__msg__Mode__MODE_OFF = 0
};

/// Constant 'MODE_PASSIVE'.
enum
{
  create_msgs__msg__Mode__MODE_PASSIVE = 1
};

/// Constant 'MODE_SAFE'.
enum
{
  create_msgs__msg__Mode__MODE_SAFE = 2
};

/// Constant 'MODE_FULL'.
enum
{
  create_msgs__msg__Mode__MODE_FULL = 3
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/Mode in the package create_msgs.
typedef struct create_msgs__msg__Mode
{
  std_msgs__msg__Header header;
  uint8_t mode;
} create_msgs__msg__Mode;

// Struct for a sequence of create_msgs__msg__Mode.
typedef struct create_msgs__msg__Mode__Sequence
{
  create_msgs__msg__Mode * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} create_msgs__msg__Mode__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CREATE_MSGS__MSG__DETAIL__MODE__STRUCT_H_
