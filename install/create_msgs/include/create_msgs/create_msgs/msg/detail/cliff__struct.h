// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from create_msgs:msg/Cliff.idl
// generated code does not contain a copyright notice

#ifndef CREATE_MSGS__MSG__DETAIL__CLIFF__STRUCT_H_
#define CREATE_MSGS__MSG__DETAIL__CLIFF__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/Cliff in the package create_msgs.
typedef struct create_msgs__msg__Cliff
{
  std_msgs__msg__Header header;
  bool is_cliff_left;
  bool is_cliff_front_left;
  bool is_cliff_right;
  bool is_cliff_front_right;
} create_msgs__msg__Cliff;

// Struct for a sequence of create_msgs__msg__Cliff.
typedef struct create_msgs__msg__Cliff__Sequence
{
  create_msgs__msg__Cliff * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} create_msgs__msg__Cliff__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CREATE_MSGS__MSG__DETAIL__CLIFF__STRUCT_H_
