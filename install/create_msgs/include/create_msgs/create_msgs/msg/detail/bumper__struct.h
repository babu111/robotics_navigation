// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from create_msgs:msg/Bumper.idl
// generated code does not contain a copyright notice

#ifndef CREATE_MSGS__MSG__DETAIL__BUMPER__STRUCT_H_
#define CREATE_MSGS__MSG__DETAIL__BUMPER__STRUCT_H_

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

/// Struct defined in msg/Bumper in the package create_msgs.
typedef struct create_msgs__msg__Bumper
{
  std_msgs__msg__Header header;
  /// Contact sensors
  bool is_left_pressed;
  bool is_right_pressed;
  /// Bumper light sensors (Create 2 only) in order from left to right
  /// Value = true if an obstacle detected
  bool is_light_left;
  bool is_light_front_left;
  bool is_light_center_left;
  bool is_light_center_right;
  bool is_light_front_right;
  bool is_light_right;
  /// Raw light sensor signals
  /// Values in range [0, 4095]
  uint16_t light_signal_left;
  uint16_t light_signal_front_left;
  uint16_t light_signal_center_left;
  uint16_t light_signal_center_right;
  uint16_t light_signal_front_right;
  uint16_t light_signal_right;
} create_msgs__msg__Bumper;

// Struct for a sequence of create_msgs__msg__Bumper.
typedef struct create_msgs__msg__Bumper__Sequence
{
  create_msgs__msg__Bumper * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} create_msgs__msg__Bumper__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CREATE_MSGS__MSG__DETAIL__BUMPER__STRUCT_H_
