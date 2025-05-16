// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from create_msgs:msg/DefineSong.idl
// generated code does not contain a copyright notice

#ifndef CREATE_MSGS__MSG__DETAIL__DEFINE_SONG__STRUCT_H_
#define CREATE_MSGS__MSG__DETAIL__DEFINE_SONG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'notes'
// Member 'durations'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/DefineSong in the package create_msgs.
typedef struct create_msgs__msg__DefineSong
{
  /// song number
  uint8_t song;
  /// song length
  uint8_t length;
  /// notes defined by the MIDI note numbering scheme. Notes outside the range of are rest notes.
  rosidl_runtime_c__uint8__Sequence notes;
  /// durations in seconds. Maximum duration is 255/64.
  rosidl_runtime_c__float__Sequence durations;
} create_msgs__msg__DefineSong;

// Struct for a sequence of create_msgs__msg__DefineSong.
typedef struct create_msgs__msg__DefineSong__Sequence
{
  create_msgs__msg__DefineSong * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} create_msgs__msg__DefineSong__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CREATE_MSGS__MSG__DETAIL__DEFINE_SONG__STRUCT_H_
