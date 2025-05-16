// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from create_msgs:msg/PlaySong.idl
// generated code does not contain a copyright notice

#ifndef CREATE_MSGS__MSG__DETAIL__PLAY_SONG__STRUCT_H_
#define CREATE_MSGS__MSG__DETAIL__PLAY_SONG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/PlaySong in the package create_msgs.
typedef struct create_msgs__msg__PlaySong
{
  /// song number
  uint8_t song;
} create_msgs__msg__PlaySong;

// Struct for a sequence of create_msgs__msg__PlaySong.
typedef struct create_msgs__msg__PlaySong__Sequence
{
  create_msgs__msg__PlaySong * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} create_msgs__msg__PlaySong__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CREATE_MSGS__MSG__DETAIL__PLAY_SONG__STRUCT_H_
