// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from create_msgs:msg/ChargingState.idl
// generated code does not contain a copyright notice

#ifndef CREATE_MSGS__MSG__DETAIL__CHARGING_STATE__STRUCT_H_
#define CREATE_MSGS__MSG__DETAIL__CHARGING_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'CHARGE_NONE'.
enum
{
  create_msgs__msg__ChargingState__CHARGE_NONE = 0
};

/// Constant 'CHARGE_RECONDITION'.
enum
{
  create_msgs__msg__ChargingState__CHARGE_RECONDITION = 1
};

/// Constant 'CHARGE_FULL'.
enum
{
  create_msgs__msg__ChargingState__CHARGE_FULL = 2
};

/// Constant 'CHARGE_TRICKLE'.
enum
{
  create_msgs__msg__ChargingState__CHARGE_TRICKLE = 3
};

/// Constant 'CHARGE_WAITING'.
enum
{
  create_msgs__msg__ChargingState__CHARGE_WAITING = 4
};

/// Constant 'CHARGE_FAULT'.
enum
{
  create_msgs__msg__ChargingState__CHARGE_FAULT = 5
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/ChargingState in the package create_msgs.
typedef struct create_msgs__msg__ChargingState
{
  std_msgs__msg__Header header;
  uint8_t state;
} create_msgs__msg__ChargingState;

// Struct for a sequence of create_msgs__msg__ChargingState.
typedef struct create_msgs__msg__ChargingState__Sequence
{
  create_msgs__msg__ChargingState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} create_msgs__msg__ChargingState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CREATE_MSGS__MSG__DETAIL__CHARGING_STATE__STRUCT_H_
