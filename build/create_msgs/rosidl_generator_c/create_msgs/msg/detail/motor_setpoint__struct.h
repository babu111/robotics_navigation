// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from create_msgs:msg/MotorSetpoint.idl
// generated code does not contain a copyright notice

#ifndef CREATE_MSGS__MSG__DETAIL__MOTOR_SETPOINT__STRUCT_H_
#define CREATE_MSGS__MSG__DETAIL__MOTOR_SETPOINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/MotorSetpoint in the package create_msgs.
/**
  * For the main and side brush motors, provide a duty cycle in the range [-1, 1]
  * The range of acceptable values for the vacuum motor is [0, 1]
 */
typedef struct create_msgs__msg__MotorSetpoint
{
  float duty_cycle;
} create_msgs__msg__MotorSetpoint;

// Struct for a sequence of create_msgs__msg__MotorSetpoint.
typedef struct create_msgs__msg__MotorSetpoint__Sequence
{
  create_msgs__msg__MotorSetpoint * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} create_msgs__msg__MotorSetpoint__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CREATE_MSGS__MSG__DETAIL__MOTOR_SETPOINT__STRUCT_H_
