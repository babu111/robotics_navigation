// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from create_msgs:msg/MotorSetpoint.idl
// generated code does not contain a copyright notice

#ifndef CREATE_MSGS__MSG__DETAIL__MOTOR_SETPOINT__TRAITS_HPP_
#define CREATE_MSGS__MSG__DETAIL__MOTOR_SETPOINT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "create_msgs/msg/detail/motor_setpoint__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace create_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const MotorSetpoint & msg,
  std::ostream & out)
{
  out << "{";
  // member: duty_cycle
  {
    out << "duty_cycle: ";
    rosidl_generator_traits::value_to_yaml(msg.duty_cycle, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MotorSetpoint & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: duty_cycle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "duty_cycle: ";
    rosidl_generator_traits::value_to_yaml(msg.duty_cycle, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MotorSetpoint & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace create_msgs

namespace rosidl_generator_traits
{

[[deprecated("use create_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const create_msgs::msg::MotorSetpoint & msg,
  std::ostream & out, size_t indentation = 0)
{
  create_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use create_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const create_msgs::msg::MotorSetpoint & msg)
{
  return create_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<create_msgs::msg::MotorSetpoint>()
{
  return "create_msgs::msg::MotorSetpoint";
}

template<>
inline const char * name<create_msgs::msg::MotorSetpoint>()
{
  return "create_msgs/msg/MotorSetpoint";
}

template<>
struct has_fixed_size<create_msgs::msg::MotorSetpoint>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<create_msgs::msg::MotorSetpoint>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<create_msgs::msg::MotorSetpoint>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CREATE_MSGS__MSG__DETAIL__MOTOR_SETPOINT__TRAITS_HPP_
