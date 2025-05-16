// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from create_msgs:msg/DefineSong.idl
// generated code does not contain a copyright notice

#ifndef CREATE_MSGS__MSG__DETAIL__DEFINE_SONG__TRAITS_HPP_
#define CREATE_MSGS__MSG__DETAIL__DEFINE_SONG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "create_msgs/msg/detail/define_song__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace create_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const DefineSong & msg,
  std::ostream & out)
{
  out << "{";
  // member: song
  {
    out << "song: ";
    rosidl_generator_traits::value_to_yaml(msg.song, out);
    out << ", ";
  }

  // member: length
  {
    out << "length: ";
    rosidl_generator_traits::value_to_yaml(msg.length, out);
    out << ", ";
  }

  // member: notes
  {
    if (msg.notes.size() == 0) {
      out << "notes: []";
    } else {
      out << "notes: [";
      size_t pending_items = msg.notes.size();
      for (auto item : msg.notes) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: durations
  {
    if (msg.durations.size() == 0) {
      out << "durations: []";
    } else {
      out << "durations: [";
      size_t pending_items = msg.durations.size();
      for (auto item : msg.durations) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DefineSong & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: song
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "song: ";
    rosidl_generator_traits::value_to_yaml(msg.song, out);
    out << "\n";
  }

  // member: length
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "length: ";
    rosidl_generator_traits::value_to_yaml(msg.length, out);
    out << "\n";
  }

  // member: notes
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.notes.size() == 0) {
      out << "notes: []\n";
    } else {
      out << "notes:\n";
      for (auto item : msg.notes) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: durations
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.durations.size() == 0) {
      out << "durations: []\n";
    } else {
      out << "durations:\n";
      for (auto item : msg.durations) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DefineSong & msg, bool use_flow_style = false)
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
  const create_msgs::msg::DefineSong & msg,
  std::ostream & out, size_t indentation = 0)
{
  create_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use create_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const create_msgs::msg::DefineSong & msg)
{
  return create_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<create_msgs::msg::DefineSong>()
{
  return "create_msgs::msg::DefineSong";
}

template<>
inline const char * name<create_msgs::msg::DefineSong>()
{
  return "create_msgs/msg/DefineSong";
}

template<>
struct has_fixed_size<create_msgs::msg::DefineSong>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<create_msgs::msg::DefineSong>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<create_msgs::msg::DefineSong>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CREATE_MSGS__MSG__DETAIL__DEFINE_SONG__TRAITS_HPP_
