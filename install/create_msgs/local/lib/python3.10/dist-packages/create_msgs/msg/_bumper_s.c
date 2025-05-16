// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from create_msgs:msg/Bumper.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "create_msgs/msg/detail/bumper__struct.h"
#include "create_msgs/msg/detail/bumper__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool create_msgs__msg__bumper__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[31];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("create_msgs.msg._bumper.Bumper", full_classname_dest, 30) == 0);
  }
  create_msgs__msg__Bumper * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // is_left_pressed
    PyObject * field = PyObject_GetAttrString(_pymsg, "is_left_pressed");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->is_left_pressed = (Py_True == field);
    Py_DECREF(field);
  }
  {  // is_right_pressed
    PyObject * field = PyObject_GetAttrString(_pymsg, "is_right_pressed");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->is_right_pressed = (Py_True == field);
    Py_DECREF(field);
  }
  {  // is_light_left
    PyObject * field = PyObject_GetAttrString(_pymsg, "is_light_left");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->is_light_left = (Py_True == field);
    Py_DECREF(field);
  }
  {  // is_light_front_left
    PyObject * field = PyObject_GetAttrString(_pymsg, "is_light_front_left");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->is_light_front_left = (Py_True == field);
    Py_DECREF(field);
  }
  {  // is_light_center_left
    PyObject * field = PyObject_GetAttrString(_pymsg, "is_light_center_left");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->is_light_center_left = (Py_True == field);
    Py_DECREF(field);
  }
  {  // is_light_center_right
    PyObject * field = PyObject_GetAttrString(_pymsg, "is_light_center_right");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->is_light_center_right = (Py_True == field);
    Py_DECREF(field);
  }
  {  // is_light_front_right
    PyObject * field = PyObject_GetAttrString(_pymsg, "is_light_front_right");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->is_light_front_right = (Py_True == field);
    Py_DECREF(field);
  }
  {  // is_light_right
    PyObject * field = PyObject_GetAttrString(_pymsg, "is_light_right");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->is_light_right = (Py_True == field);
    Py_DECREF(field);
  }
  {  // light_signal_left
    PyObject * field = PyObject_GetAttrString(_pymsg, "light_signal_left");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->light_signal_left = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // light_signal_front_left
    PyObject * field = PyObject_GetAttrString(_pymsg, "light_signal_front_left");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->light_signal_front_left = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // light_signal_center_left
    PyObject * field = PyObject_GetAttrString(_pymsg, "light_signal_center_left");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->light_signal_center_left = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // light_signal_center_right
    PyObject * field = PyObject_GetAttrString(_pymsg, "light_signal_center_right");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->light_signal_center_right = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // light_signal_front_right
    PyObject * field = PyObject_GetAttrString(_pymsg, "light_signal_front_right");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->light_signal_front_right = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // light_signal_right
    PyObject * field = PyObject_GetAttrString(_pymsg, "light_signal_right");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->light_signal_right = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * create_msgs__msg__bumper__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Bumper */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("create_msgs.msg._bumper");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Bumper");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  create_msgs__msg__Bumper * ros_message = (create_msgs__msg__Bumper *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // is_left_pressed
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->is_left_pressed ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "is_left_pressed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // is_right_pressed
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->is_right_pressed ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "is_right_pressed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // is_light_left
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->is_light_left ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "is_light_left", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // is_light_front_left
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->is_light_front_left ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "is_light_front_left", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // is_light_center_left
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->is_light_center_left ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "is_light_center_left", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // is_light_center_right
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->is_light_center_right ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "is_light_center_right", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // is_light_front_right
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->is_light_front_right ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "is_light_front_right", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // is_light_right
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->is_light_right ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "is_light_right", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // light_signal_left
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->light_signal_left);
    {
      int rc = PyObject_SetAttrString(_pymessage, "light_signal_left", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // light_signal_front_left
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->light_signal_front_left);
    {
      int rc = PyObject_SetAttrString(_pymessage, "light_signal_front_left", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // light_signal_center_left
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->light_signal_center_left);
    {
      int rc = PyObject_SetAttrString(_pymessage, "light_signal_center_left", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // light_signal_center_right
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->light_signal_center_right);
    {
      int rc = PyObject_SetAttrString(_pymessage, "light_signal_center_right", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // light_signal_front_right
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->light_signal_front_right);
    {
      int rc = PyObject_SetAttrString(_pymessage, "light_signal_front_right", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // light_signal_right
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->light_signal_right);
    {
      int rc = PyObject_SetAttrString(_pymessage, "light_signal_right", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
