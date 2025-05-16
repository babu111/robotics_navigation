// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from create_msgs:msg/Bumper.idl
// generated code does not contain a copyright notice
#include "create_msgs/msg/detail/bumper__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
create_msgs__msg__Bumper__init(create_msgs__msg__Bumper * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    create_msgs__msg__Bumper__fini(msg);
    return false;
  }
  // is_left_pressed
  // is_right_pressed
  // is_light_left
  // is_light_front_left
  // is_light_center_left
  // is_light_center_right
  // is_light_front_right
  // is_light_right
  // light_signal_left
  // light_signal_front_left
  // light_signal_center_left
  // light_signal_center_right
  // light_signal_front_right
  // light_signal_right
  return true;
}

void
create_msgs__msg__Bumper__fini(create_msgs__msg__Bumper * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // is_left_pressed
  // is_right_pressed
  // is_light_left
  // is_light_front_left
  // is_light_center_left
  // is_light_center_right
  // is_light_front_right
  // is_light_right
  // light_signal_left
  // light_signal_front_left
  // light_signal_center_left
  // light_signal_center_right
  // light_signal_front_right
  // light_signal_right
}

bool
create_msgs__msg__Bumper__are_equal(const create_msgs__msg__Bumper * lhs, const create_msgs__msg__Bumper * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // is_left_pressed
  if (lhs->is_left_pressed != rhs->is_left_pressed) {
    return false;
  }
  // is_right_pressed
  if (lhs->is_right_pressed != rhs->is_right_pressed) {
    return false;
  }
  // is_light_left
  if (lhs->is_light_left != rhs->is_light_left) {
    return false;
  }
  // is_light_front_left
  if (lhs->is_light_front_left != rhs->is_light_front_left) {
    return false;
  }
  // is_light_center_left
  if (lhs->is_light_center_left != rhs->is_light_center_left) {
    return false;
  }
  // is_light_center_right
  if (lhs->is_light_center_right != rhs->is_light_center_right) {
    return false;
  }
  // is_light_front_right
  if (lhs->is_light_front_right != rhs->is_light_front_right) {
    return false;
  }
  // is_light_right
  if (lhs->is_light_right != rhs->is_light_right) {
    return false;
  }
  // light_signal_left
  if (lhs->light_signal_left != rhs->light_signal_left) {
    return false;
  }
  // light_signal_front_left
  if (lhs->light_signal_front_left != rhs->light_signal_front_left) {
    return false;
  }
  // light_signal_center_left
  if (lhs->light_signal_center_left != rhs->light_signal_center_left) {
    return false;
  }
  // light_signal_center_right
  if (lhs->light_signal_center_right != rhs->light_signal_center_right) {
    return false;
  }
  // light_signal_front_right
  if (lhs->light_signal_front_right != rhs->light_signal_front_right) {
    return false;
  }
  // light_signal_right
  if (lhs->light_signal_right != rhs->light_signal_right) {
    return false;
  }
  return true;
}

bool
create_msgs__msg__Bumper__copy(
  const create_msgs__msg__Bumper * input,
  create_msgs__msg__Bumper * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // is_left_pressed
  output->is_left_pressed = input->is_left_pressed;
  // is_right_pressed
  output->is_right_pressed = input->is_right_pressed;
  // is_light_left
  output->is_light_left = input->is_light_left;
  // is_light_front_left
  output->is_light_front_left = input->is_light_front_left;
  // is_light_center_left
  output->is_light_center_left = input->is_light_center_left;
  // is_light_center_right
  output->is_light_center_right = input->is_light_center_right;
  // is_light_front_right
  output->is_light_front_right = input->is_light_front_right;
  // is_light_right
  output->is_light_right = input->is_light_right;
  // light_signal_left
  output->light_signal_left = input->light_signal_left;
  // light_signal_front_left
  output->light_signal_front_left = input->light_signal_front_left;
  // light_signal_center_left
  output->light_signal_center_left = input->light_signal_center_left;
  // light_signal_center_right
  output->light_signal_center_right = input->light_signal_center_right;
  // light_signal_front_right
  output->light_signal_front_right = input->light_signal_front_right;
  // light_signal_right
  output->light_signal_right = input->light_signal_right;
  return true;
}

create_msgs__msg__Bumper *
create_msgs__msg__Bumper__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  create_msgs__msg__Bumper * msg = (create_msgs__msg__Bumper *)allocator.allocate(sizeof(create_msgs__msg__Bumper), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(create_msgs__msg__Bumper));
  bool success = create_msgs__msg__Bumper__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
create_msgs__msg__Bumper__destroy(create_msgs__msg__Bumper * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    create_msgs__msg__Bumper__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
create_msgs__msg__Bumper__Sequence__init(create_msgs__msg__Bumper__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  create_msgs__msg__Bumper * data = NULL;

  if (size) {
    data = (create_msgs__msg__Bumper *)allocator.zero_allocate(size, sizeof(create_msgs__msg__Bumper), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = create_msgs__msg__Bumper__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        create_msgs__msg__Bumper__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
create_msgs__msg__Bumper__Sequence__fini(create_msgs__msg__Bumper__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      create_msgs__msg__Bumper__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

create_msgs__msg__Bumper__Sequence *
create_msgs__msg__Bumper__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  create_msgs__msg__Bumper__Sequence * array = (create_msgs__msg__Bumper__Sequence *)allocator.allocate(sizeof(create_msgs__msg__Bumper__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = create_msgs__msg__Bumper__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
create_msgs__msg__Bumper__Sequence__destroy(create_msgs__msg__Bumper__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    create_msgs__msg__Bumper__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
create_msgs__msg__Bumper__Sequence__are_equal(const create_msgs__msg__Bumper__Sequence * lhs, const create_msgs__msg__Bumper__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!create_msgs__msg__Bumper__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
create_msgs__msg__Bumper__Sequence__copy(
  const create_msgs__msg__Bumper__Sequence * input,
  create_msgs__msg__Bumper__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(create_msgs__msg__Bumper);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    create_msgs__msg__Bumper * data =
      (create_msgs__msg__Bumper *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!create_msgs__msg__Bumper__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          create_msgs__msg__Bumper__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!create_msgs__msg__Bumper__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
