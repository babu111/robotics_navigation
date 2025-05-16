// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from create_msgs:msg/Cliff.idl
// generated code does not contain a copyright notice
#include "create_msgs/msg/detail/cliff__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
create_msgs__msg__Cliff__init(create_msgs__msg__Cliff * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    create_msgs__msg__Cliff__fini(msg);
    return false;
  }
  // is_cliff_left
  // is_cliff_front_left
  // is_cliff_right
  // is_cliff_front_right
  return true;
}

void
create_msgs__msg__Cliff__fini(create_msgs__msg__Cliff * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // is_cliff_left
  // is_cliff_front_left
  // is_cliff_right
  // is_cliff_front_right
}

bool
create_msgs__msg__Cliff__are_equal(const create_msgs__msg__Cliff * lhs, const create_msgs__msg__Cliff * rhs)
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
  // is_cliff_left
  if (lhs->is_cliff_left != rhs->is_cliff_left) {
    return false;
  }
  // is_cliff_front_left
  if (lhs->is_cliff_front_left != rhs->is_cliff_front_left) {
    return false;
  }
  // is_cliff_right
  if (lhs->is_cliff_right != rhs->is_cliff_right) {
    return false;
  }
  // is_cliff_front_right
  if (lhs->is_cliff_front_right != rhs->is_cliff_front_right) {
    return false;
  }
  return true;
}

bool
create_msgs__msg__Cliff__copy(
  const create_msgs__msg__Cliff * input,
  create_msgs__msg__Cliff * output)
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
  // is_cliff_left
  output->is_cliff_left = input->is_cliff_left;
  // is_cliff_front_left
  output->is_cliff_front_left = input->is_cliff_front_left;
  // is_cliff_right
  output->is_cliff_right = input->is_cliff_right;
  // is_cliff_front_right
  output->is_cliff_front_right = input->is_cliff_front_right;
  return true;
}

create_msgs__msg__Cliff *
create_msgs__msg__Cliff__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  create_msgs__msg__Cliff * msg = (create_msgs__msg__Cliff *)allocator.allocate(sizeof(create_msgs__msg__Cliff), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(create_msgs__msg__Cliff));
  bool success = create_msgs__msg__Cliff__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
create_msgs__msg__Cliff__destroy(create_msgs__msg__Cliff * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    create_msgs__msg__Cliff__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
create_msgs__msg__Cliff__Sequence__init(create_msgs__msg__Cliff__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  create_msgs__msg__Cliff * data = NULL;

  if (size) {
    data = (create_msgs__msg__Cliff *)allocator.zero_allocate(size, sizeof(create_msgs__msg__Cliff), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = create_msgs__msg__Cliff__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        create_msgs__msg__Cliff__fini(&data[i - 1]);
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
create_msgs__msg__Cliff__Sequence__fini(create_msgs__msg__Cliff__Sequence * array)
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
      create_msgs__msg__Cliff__fini(&array->data[i]);
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

create_msgs__msg__Cliff__Sequence *
create_msgs__msg__Cliff__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  create_msgs__msg__Cliff__Sequence * array = (create_msgs__msg__Cliff__Sequence *)allocator.allocate(sizeof(create_msgs__msg__Cliff__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = create_msgs__msg__Cliff__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
create_msgs__msg__Cliff__Sequence__destroy(create_msgs__msg__Cliff__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    create_msgs__msg__Cliff__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
create_msgs__msg__Cliff__Sequence__are_equal(const create_msgs__msg__Cliff__Sequence * lhs, const create_msgs__msg__Cliff__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!create_msgs__msg__Cliff__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
create_msgs__msg__Cliff__Sequence__copy(
  const create_msgs__msg__Cliff__Sequence * input,
  create_msgs__msg__Cliff__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(create_msgs__msg__Cliff);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    create_msgs__msg__Cliff * data =
      (create_msgs__msg__Cliff *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!create_msgs__msg__Cliff__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          create_msgs__msg__Cliff__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!create_msgs__msg__Cliff__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
