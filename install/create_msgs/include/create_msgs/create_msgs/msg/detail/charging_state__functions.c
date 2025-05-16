// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from create_msgs:msg/ChargingState.idl
// generated code does not contain a copyright notice
#include "create_msgs/msg/detail/charging_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
create_msgs__msg__ChargingState__init(create_msgs__msg__ChargingState * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    create_msgs__msg__ChargingState__fini(msg);
    return false;
  }
  // state
  return true;
}

void
create_msgs__msg__ChargingState__fini(create_msgs__msg__ChargingState * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // state
}

bool
create_msgs__msg__ChargingState__are_equal(const create_msgs__msg__ChargingState * lhs, const create_msgs__msg__ChargingState * rhs)
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
  // state
  if (lhs->state != rhs->state) {
    return false;
  }
  return true;
}

bool
create_msgs__msg__ChargingState__copy(
  const create_msgs__msg__ChargingState * input,
  create_msgs__msg__ChargingState * output)
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
  // state
  output->state = input->state;
  return true;
}

create_msgs__msg__ChargingState *
create_msgs__msg__ChargingState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  create_msgs__msg__ChargingState * msg = (create_msgs__msg__ChargingState *)allocator.allocate(sizeof(create_msgs__msg__ChargingState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(create_msgs__msg__ChargingState));
  bool success = create_msgs__msg__ChargingState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
create_msgs__msg__ChargingState__destroy(create_msgs__msg__ChargingState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    create_msgs__msg__ChargingState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
create_msgs__msg__ChargingState__Sequence__init(create_msgs__msg__ChargingState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  create_msgs__msg__ChargingState * data = NULL;

  if (size) {
    data = (create_msgs__msg__ChargingState *)allocator.zero_allocate(size, sizeof(create_msgs__msg__ChargingState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = create_msgs__msg__ChargingState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        create_msgs__msg__ChargingState__fini(&data[i - 1]);
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
create_msgs__msg__ChargingState__Sequence__fini(create_msgs__msg__ChargingState__Sequence * array)
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
      create_msgs__msg__ChargingState__fini(&array->data[i]);
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

create_msgs__msg__ChargingState__Sequence *
create_msgs__msg__ChargingState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  create_msgs__msg__ChargingState__Sequence * array = (create_msgs__msg__ChargingState__Sequence *)allocator.allocate(sizeof(create_msgs__msg__ChargingState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = create_msgs__msg__ChargingState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
create_msgs__msg__ChargingState__Sequence__destroy(create_msgs__msg__ChargingState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    create_msgs__msg__ChargingState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
create_msgs__msg__ChargingState__Sequence__are_equal(const create_msgs__msg__ChargingState__Sequence * lhs, const create_msgs__msg__ChargingState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!create_msgs__msg__ChargingState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
create_msgs__msg__ChargingState__Sequence__copy(
  const create_msgs__msg__ChargingState__Sequence * input,
  create_msgs__msg__ChargingState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(create_msgs__msg__ChargingState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    create_msgs__msg__ChargingState * data =
      (create_msgs__msg__ChargingState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!create_msgs__msg__ChargingState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          create_msgs__msg__ChargingState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!create_msgs__msg__ChargingState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
