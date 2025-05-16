// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from create_msgs:msg/Mode.idl
// generated code does not contain a copyright notice
#include "create_msgs/msg/detail/mode__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
create_msgs__msg__Mode__init(create_msgs__msg__Mode * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    create_msgs__msg__Mode__fini(msg);
    return false;
  }
  // mode
  return true;
}

void
create_msgs__msg__Mode__fini(create_msgs__msg__Mode * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // mode
}

bool
create_msgs__msg__Mode__are_equal(const create_msgs__msg__Mode * lhs, const create_msgs__msg__Mode * rhs)
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
  // mode
  if (lhs->mode != rhs->mode) {
    return false;
  }
  return true;
}

bool
create_msgs__msg__Mode__copy(
  const create_msgs__msg__Mode * input,
  create_msgs__msg__Mode * output)
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
  // mode
  output->mode = input->mode;
  return true;
}

create_msgs__msg__Mode *
create_msgs__msg__Mode__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  create_msgs__msg__Mode * msg = (create_msgs__msg__Mode *)allocator.allocate(sizeof(create_msgs__msg__Mode), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(create_msgs__msg__Mode));
  bool success = create_msgs__msg__Mode__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
create_msgs__msg__Mode__destroy(create_msgs__msg__Mode * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    create_msgs__msg__Mode__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
create_msgs__msg__Mode__Sequence__init(create_msgs__msg__Mode__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  create_msgs__msg__Mode * data = NULL;

  if (size) {
    data = (create_msgs__msg__Mode *)allocator.zero_allocate(size, sizeof(create_msgs__msg__Mode), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = create_msgs__msg__Mode__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        create_msgs__msg__Mode__fini(&data[i - 1]);
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
create_msgs__msg__Mode__Sequence__fini(create_msgs__msg__Mode__Sequence * array)
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
      create_msgs__msg__Mode__fini(&array->data[i]);
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

create_msgs__msg__Mode__Sequence *
create_msgs__msg__Mode__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  create_msgs__msg__Mode__Sequence * array = (create_msgs__msg__Mode__Sequence *)allocator.allocate(sizeof(create_msgs__msg__Mode__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = create_msgs__msg__Mode__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
create_msgs__msg__Mode__Sequence__destroy(create_msgs__msg__Mode__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    create_msgs__msg__Mode__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
create_msgs__msg__Mode__Sequence__are_equal(const create_msgs__msg__Mode__Sequence * lhs, const create_msgs__msg__Mode__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!create_msgs__msg__Mode__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
create_msgs__msg__Mode__Sequence__copy(
  const create_msgs__msg__Mode__Sequence * input,
  create_msgs__msg__Mode__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(create_msgs__msg__Mode);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    create_msgs__msg__Mode * data =
      (create_msgs__msg__Mode *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!create_msgs__msg__Mode__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          create_msgs__msg__Mode__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!create_msgs__msg__Mode__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
