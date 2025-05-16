// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from create_msgs:msg/DefineSong.idl
// generated code does not contain a copyright notice
#include "create_msgs/msg/detail/define_song__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `notes`
// Member `durations`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
create_msgs__msg__DefineSong__init(create_msgs__msg__DefineSong * msg)
{
  if (!msg) {
    return false;
  }
  // song
  // length
  // notes
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->notes, 0)) {
    create_msgs__msg__DefineSong__fini(msg);
    return false;
  }
  // durations
  if (!rosidl_runtime_c__float__Sequence__init(&msg->durations, 0)) {
    create_msgs__msg__DefineSong__fini(msg);
    return false;
  }
  return true;
}

void
create_msgs__msg__DefineSong__fini(create_msgs__msg__DefineSong * msg)
{
  if (!msg) {
    return;
  }
  // song
  // length
  // notes
  rosidl_runtime_c__uint8__Sequence__fini(&msg->notes);
  // durations
  rosidl_runtime_c__float__Sequence__fini(&msg->durations);
}

bool
create_msgs__msg__DefineSong__are_equal(const create_msgs__msg__DefineSong * lhs, const create_msgs__msg__DefineSong * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // song
  if (lhs->song != rhs->song) {
    return false;
  }
  // length
  if (lhs->length != rhs->length) {
    return false;
  }
  // notes
  if (!rosidl_runtime_c__uint8__Sequence__are_equal(
      &(lhs->notes), &(rhs->notes)))
  {
    return false;
  }
  // durations
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->durations), &(rhs->durations)))
  {
    return false;
  }
  return true;
}

bool
create_msgs__msg__DefineSong__copy(
  const create_msgs__msg__DefineSong * input,
  create_msgs__msg__DefineSong * output)
{
  if (!input || !output) {
    return false;
  }
  // song
  output->song = input->song;
  // length
  output->length = input->length;
  // notes
  if (!rosidl_runtime_c__uint8__Sequence__copy(
      &(input->notes), &(output->notes)))
  {
    return false;
  }
  // durations
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->durations), &(output->durations)))
  {
    return false;
  }
  return true;
}

create_msgs__msg__DefineSong *
create_msgs__msg__DefineSong__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  create_msgs__msg__DefineSong * msg = (create_msgs__msg__DefineSong *)allocator.allocate(sizeof(create_msgs__msg__DefineSong), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(create_msgs__msg__DefineSong));
  bool success = create_msgs__msg__DefineSong__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
create_msgs__msg__DefineSong__destroy(create_msgs__msg__DefineSong * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    create_msgs__msg__DefineSong__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
create_msgs__msg__DefineSong__Sequence__init(create_msgs__msg__DefineSong__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  create_msgs__msg__DefineSong * data = NULL;

  if (size) {
    data = (create_msgs__msg__DefineSong *)allocator.zero_allocate(size, sizeof(create_msgs__msg__DefineSong), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = create_msgs__msg__DefineSong__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        create_msgs__msg__DefineSong__fini(&data[i - 1]);
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
create_msgs__msg__DefineSong__Sequence__fini(create_msgs__msg__DefineSong__Sequence * array)
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
      create_msgs__msg__DefineSong__fini(&array->data[i]);
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

create_msgs__msg__DefineSong__Sequence *
create_msgs__msg__DefineSong__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  create_msgs__msg__DefineSong__Sequence * array = (create_msgs__msg__DefineSong__Sequence *)allocator.allocate(sizeof(create_msgs__msg__DefineSong__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = create_msgs__msg__DefineSong__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
create_msgs__msg__DefineSong__Sequence__destroy(create_msgs__msg__DefineSong__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    create_msgs__msg__DefineSong__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
create_msgs__msg__DefineSong__Sequence__are_equal(const create_msgs__msg__DefineSong__Sequence * lhs, const create_msgs__msg__DefineSong__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!create_msgs__msg__DefineSong__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
create_msgs__msg__DefineSong__Sequence__copy(
  const create_msgs__msg__DefineSong__Sequence * input,
  create_msgs__msg__DefineSong__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(create_msgs__msg__DefineSong);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    create_msgs__msg__DefineSong * data =
      (create_msgs__msg__DefineSong *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!create_msgs__msg__DefineSong__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          create_msgs__msg__DefineSong__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!create_msgs__msg__DefineSong__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
