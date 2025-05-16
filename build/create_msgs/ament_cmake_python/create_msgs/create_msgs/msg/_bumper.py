# generated from rosidl_generator_py/resource/_idl.py.em
# with input from create_msgs:msg/Bumper.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Bumper(type):
    """Metaclass of message 'Bumper'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('create_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'create_msgs.msg.Bumper')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__bumper
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__bumper
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__bumper
            cls._TYPE_SUPPORT = module.type_support_msg__msg__bumper
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__bumper

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Bumper(metaclass=Metaclass_Bumper):
    """Message class 'Bumper'."""

    __slots__ = [
        '_header',
        '_is_left_pressed',
        '_is_right_pressed',
        '_is_light_left',
        '_is_light_front_left',
        '_is_light_center_left',
        '_is_light_center_right',
        '_is_light_front_right',
        '_is_light_right',
        '_light_signal_left',
        '_light_signal_front_left',
        '_light_signal_center_left',
        '_light_signal_center_right',
        '_light_signal_front_right',
        '_light_signal_right',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'is_left_pressed': 'boolean',
        'is_right_pressed': 'boolean',
        'is_light_left': 'boolean',
        'is_light_front_left': 'boolean',
        'is_light_center_left': 'boolean',
        'is_light_center_right': 'boolean',
        'is_light_front_right': 'boolean',
        'is_light_right': 'boolean',
        'light_signal_left': 'uint16',
        'light_signal_front_left': 'uint16',
        'light_signal_center_left': 'uint16',
        'light_signal_center_right': 'uint16',
        'light_signal_front_right': 'uint16',
        'light_signal_right': 'uint16',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.is_left_pressed = kwargs.get('is_left_pressed', bool())
        self.is_right_pressed = kwargs.get('is_right_pressed', bool())
        self.is_light_left = kwargs.get('is_light_left', bool())
        self.is_light_front_left = kwargs.get('is_light_front_left', bool())
        self.is_light_center_left = kwargs.get('is_light_center_left', bool())
        self.is_light_center_right = kwargs.get('is_light_center_right', bool())
        self.is_light_front_right = kwargs.get('is_light_front_right', bool())
        self.is_light_right = kwargs.get('is_light_right', bool())
        self.light_signal_left = kwargs.get('light_signal_left', int())
        self.light_signal_front_left = kwargs.get('light_signal_front_left', int())
        self.light_signal_center_left = kwargs.get('light_signal_center_left', int())
        self.light_signal_center_right = kwargs.get('light_signal_center_right', int())
        self.light_signal_front_right = kwargs.get('light_signal_front_right', int())
        self.light_signal_right = kwargs.get('light_signal_right', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.is_left_pressed != other.is_left_pressed:
            return False
        if self.is_right_pressed != other.is_right_pressed:
            return False
        if self.is_light_left != other.is_light_left:
            return False
        if self.is_light_front_left != other.is_light_front_left:
            return False
        if self.is_light_center_left != other.is_light_center_left:
            return False
        if self.is_light_center_right != other.is_light_center_right:
            return False
        if self.is_light_front_right != other.is_light_front_right:
            return False
        if self.is_light_right != other.is_light_right:
            return False
        if self.light_signal_left != other.light_signal_left:
            return False
        if self.light_signal_front_left != other.light_signal_front_left:
            return False
        if self.light_signal_center_left != other.light_signal_center_left:
            return False
        if self.light_signal_center_right != other.light_signal_center_right:
            return False
        if self.light_signal_front_right != other.light_signal_front_right:
            return False
        if self.light_signal_right != other.light_signal_right:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def is_left_pressed(self):
        """Message field 'is_left_pressed'."""
        return self._is_left_pressed

    @is_left_pressed.setter
    def is_left_pressed(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_left_pressed' field must be of type 'bool'"
        self._is_left_pressed = value

    @builtins.property
    def is_right_pressed(self):
        """Message field 'is_right_pressed'."""
        return self._is_right_pressed

    @is_right_pressed.setter
    def is_right_pressed(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_right_pressed' field must be of type 'bool'"
        self._is_right_pressed = value

    @builtins.property
    def is_light_left(self):
        """Message field 'is_light_left'."""
        return self._is_light_left

    @is_light_left.setter
    def is_light_left(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_light_left' field must be of type 'bool'"
        self._is_light_left = value

    @builtins.property
    def is_light_front_left(self):
        """Message field 'is_light_front_left'."""
        return self._is_light_front_left

    @is_light_front_left.setter
    def is_light_front_left(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_light_front_left' field must be of type 'bool'"
        self._is_light_front_left = value

    @builtins.property
    def is_light_center_left(self):
        """Message field 'is_light_center_left'."""
        return self._is_light_center_left

    @is_light_center_left.setter
    def is_light_center_left(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_light_center_left' field must be of type 'bool'"
        self._is_light_center_left = value

    @builtins.property
    def is_light_center_right(self):
        """Message field 'is_light_center_right'."""
        return self._is_light_center_right

    @is_light_center_right.setter
    def is_light_center_right(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_light_center_right' field must be of type 'bool'"
        self._is_light_center_right = value

    @builtins.property
    def is_light_front_right(self):
        """Message field 'is_light_front_right'."""
        return self._is_light_front_right

    @is_light_front_right.setter
    def is_light_front_right(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_light_front_right' field must be of type 'bool'"
        self._is_light_front_right = value

    @builtins.property
    def is_light_right(self):
        """Message field 'is_light_right'."""
        return self._is_light_right

    @is_light_right.setter
    def is_light_right(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_light_right' field must be of type 'bool'"
        self._is_light_right = value

    @builtins.property
    def light_signal_left(self):
        """Message field 'light_signal_left'."""
        return self._light_signal_left

    @light_signal_left.setter
    def light_signal_left(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'light_signal_left' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'light_signal_left' field must be an unsigned integer in [0, 65535]"
        self._light_signal_left = value

    @builtins.property
    def light_signal_front_left(self):
        """Message field 'light_signal_front_left'."""
        return self._light_signal_front_left

    @light_signal_front_left.setter
    def light_signal_front_left(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'light_signal_front_left' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'light_signal_front_left' field must be an unsigned integer in [0, 65535]"
        self._light_signal_front_left = value

    @builtins.property
    def light_signal_center_left(self):
        """Message field 'light_signal_center_left'."""
        return self._light_signal_center_left

    @light_signal_center_left.setter
    def light_signal_center_left(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'light_signal_center_left' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'light_signal_center_left' field must be an unsigned integer in [0, 65535]"
        self._light_signal_center_left = value

    @builtins.property
    def light_signal_center_right(self):
        """Message field 'light_signal_center_right'."""
        return self._light_signal_center_right

    @light_signal_center_right.setter
    def light_signal_center_right(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'light_signal_center_right' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'light_signal_center_right' field must be an unsigned integer in [0, 65535]"
        self._light_signal_center_right = value

    @builtins.property
    def light_signal_front_right(self):
        """Message field 'light_signal_front_right'."""
        return self._light_signal_front_right

    @light_signal_front_right.setter
    def light_signal_front_right(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'light_signal_front_right' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'light_signal_front_right' field must be an unsigned integer in [0, 65535]"
        self._light_signal_front_right = value

    @builtins.property
    def light_signal_right(self):
        """Message field 'light_signal_right'."""
        return self._light_signal_right

    @light_signal_right.setter
    def light_signal_right(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'light_signal_right' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'light_signal_right' field must be an unsigned integer in [0, 65535]"
        self._light_signal_right = value
