# generated from rosidl_generator_py/resource/_idl.py.em
# with input from turtlebot3_applications_msgs:srv/TakePanorama.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TakePanorama_Request(type):
    """Metaclass of message 'TakePanorama_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'SNAPANDROTATE': 0,
        'CONTINUOUS': 1,
        'STOP': 2,
        'STARTED': 0,
        'IN_PROGRESS': 1,
        'STOPPED': 2,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('turtlebot3_applications_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'turtlebot3_applications_msgs.srv.TakePanorama_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__take_panorama__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__take_panorama__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__take_panorama__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__take_panorama__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__take_panorama__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'SNAPANDROTATE': cls.__constants['SNAPANDROTATE'],
            'CONTINUOUS': cls.__constants['CONTINUOUS'],
            'STOP': cls.__constants['STOP'],
            'STARTED': cls.__constants['STARTED'],
            'IN_PROGRESS': cls.__constants['IN_PROGRESS'],
            'STOPPED': cls.__constants['STOPPED'],
        }

    @property
    def SNAPANDROTATE(self):
        """Message constant 'SNAPANDROTATE'."""
        return Metaclass_TakePanorama_Request.__constants['SNAPANDROTATE']

    @property
    def CONTINUOUS(self):
        """Message constant 'CONTINUOUS'."""
        return Metaclass_TakePanorama_Request.__constants['CONTINUOUS']

    @property
    def STOP(self):
        """Message constant 'STOP'."""
        return Metaclass_TakePanorama_Request.__constants['STOP']

    @property
    def STARTED(self):
        """Message constant 'STARTED'."""
        return Metaclass_TakePanorama_Request.__constants['STARTED']

    @property
    def IN_PROGRESS(self):
        """Message constant 'IN_PROGRESS'."""
        return Metaclass_TakePanorama_Request.__constants['IN_PROGRESS']

    @property
    def STOPPED(self):
        """Message constant 'STOPPED'."""
        return Metaclass_TakePanorama_Request.__constants['STOPPED']


class TakePanorama_Request(metaclass=Metaclass_TakePanorama_Request):
    """
    Message class 'TakePanorama_Request'.

    Constants:
      SNAPANDROTATE
      CONTINUOUS
      STOP
      STARTED
      IN_PROGRESS
      STOPPED
    """

    __slots__ = [
        '_mode',
        '_pano_angle',
        '_snap_interval',
        '_rot_vel',
    ]

    _fields_and_field_types = {
        'mode': 'uint8',
        'pano_angle': 'float',
        'snap_interval': 'float',
        'rot_vel': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.mode = kwargs.get('mode', int())
        self.pano_angle = kwargs.get('pano_angle', float())
        self.snap_interval = kwargs.get('snap_interval', float())
        self.rot_vel = kwargs.get('rot_vel', float())

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
        if self.mode != other.mode:
            return False
        if self.pano_angle != other.pano_angle:
            return False
        if self.snap_interval != other.snap_interval:
            return False
        if self.rot_vel != other.rot_vel:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def mode(self):
        """Message field 'mode'."""
        return self._mode

    @mode.setter
    def mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'mode' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'mode' field must be an unsigned integer in [0, 255]"
        self._mode = value

    @builtins.property
    def pano_angle(self):
        """Message field 'pano_angle'."""
        return self._pano_angle

    @pano_angle.setter
    def pano_angle(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pano_angle' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'pano_angle' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._pano_angle = value

    @builtins.property
    def snap_interval(self):
        """Message field 'snap_interval'."""
        return self._snap_interval

    @snap_interval.setter
    def snap_interval(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'snap_interval' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'snap_interval' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._snap_interval = value

    @builtins.property
    def rot_vel(self):
        """Message field 'rot_vel'."""
        return self._rot_vel

    @rot_vel.setter
    def rot_vel(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'rot_vel' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'rot_vel' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._rot_vel = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_TakePanorama_Response(type):
    """Metaclass of message 'TakePanorama_Response'."""

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
            module = import_type_support('turtlebot3_applications_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'turtlebot3_applications_msgs.srv.TakePanorama_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__take_panorama__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__take_panorama__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__take_panorama__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__take_panorama__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__take_panorama__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class TakePanorama_Response(metaclass=Metaclass_TakePanorama_Response):
    """Message class 'TakePanorama_Response'."""

    __slots__ = [
        '_status',
    ]

    _fields_and_field_types = {
        'status': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.status = kwargs.get('status', int())

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
        if self.status != other.status:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def status(self):
        """Message field 'status'."""
        return self._status

    @status.setter
    def status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'status' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'status' field must be an unsigned integer in [0, 255]"
        self._status = value


class Metaclass_TakePanorama(type):
    """Metaclass of service 'TakePanorama'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('turtlebot3_applications_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'turtlebot3_applications_msgs.srv.TakePanorama')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__take_panorama

            from turtlebot3_applications_msgs.srv import _take_panorama
            if _take_panorama.Metaclass_TakePanorama_Request._TYPE_SUPPORT is None:
                _take_panorama.Metaclass_TakePanorama_Request.__import_type_support__()
            if _take_panorama.Metaclass_TakePanorama_Response._TYPE_SUPPORT is None:
                _take_panorama.Metaclass_TakePanorama_Response.__import_type_support__()


class TakePanorama(metaclass=Metaclass_TakePanorama):
    from turtlebot3_applications_msgs.srv._take_panorama import TakePanorama_Request as Request
    from turtlebot3_applications_msgs.srv._take_panorama import TakePanorama_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
