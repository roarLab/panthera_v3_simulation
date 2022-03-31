# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from panthera_locomotion/ICRsearchRequest.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class ICRsearchRequest(genpy.Message):
  _md5sum = "9b0f9d7b5541dc5d4d54f360e0b450a0"
  _type = "panthera_locomotion/ICRsearchRequest"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """bool received_angle
float64 turn_angle
"""
  __slots__ = ['received_angle','turn_angle']
  _slot_types = ['bool','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       received_angle,turn_angle

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ICRsearchRequest, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.received_angle is None:
        self.received_angle = False
      if self.turn_angle is None:
        self.turn_angle = 0.
    else:
      self.received_angle = False
      self.turn_angle = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_Bd().pack(_x.received_angle, _x.turn_angle))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 9
      (_x.received_angle, _x.turn_angle,) = _get_struct_Bd().unpack(str[start:end])
      self.received_angle = bool(self.received_angle)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_Bd().pack(_x.received_angle, _x.turn_angle))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 9
      (_x.received_angle, _x.turn_angle,) = _get_struct_Bd().unpack(str[start:end])
      self.received_angle = bool(self.received_angle)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_Bd = None
def _get_struct_Bd():
    global _struct_Bd
    if _struct_Bd is None:
        _struct_Bd = struct.Struct("<Bd")
    return _struct_Bd
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from panthera_locomotion/ICRsearchResponse.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg

class ICRsearchResponse(genpy.Message):
  _md5sum = "b155cda63f3fb3bd2896ba1fe0ba2d6b"
  _type = "panthera_locomotion/ICRsearchResponse"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """bool feasibility
geometry_msgs/Twist wheel_angles
geometry_msgs/Twist wheel_speeds


================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z"""
  __slots__ = ['feasibility','wheel_angles','wheel_speeds']
  _slot_types = ['bool','geometry_msgs/Twist','geometry_msgs/Twist']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       feasibility,wheel_angles,wheel_speeds

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ICRsearchResponse, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.feasibility is None:
        self.feasibility = False
      if self.wheel_angles is None:
        self.wheel_angles = geometry_msgs.msg.Twist()
      if self.wheel_speeds is None:
        self.wheel_speeds = geometry_msgs.msg.Twist()
    else:
      self.feasibility = False
      self.wheel_angles = geometry_msgs.msg.Twist()
      self.wheel_speeds = geometry_msgs.msg.Twist()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_B12d().pack(_x.feasibility, _x.wheel_angles.linear.x, _x.wheel_angles.linear.y, _x.wheel_angles.linear.z, _x.wheel_angles.angular.x, _x.wheel_angles.angular.y, _x.wheel_angles.angular.z, _x.wheel_speeds.linear.x, _x.wheel_speeds.linear.y, _x.wheel_speeds.linear.z, _x.wheel_speeds.angular.x, _x.wheel_speeds.angular.y, _x.wheel_speeds.angular.z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.wheel_angles is None:
        self.wheel_angles = geometry_msgs.msg.Twist()
      if self.wheel_speeds is None:
        self.wheel_speeds = geometry_msgs.msg.Twist()
      end = 0
      _x = self
      start = end
      end += 97
      (_x.feasibility, _x.wheel_angles.linear.x, _x.wheel_angles.linear.y, _x.wheel_angles.linear.z, _x.wheel_angles.angular.x, _x.wheel_angles.angular.y, _x.wheel_angles.angular.z, _x.wheel_speeds.linear.x, _x.wheel_speeds.linear.y, _x.wheel_speeds.linear.z, _x.wheel_speeds.angular.x, _x.wheel_speeds.angular.y, _x.wheel_speeds.angular.z,) = _get_struct_B12d().unpack(str[start:end])
      self.feasibility = bool(self.feasibility)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_B12d().pack(_x.feasibility, _x.wheel_angles.linear.x, _x.wheel_angles.linear.y, _x.wheel_angles.linear.z, _x.wheel_angles.angular.x, _x.wheel_angles.angular.y, _x.wheel_angles.angular.z, _x.wheel_speeds.linear.x, _x.wheel_speeds.linear.y, _x.wheel_speeds.linear.z, _x.wheel_speeds.angular.x, _x.wheel_speeds.angular.y, _x.wheel_speeds.angular.z))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.wheel_angles is None:
        self.wheel_angles = geometry_msgs.msg.Twist()
      if self.wheel_speeds is None:
        self.wheel_speeds = geometry_msgs.msg.Twist()
      end = 0
      _x = self
      start = end
      end += 97
      (_x.feasibility, _x.wheel_angles.linear.x, _x.wheel_angles.linear.y, _x.wheel_angles.linear.z, _x.wheel_angles.angular.x, _x.wheel_angles.angular.y, _x.wheel_angles.angular.z, _x.wheel_speeds.linear.x, _x.wheel_speeds.linear.y, _x.wheel_speeds.linear.z, _x.wheel_speeds.angular.x, _x.wheel_speeds.angular.y, _x.wheel_speeds.angular.z,) = _get_struct_B12d().unpack(str[start:end])
      self.feasibility = bool(self.feasibility)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_B12d = None
def _get_struct_B12d():
    global _struct_B12d
    if _struct_B12d is None:
        _struct_B12d = struct.Struct("<B12d")
    return _struct_B12d
class ICRsearch(object):
  _type          = 'panthera_locomotion/ICRsearch'
  _md5sum = 'bcaf6636f1df5330595534eeebecd7f8'
  _request_class  = ICRsearchRequest
  _response_class = ICRsearchResponse
