"""autogenerated by genpy from ptam/move_robotRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class move_robotRequest(genpy.Message):
  _md5sum = "533042a64ee768c9dd424ef588119ef4"
  _type = "ptam/move_robotRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float64 start_x
float64 start_y
float64 start_theta
float64 goal_x
float64 goal_y
float64 goal_theta

"""
  __slots__ = ['start_x','start_y','start_theta','goal_x','goal_y','goal_theta']
  _slot_types = ['float64','float64','float64','float64','float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       start_x,start_y,start_theta,goal_x,goal_y,goal_theta

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(move_robotRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.start_x is None:
        self.start_x = 0.
      if self.start_y is None:
        self.start_y = 0.
      if self.start_theta is None:
        self.start_theta = 0.
      if self.goal_x is None:
        self.goal_x = 0.
      if self.goal_y is None:
        self.goal_y = 0.
      if self.goal_theta is None:
        self.goal_theta = 0.
    else:
      self.start_x = 0.
      self.start_y = 0.
      self.start_theta = 0.
      self.goal_x = 0.
      self.goal_y = 0.
      self.goal_theta = 0.

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
      buff.write(_struct_6d.pack(_x.start_x, _x.start_y, _x.start_theta, _x.goal_x, _x.goal_y, _x.goal_theta))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 48
      (_x.start_x, _x.start_y, _x.start_theta, _x.goal_x, _x.goal_y, _x.goal_theta,) = _struct_6d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_6d.pack(_x.start_x, _x.start_y, _x.start_theta, _x.goal_x, _x.goal_y, _x.goal_theta))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 48
      (_x.start_x, _x.start_y, _x.start_theta, _x.goal_x, _x.goal_y, _x.goal_theta,) = _struct_6d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_6d = struct.Struct("<6d")
"""autogenerated by genpy from ptam/move_robotResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class move_robotResponse(genpy.Message):
  _md5sum = "d41d8cd98f00b204e9800998ecf8427e"
  _type = "ptam/move_robotResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """

"""
  __slots__ = []
  _slot_types = []

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(move_robotResponse, self).__init__(*args, **kwds)

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
      pass
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      pass
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
class move_robot(object):
  _type          = 'ptam/move_robot'
  _md5sum = '533042a64ee768c9dd424ef588119ef4'
  _request_class  = move_robotRequest
  _response_class = move_robotResponse
