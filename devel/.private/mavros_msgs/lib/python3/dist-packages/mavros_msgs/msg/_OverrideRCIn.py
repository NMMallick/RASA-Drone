# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from mavros_msgs/OverrideRCIn.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class OverrideRCIn(genpy.Message):
  _md5sum = "fd1e1c08fa504ec32737c41f45223398"
  _type = "mavros_msgs/OverrideRCIn"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """# Override RC Input
# Currently MAVLink defines override for 18 channels

# https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE

uint16 CHAN_RELEASE=0
uint16 CHAN_NOCHANGE=65535

uint16[18] channels
"""
  # Pseudo-constants
  CHAN_RELEASE = 0
  CHAN_NOCHANGE = 65535

  __slots__ = ['channels']
  _slot_types = ['uint16[18]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       channels

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(OverrideRCIn, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.channels is None:
        self.channels = [0] * 18
    else:
      self.channels = [0] * 18

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
      buff.write(_get_struct_18H().pack(*self.channels))
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
      start = end
      end += 36
      self.channels = _get_struct_18H().unpack(str[start:end])
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
      buff.write(self.channels.tostring())
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
      start = end
      end += 36
      self.channels = numpy.frombuffer(str[start:end], dtype=numpy.uint16, count=18)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_18H = None
def _get_struct_18H():
    global _struct_18H
    if _struct_18H is None:
        _struct_18H = struct.Struct("<18H")
    return _struct_18H
