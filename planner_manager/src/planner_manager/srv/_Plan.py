"""autogenerated by genpy from planner_manager/PlanRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class PlanRequest(genpy.Message):
  _md5sum = "89b81386720be1cd0ce7a3953fcd1b19"
  _type = "planner_manager/PlanRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """uint8 mode

"""
  __slots__ = ['mode']
  _slot_types = ['uint8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       mode

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(PlanRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.mode is None:
        self.mode = 0
    else:
      self.mode = 0

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
      buff.write(_struct_B.pack(self.mode))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 1
      (self.mode,) = _struct_B.unpack(str[start:end])
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
      buff.write(_struct_B.pack(self.mode))
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
      start = end
      end += 1
      (self.mode,) = _struct_B.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_B = struct.Struct("<B")
"""autogenerated by genpy from planner_manager/PlanResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import trajectory_msgs.msg
import genpy
import std_msgs.msg

class PlanResponse(genpy.Message):
  _md5sum = "044b8b6e7f41c18680e0bc1687bd081a"
  _type = "planner_manager/PlanResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """trajectory_msgs/JointTrajectory[] man_plan


================================================================================
MSG: trajectory_msgs/JointTrajectory
Header header
string[] joint_names
JointTrajectoryPoint[] points
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: trajectory_msgs/JointTrajectoryPoint
float64[] positions
float64[] velocities
float64[] accelerations
duration time_from_start
"""
  __slots__ = ['man_plan']
  _slot_types = ['trajectory_msgs/JointTrajectory[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       man_plan

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(PlanResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.man_plan is None:
        self.man_plan = []
    else:
      self.man_plan = []

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
      length = len(self.man_plan)
      buff.write(_struct_I.pack(length))
      for val1 in self.man_plan:
        _v1 = val1.header
        buff.write(_struct_I.pack(_v1.seq))
        _v2 = _v1.stamp
        _x = _v2
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v1.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        length = len(val1.joint_names)
        buff.write(_struct_I.pack(length))
        for val2 in val1.joint_names:
          length = len(val2)
          if python3 or type(val2) == unicode:
            val2 = val2.encode('utf-8')
            length = len(val2)
          buff.write(struct.pack('<I%ss'%length, length, val2))
        length = len(val1.points)
        buff.write(_struct_I.pack(length))
        for val2 in val1.points:
          length = len(val2.positions)
          buff.write(_struct_I.pack(length))
          pattern = '<%sd'%length
          buff.write(struct.pack(pattern, *val2.positions))
          length = len(val2.velocities)
          buff.write(_struct_I.pack(length))
          pattern = '<%sd'%length
          buff.write(struct.pack(pattern, *val2.velocities))
          length = len(val2.accelerations)
          buff.write(_struct_I.pack(length))
          pattern = '<%sd'%length
          buff.write(struct.pack(pattern, *val2.accelerations))
          _v3 = val2.time_from_start
          _x = _v3
          buff.write(_struct_2i.pack(_x.secs, _x.nsecs))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.man_plan is None:
        self.man_plan = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.man_plan = []
      for i in range(0, length):
        val1 = trajectory_msgs.msg.JointTrajectory()
        _v4 = val1.header
        start = end
        end += 4
        (_v4.seq,) = _struct_I.unpack(str[start:end])
        _v5 = _v4.stamp
        _x = _v5
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v4.frame_id = str[start:end].decode('utf-8')
        else:
          _v4.frame_id = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.joint_names = []
        for i in range(0, length):
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          start = end
          end += length
          if python3:
            val2 = str[start:end].decode('utf-8')
          else:
            val2 = str[start:end]
          val1.joint_names.append(val2)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.points = []
        for i in range(0, length):
          val2 = trajectory_msgs.msg.JointTrajectoryPoint()
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          pattern = '<%sd'%length
          start = end
          end += struct.calcsize(pattern)
          val2.positions = struct.unpack(pattern, str[start:end])
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          pattern = '<%sd'%length
          start = end
          end += struct.calcsize(pattern)
          val2.velocities = struct.unpack(pattern, str[start:end])
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          pattern = '<%sd'%length
          start = end
          end += struct.calcsize(pattern)
          val2.accelerations = struct.unpack(pattern, str[start:end])
          _v6 = val2.time_from_start
          _x = _v6
          start = end
          end += 8
          (_x.secs, _x.nsecs,) = _struct_2i.unpack(str[start:end])
          val1.points.append(val2)
        self.man_plan.append(val1)
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
      length = len(self.man_plan)
      buff.write(_struct_I.pack(length))
      for val1 in self.man_plan:
        _v7 = val1.header
        buff.write(_struct_I.pack(_v7.seq))
        _v8 = _v7.stamp
        _x = _v8
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v7.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        length = len(val1.joint_names)
        buff.write(_struct_I.pack(length))
        for val2 in val1.joint_names:
          length = len(val2)
          if python3 or type(val2) == unicode:
            val2 = val2.encode('utf-8')
            length = len(val2)
          buff.write(struct.pack('<I%ss'%length, length, val2))
        length = len(val1.points)
        buff.write(_struct_I.pack(length))
        for val2 in val1.points:
          length = len(val2.positions)
          buff.write(_struct_I.pack(length))
          pattern = '<%sd'%length
          buff.write(val2.positions.tostring())
          length = len(val2.velocities)
          buff.write(_struct_I.pack(length))
          pattern = '<%sd'%length
          buff.write(val2.velocities.tostring())
          length = len(val2.accelerations)
          buff.write(_struct_I.pack(length))
          pattern = '<%sd'%length
          buff.write(val2.accelerations.tostring())
          _v9 = val2.time_from_start
          _x = _v9
          buff.write(_struct_2i.pack(_x.secs, _x.nsecs))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.man_plan is None:
        self.man_plan = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.man_plan = []
      for i in range(0, length):
        val1 = trajectory_msgs.msg.JointTrajectory()
        _v10 = val1.header
        start = end
        end += 4
        (_v10.seq,) = _struct_I.unpack(str[start:end])
        _v11 = _v10.stamp
        _x = _v11
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v10.frame_id = str[start:end].decode('utf-8')
        else:
          _v10.frame_id = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.joint_names = []
        for i in range(0, length):
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          start = end
          end += length
          if python3:
            val2 = str[start:end].decode('utf-8')
          else:
            val2 = str[start:end]
          val1.joint_names.append(val2)
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.points = []
        for i in range(0, length):
          val2 = trajectory_msgs.msg.JointTrajectoryPoint()
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          pattern = '<%sd'%length
          start = end
          end += struct.calcsize(pattern)
          val2.positions = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          pattern = '<%sd'%length
          start = end
          end += struct.calcsize(pattern)
          val2.velocities = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          pattern = '<%sd'%length
          start = end
          end += struct.calcsize(pattern)
          val2.accelerations = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
          _v12 = val2.time_from_start
          _x = _v12
          start = end
          end += 8
          (_x.secs, _x.nsecs,) = _struct_2i.unpack(str[start:end])
          val1.points.append(val2)
        self.man_plan.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_2I = struct.Struct("<2I")
_struct_2i = struct.Struct("<2i")
class Plan(object):
  _type          = 'planner_manager/Plan'
  _md5sum = '08504a5995fbf0921e8fbc0f2cd5181a'
  _request_class  = PlanRequest
  _response_class = PlanResponse