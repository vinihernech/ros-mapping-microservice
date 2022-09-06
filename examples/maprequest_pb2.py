# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: maprequest.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from is_msgs import common_pb2 as is__msgs_dot_common__pb2
from google.protobuf import timestamp_pb2 as google_dot_protobuf_dot_timestamp__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='maprequest.proto',
  package='is.robot',
  syntax='proto3',
  serialized_options=_b('\n\014com.is.robotP\001'),
  serialized_pb=_b('\n\x10maprequest.proto\x12\x08is.robot\x1a\x14is_msgs/common.proto\x1a\x1fgoogle/protobuf/timestamp.proto\"8\n\nMapRequest\x12\n\n\x02id\x18\x01 \x01(\x04\x12\x1e\n\x05poses\x18\x02 \x03(\x0b\x32\x0f.is.common.Pose\"\x82\x01\n\x0fMapRequestReply\x12\n\n\x02id\x18\x01 \x01(\x04\x12\x16\n\x0emap_resolution\x18\x02 \x01(\x02\x12\r\n\x05width\x18\x03 \x01(\r\x12\x0e\n\x06height\x18\x04 \x01(\r\x12\x1f\n\x06origin\x18\x05 \x01(\x0b\x32\x0f.is.common.Pose\x12\x0b\n\x03map\x18\x06 \x03(\x03\x42\x10\n\x0c\x63om.is.robotP\x01\x62\x06proto3')
  ,
  dependencies=[is__msgs_dot_common__pb2.DESCRIPTOR,google_dot_protobuf_dot_timestamp__pb2.DESCRIPTOR,])




_MAPREQUEST = _descriptor.Descriptor(
  name='MapRequest',
  full_name='is.robot.MapRequest',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='is.robot.MapRequest.id', index=0,
      number=1, type=4, cpp_type=4, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='poses', full_name='is.robot.MapRequest.poses', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=85,
  serialized_end=141,
)


_MAPREQUESTREPLY = _descriptor.Descriptor(
  name='MapRequestReply',
  full_name='is.robot.MapRequestReply',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='is.robot.MapRequestReply.id', index=0,
      number=1, type=4, cpp_type=4, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='map_resolution', full_name='is.robot.MapRequestReply.map_resolution', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='width', full_name='is.robot.MapRequestReply.width', index=2,
      number=3, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='height', full_name='is.robot.MapRequestReply.height', index=3,
      number=4, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='origin', full_name='is.robot.MapRequestReply.origin', index=4,
      number=5, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='map', full_name='is.robot.MapRequestReply.map', index=5,
      number=6, type=3, cpp_type=2, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=144,
  serialized_end=274,
)

_MAPREQUEST.fields_by_name['poses'].message_type = is__msgs_dot_common__pb2._POSE
_MAPREQUESTREPLY.fields_by_name['origin'].message_type = is__msgs_dot_common__pb2._POSE
DESCRIPTOR.message_types_by_name['MapRequest'] = _MAPREQUEST
DESCRIPTOR.message_types_by_name['MapRequestReply'] = _MAPREQUESTREPLY
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

MapRequest = _reflection.GeneratedProtocolMessageType('MapRequest', (_message.Message,), dict(
  DESCRIPTOR = _MAPREQUEST,
  __module__ = 'maprequest_pb2'
  # @@protoc_insertion_point(class_scope:is.robot.MapRequest)
  ))
_sym_db.RegisterMessage(MapRequest)

MapRequestReply = _reflection.GeneratedProtocolMessageType('MapRequestReply', (_message.Message,), dict(
  DESCRIPTOR = _MAPREQUESTREPLY,
  __module__ = 'maprequest_pb2'
  # @@protoc_insertion_point(class_scope:is.robot.MapRequestReply)
  ))
_sym_db.RegisterMessage(MapRequestReply)


DESCRIPTOR._options = None
# @@protoc_insertion_point(module_scope)