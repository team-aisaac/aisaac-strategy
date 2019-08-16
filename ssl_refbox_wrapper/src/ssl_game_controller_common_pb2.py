# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ssl_game_controller_common.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='ssl_game_controller_common.proto',
  package='',
  serialized_pb=_b('\n ssl_game_controller_common.proto\"(\n\x05\x42otId\x12\n\n\x02id\x18\x01 \x01(\x05\x12\x13\n\x04team\x18\x02 \x01(\x0e\x32\x05.Team\" \n\x08Location\x12\t\n\x01x\x18\x01 \x02(\x02\x12\t\n\x01y\x18\x02 \x02(\x02\"\xa1\x02\n\x0f\x43ontrollerReply\x12\x30\n\x0bstatus_code\x18\x01 \x01(\x0e\x32\x1b.ControllerReply.StatusCode\x12\x0e\n\x06reason\x18\x02 \x01(\t\x12\x12\n\nnext_token\x18\x03 \x01(\t\x12\x33\n\x0cverification\x18\x04 \x01(\x0e\x32\x1d.ControllerReply.Verification\";\n\nStatusCode\x12\x17\n\x13UNKNOWN_STATUS_CODE\x10\x00\x12\x06\n\x02OK\x10\x01\x12\x0c\n\x08REJECTED\x10\x02\"F\n\x0cVerification\x12\x18\n\x14UNKNOWN_VERIFICATION\x10\x00\x12\x0c\n\x08VERIFIED\x10\x01\x12\x0e\n\nUNVERIFIED\x10\x02\",\n\tSignature\x12\r\n\x05token\x18\x01 \x02(\t\x12\x10\n\x08pkcs1v15\x18\x02 \x02(\x0c\"Y\n\x14\x42\x61llSpeedMeasurement\x12\x11\n\ttimestamp\x18\x01 \x02(\x04\x12\x12\n\nball_speed\x18\x02 \x02(\x02\x12\x1a\n\x12initial_ball_speed\x18\x03 \x01(\x02*)\n\x04Team\x12\x0b\n\x07UNKNOWN\x10\x00\x12\n\n\x06YELLOW\x10\x01\x12\x08\n\x04\x42LUE\x10\x02')
)
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

_TEAM = _descriptor.EnumDescriptor(
  name='Team',
  full_name='Team',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='UNKNOWN', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='YELLOW', index=1, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='BLUE', index=2, number=2,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=541,
  serialized_end=582,
)
_sym_db.RegisterEnumDescriptor(_TEAM)

Team = enum_type_wrapper.EnumTypeWrapper(_TEAM)
UNKNOWN = 0
YELLOW = 1
BLUE = 2


_CONTROLLERREPLY_STATUSCODE = _descriptor.EnumDescriptor(
  name='StatusCode',
  full_name='ControllerReply.StatusCode',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='UNKNOWN_STATUS_CODE', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='OK', index=1, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='REJECTED', index=2, number=2,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=271,
  serialized_end=330,
)
_sym_db.RegisterEnumDescriptor(_CONTROLLERREPLY_STATUSCODE)

_CONTROLLERREPLY_VERIFICATION = _descriptor.EnumDescriptor(
  name='Verification',
  full_name='ControllerReply.Verification',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='UNKNOWN_VERIFICATION', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='VERIFIED', index=1, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='UNVERIFIED', index=2, number=2,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=332,
  serialized_end=402,
)
_sym_db.RegisterEnumDescriptor(_CONTROLLERREPLY_VERIFICATION)


_BOTID = _descriptor.Descriptor(
  name='BotId',
  full_name='BotId',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='BotId.id', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='team', full_name='BotId.team', index=1,
      number=2, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=36,
  serialized_end=76,
)


_LOCATION = _descriptor.Descriptor(
  name='Location',
  full_name='Location',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='x', full_name='Location.x', index=0,
      number=1, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='y', full_name='Location.y', index=1,
      number=2, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=78,
  serialized_end=110,
)


_CONTROLLERREPLY = _descriptor.Descriptor(
  name='ControllerReply',
  full_name='ControllerReply',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='status_code', full_name='ControllerReply.status_code', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='reason', full_name='ControllerReply.reason', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='next_token', full_name='ControllerReply.next_token', index=2,
      number=3, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='verification', full_name='ControllerReply.verification', index=3,
      number=4, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _CONTROLLERREPLY_STATUSCODE,
    _CONTROLLERREPLY_VERIFICATION,
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=113,
  serialized_end=402,
)


_SIGNATURE = _descriptor.Descriptor(
  name='Signature',
  full_name='Signature',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='token', full_name='Signature.token', index=0,
      number=1, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='pkcs1v15', full_name='Signature.pkcs1v15', index=1,
      number=2, type=12, cpp_type=9, label=2,
      has_default_value=False, default_value=_b(""),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=404,
  serialized_end=448,
)


_BALLSPEEDMEASUREMENT = _descriptor.Descriptor(
  name='BallSpeedMeasurement',
  full_name='BallSpeedMeasurement',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='timestamp', full_name='BallSpeedMeasurement.timestamp', index=0,
      number=1, type=4, cpp_type=4, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='ball_speed', full_name='BallSpeedMeasurement.ball_speed', index=1,
      number=2, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='initial_ball_speed', full_name='BallSpeedMeasurement.initial_ball_speed', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=450,
  serialized_end=539,
)

_BOTID.fields_by_name['team'].enum_type = _TEAM
_CONTROLLERREPLY.fields_by_name['status_code'].enum_type = _CONTROLLERREPLY_STATUSCODE
_CONTROLLERREPLY.fields_by_name['verification'].enum_type = _CONTROLLERREPLY_VERIFICATION
_CONTROLLERREPLY_STATUSCODE.containing_type = _CONTROLLERREPLY
_CONTROLLERREPLY_VERIFICATION.containing_type = _CONTROLLERREPLY
DESCRIPTOR.message_types_by_name['BotId'] = _BOTID
DESCRIPTOR.message_types_by_name['Location'] = _LOCATION
DESCRIPTOR.message_types_by_name['ControllerReply'] = _CONTROLLERREPLY
DESCRIPTOR.message_types_by_name['Signature'] = _SIGNATURE
DESCRIPTOR.message_types_by_name['BallSpeedMeasurement'] = _BALLSPEEDMEASUREMENT
DESCRIPTOR.enum_types_by_name['Team'] = _TEAM

BotId = _reflection.GeneratedProtocolMessageType('BotId', (_message.Message,), dict(
  DESCRIPTOR = _BOTID,
  __module__ = 'ssl_game_controller_common_pb2'
  # @@protoc_insertion_point(class_scope:BotId)
  ))
_sym_db.RegisterMessage(BotId)

Location = _reflection.GeneratedProtocolMessageType('Location', (_message.Message,), dict(
  DESCRIPTOR = _LOCATION,
  __module__ = 'ssl_game_controller_common_pb2'
  # @@protoc_insertion_point(class_scope:Location)
  ))
_sym_db.RegisterMessage(Location)

ControllerReply = _reflection.GeneratedProtocolMessageType('ControllerReply', (_message.Message,), dict(
  DESCRIPTOR = _CONTROLLERREPLY,
  __module__ = 'ssl_game_controller_common_pb2'
  # @@protoc_insertion_point(class_scope:ControllerReply)
  ))
_sym_db.RegisterMessage(ControllerReply)

Signature = _reflection.GeneratedProtocolMessageType('Signature', (_message.Message,), dict(
  DESCRIPTOR = _SIGNATURE,
  __module__ = 'ssl_game_controller_common_pb2'
  # @@protoc_insertion_point(class_scope:Signature)
  ))
_sym_db.RegisterMessage(Signature)

BallSpeedMeasurement = _reflection.GeneratedProtocolMessageType('BallSpeedMeasurement', (_message.Message,), dict(
  DESCRIPTOR = _BALLSPEEDMEASUREMENT,
  __module__ = 'ssl_game_controller_common_pb2'
  # @@protoc_insertion_point(class_scope:BallSpeedMeasurement)
  ))
_sym_db.RegisterMessage(BallSpeedMeasurement)


# @@protoc_insertion_point(module_scope)
