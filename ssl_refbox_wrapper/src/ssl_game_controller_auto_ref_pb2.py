# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ssl_game_controller_auto_ref.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import ssl_game_controller_common_pb2
import ssl_game_event_2019_pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='ssl_game_controller_auto_ref.proto',
  package='',
  serialized_pb=_b('\n\"ssl_game_controller_auto_ref.proto\x1a ssl_game_controller_common.proto\x1a\x19ssl_game_event_2019.proto\"H\n\x13\x41utoRefRegistration\x12\x12\n\nidentifier\x18\x01 \x02(\t\x12\x1d\n\tsignature\x18\x02 \x01(\x0b\x32\n.Signature\"\x83\x01\n\x13\x41utoRefToController\x12\x1d\n\tsignature\x18\x01 \x01(\x0b\x32\n.Signature\x12\x1e\n\ngame_event\x18\x02 \x01(\x0b\x32\n.GameEvent\x12-\n\x10\x61uto_ref_message\x18\x03 \x01(\x0b\x32\x0f.AutoRefMessageB\x02\x18\x01\"J\n\x13\x43ontrollerToAutoRef\x12,\n\x10\x63ontroller_reply\x18\x01 \x01(\x0b\x32\x10.ControllerReplyH\x00\x42\x05\n\x03msg\"\xe1\x01\n\x0e\x41utoRefMessage\x12\x10\n\x06\x63ustom\x18\x01 \x01(\tH\x00\x12\x34\n\rwait_for_bots\x18\x02 \x01(\x0b\x32\x1b.AutoRefMessage.WaitForBotsH\x00\x1a|\n\x0bWaitForBots\x12\x37\n\tviolators\x18\x01 \x03(\x0b\x32$.AutoRefMessage.WaitForBots.Violator\x1a\x34\n\x08Violator\x12\x16\n\x06\x62ot_id\x18\x01 \x02(\x0b\x32\x06.BotId\x12\x10\n\x08\x64istance\x18\x02 \x02(\x02\x42\t\n\x07message')
  ,
  dependencies=[ssl_game_controller_common_pb2.DESCRIPTOR,ssl_game_event_2019_pb2.DESCRIPTOR,])
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_AUTOREFREGISTRATION = _descriptor.Descriptor(
  name='AutoRefRegistration',
  full_name='AutoRefRegistration',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='identifier', full_name='AutoRefRegistration.identifier', index=0,
      number=1, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='signature', full_name='AutoRefRegistration.signature', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
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
  serialized_start=99,
  serialized_end=171,
)


_AUTOREFTOCONTROLLER = _descriptor.Descriptor(
  name='AutoRefToController',
  full_name='AutoRefToController',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='signature', full_name='AutoRefToController.signature', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='game_event', full_name='AutoRefToController.game_event', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='auto_ref_message', full_name='AutoRefToController.auto_ref_message', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=_descriptor._ParseOptions(descriptor_pb2.FieldOptions(), _b('\030\001'))),
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
  serialized_start=174,
  serialized_end=305,
)


_CONTROLLERTOAUTOREF = _descriptor.Descriptor(
  name='ControllerToAutoRef',
  full_name='ControllerToAutoRef',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='controller_reply', full_name='ControllerToAutoRef.controller_reply', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
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
    _descriptor.OneofDescriptor(
      name='msg', full_name='ControllerToAutoRef.msg',
      index=0, containing_type=None, fields=[]),
  ],
  serialized_start=307,
  serialized_end=381,
)


_AUTOREFMESSAGE_WAITFORBOTS_VIOLATOR = _descriptor.Descriptor(
  name='Violator',
  full_name='AutoRefMessage.WaitForBots.Violator',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='bot_id', full_name='AutoRefMessage.WaitForBots.Violator.bot_id', index=0,
      number=1, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='distance', full_name='AutoRefMessage.WaitForBots.Violator.distance', index=1,
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
  serialized_start=546,
  serialized_end=598,
)

_AUTOREFMESSAGE_WAITFORBOTS = _descriptor.Descriptor(
  name='WaitForBots',
  full_name='AutoRefMessage.WaitForBots',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='violators', full_name='AutoRefMessage.WaitForBots.violators', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[_AUTOREFMESSAGE_WAITFORBOTS_VIOLATOR, ],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=474,
  serialized_end=598,
)

_AUTOREFMESSAGE = _descriptor.Descriptor(
  name='AutoRefMessage',
  full_name='AutoRefMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='custom', full_name='AutoRefMessage.custom', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='wait_for_bots', full_name='AutoRefMessage.wait_for_bots', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[_AUTOREFMESSAGE_WAITFORBOTS, ],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  oneofs=[
    _descriptor.OneofDescriptor(
      name='message', full_name='AutoRefMessage.message',
      index=0, containing_type=None, fields=[]),
  ],
  serialized_start=384,
  serialized_end=609,
)

_AUTOREFREGISTRATION.fields_by_name['signature'].message_type = ssl_game_controller_common_pb2._SIGNATURE
_AUTOREFTOCONTROLLER.fields_by_name['signature'].message_type = ssl_game_controller_common_pb2._SIGNATURE
_AUTOREFTOCONTROLLER.fields_by_name['game_event'].message_type = ssl_game_event_2019_pb2._GAMEEVENT
_AUTOREFTOCONTROLLER.fields_by_name['auto_ref_message'].message_type = _AUTOREFMESSAGE
_CONTROLLERTOAUTOREF.fields_by_name['controller_reply'].message_type = ssl_game_controller_common_pb2._CONTROLLERREPLY
_CONTROLLERTOAUTOREF.oneofs_by_name['msg'].fields.append(
  _CONTROLLERTOAUTOREF.fields_by_name['controller_reply'])
_CONTROLLERTOAUTOREF.fields_by_name['controller_reply'].containing_oneof = _CONTROLLERTOAUTOREF.oneofs_by_name['msg']
_AUTOREFMESSAGE_WAITFORBOTS_VIOLATOR.fields_by_name['bot_id'].message_type = ssl_game_controller_common_pb2._BOTID
_AUTOREFMESSAGE_WAITFORBOTS_VIOLATOR.containing_type = _AUTOREFMESSAGE_WAITFORBOTS
_AUTOREFMESSAGE_WAITFORBOTS.fields_by_name['violators'].message_type = _AUTOREFMESSAGE_WAITFORBOTS_VIOLATOR
_AUTOREFMESSAGE_WAITFORBOTS.containing_type = _AUTOREFMESSAGE
_AUTOREFMESSAGE.fields_by_name['wait_for_bots'].message_type = _AUTOREFMESSAGE_WAITFORBOTS
_AUTOREFMESSAGE.oneofs_by_name['message'].fields.append(
  _AUTOREFMESSAGE.fields_by_name['custom'])
_AUTOREFMESSAGE.fields_by_name['custom'].containing_oneof = _AUTOREFMESSAGE.oneofs_by_name['message']
_AUTOREFMESSAGE.oneofs_by_name['message'].fields.append(
  _AUTOREFMESSAGE.fields_by_name['wait_for_bots'])
_AUTOREFMESSAGE.fields_by_name['wait_for_bots'].containing_oneof = _AUTOREFMESSAGE.oneofs_by_name['message']
DESCRIPTOR.message_types_by_name['AutoRefRegistration'] = _AUTOREFREGISTRATION
DESCRIPTOR.message_types_by_name['AutoRefToController'] = _AUTOREFTOCONTROLLER
DESCRIPTOR.message_types_by_name['ControllerToAutoRef'] = _CONTROLLERTOAUTOREF
DESCRIPTOR.message_types_by_name['AutoRefMessage'] = _AUTOREFMESSAGE

AutoRefRegistration = _reflection.GeneratedProtocolMessageType('AutoRefRegistration', (_message.Message,), dict(
  DESCRIPTOR = _AUTOREFREGISTRATION,
  __module__ = 'ssl_game_controller_auto_ref_pb2'
  # @@protoc_insertion_point(class_scope:AutoRefRegistration)
  ))
_sym_db.RegisterMessage(AutoRefRegistration)

AutoRefToController = _reflection.GeneratedProtocolMessageType('AutoRefToController', (_message.Message,), dict(
  DESCRIPTOR = _AUTOREFTOCONTROLLER,
  __module__ = 'ssl_game_controller_auto_ref_pb2'
  # @@protoc_insertion_point(class_scope:AutoRefToController)
  ))
_sym_db.RegisterMessage(AutoRefToController)

ControllerToAutoRef = _reflection.GeneratedProtocolMessageType('ControllerToAutoRef', (_message.Message,), dict(
  DESCRIPTOR = _CONTROLLERTOAUTOREF,
  __module__ = 'ssl_game_controller_auto_ref_pb2'
  # @@protoc_insertion_point(class_scope:ControllerToAutoRef)
  ))
_sym_db.RegisterMessage(ControllerToAutoRef)

AutoRefMessage = _reflection.GeneratedProtocolMessageType('AutoRefMessage', (_message.Message,), dict(

  WaitForBots = _reflection.GeneratedProtocolMessageType('WaitForBots', (_message.Message,), dict(

    Violator = _reflection.GeneratedProtocolMessageType('Violator', (_message.Message,), dict(
      DESCRIPTOR = _AUTOREFMESSAGE_WAITFORBOTS_VIOLATOR,
      __module__ = 'ssl_game_controller_auto_ref_pb2'
      # @@protoc_insertion_point(class_scope:AutoRefMessage.WaitForBots.Violator)
      ))
    ,
    DESCRIPTOR = _AUTOREFMESSAGE_WAITFORBOTS,
    __module__ = 'ssl_game_controller_auto_ref_pb2'
    # @@protoc_insertion_point(class_scope:AutoRefMessage.WaitForBots)
    ))
  ,
  DESCRIPTOR = _AUTOREFMESSAGE,
  __module__ = 'ssl_game_controller_auto_ref_pb2'
  # @@protoc_insertion_point(class_scope:AutoRefMessage)
  ))
_sym_db.RegisterMessage(AutoRefMessage)
_sym_db.RegisterMessage(AutoRefMessage.WaitForBots)
_sym_db.RegisterMessage(AutoRefMessage.WaitForBots.Violator)


_AUTOREFTOCONTROLLER.fields_by_name['auto_ref_message'].has_options = True
_AUTOREFTOCONTROLLER.fields_by_name['auto_ref_message']._options = _descriptor._ParseOptions(descriptor_pb2.FieldOptions(), _b('\030\001'))
# @@protoc_insertion_point(module_scope)
