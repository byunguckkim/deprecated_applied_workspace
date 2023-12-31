# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: simian/public/proto/stack_state.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




# @@protoc_insertion_point(post_imports)

_sym_db = _symbol_database.Default()


DESCRIPTOR = _descriptor.FileDescriptor(
  name='simian/public/proto/stack_state.proto',
  package='simian_public.stack_state',
  syntax='proto3',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n%simian/public/proto/stack_state.proto\x12\x19simian_public.stack_state\"\x80\x02\n\x15StackStateInformation\x12\x14\n\x0c\x64isplay_name\x18\x01 \x01(\t\x12\\\n\x12\x64isplay_name_color\x18\x02 \x01(\x0e\x32@.simian_public.stack_state.StackStateInformation.StackStateColor\"s\n\x0fStackStateColor\x12\t\n\x05WHITE\x10\x00\x12\x08\n\x04GRAY\x10\x01\x12\n\n\x06PURPLE\x10\x02\x12\x08\n\x04\x42LUE\x10\x03\x12\t\n\x05GREEN\x10\x04\x12\n\n\x06YELLOW\x10\x05\x12\n\n\x06ORANGE\x10\x06\x12\x07\n\x03RED\x10\x07\x12\t\n\x05\x42ROWN\x10\x08\x62\x06proto3'
)



_STACKSTATEINFORMATION_STACKSTATECOLOR = _descriptor.EnumDescriptor(
  name='StackStateColor',
  full_name='simian_public.stack_state.StackStateInformation.StackStateColor',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='WHITE', index=0, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='GRAY', index=1, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='PURPLE', index=2, number=2,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='BLUE', index=3, number=3,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='GREEN', index=4, number=4,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='YELLOW', index=5, number=5,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='ORANGE', index=6, number=6,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='RED', index=7, number=7,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='BROWN', index=8, number=8,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=210,
  serialized_end=325,
)
_sym_db.RegisterEnumDescriptor(_STACKSTATEINFORMATION_STACKSTATECOLOR)


_STACKSTATEINFORMATION = _descriptor.Descriptor(
  name='StackStateInformation',
  full_name='simian_public.stack_state.StackStateInformation',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='display_name', full_name='simian_public.stack_state.StackStateInformation.display_name', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='display_name_color', full_name='simian_public.stack_state.StackStateInformation.display_name_color', index=1,
      number=2, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _STACKSTATEINFORMATION_STACKSTATECOLOR,
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=69,
  serialized_end=325,
)

_STACKSTATEINFORMATION.fields_by_name['display_name_color'].enum_type = _STACKSTATEINFORMATION_STACKSTATECOLOR
_STACKSTATEINFORMATION_STACKSTATECOLOR.containing_type = _STACKSTATEINFORMATION
DESCRIPTOR.message_types_by_name['StackStateInformation'] = _STACKSTATEINFORMATION
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

StackStateInformation = _reflection.GeneratedProtocolMessageType('StackStateInformation', (_message.Message,), {
  'DESCRIPTOR' : _STACKSTATEINFORMATION,
  '__module__' : 'simian.public.proto.stack_state_pb2'
  # @@protoc_insertion_point(class_scope:simian_public.stack_state.StackStateInformation)
  })
_sym_db.RegisterMessage(StackStateInformation)


# @@protoc_insertion_point(module_scope)
