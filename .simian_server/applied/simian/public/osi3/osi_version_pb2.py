# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: simian/public/osi3/osi_version.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from google.protobuf import descriptor_pb2 as google_dot_protobuf_dot_descriptor__pb2


# @@protoc_insertion_point(post_imports)

_sym_db = _symbol_database.Default()


DESCRIPTOR = _descriptor.FileDescriptor(
  name='simian/public/osi3/osi_version.proto',
  package='osi3',
  syntax='proto2',
  serialized_options=b'H\001\302\306\'\002\010\003\302\306\'\002\020\000\302\306\'\002\030\000',
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n$simian/public/osi3/osi_version.proto\x12\x04osi3\x1a google/protobuf/descriptor.proto\"W\n\x10InterfaceVersion\x12\x15\n\rversion_major\x18\x01 \x01(\r\x12\x15\n\rversion_minor\x18\x02 \x01(\r\x12\x15\n\rversion_patch\x18\x03 \x01(\r:Y\n\x19\x63urrent_interface_version\x12\x1c.google.protobuf.FileOptions\x18\xe8\xf8\x04 \x01(\x0b\x32\x16.osi3.InterfaceVersionB\x14H\x01\xc2\xc6\'\x02\x08\x03\xc2\xc6\'\x02\x10\x00\xc2\xc6\'\x02\x18\x00'
  ,
  dependencies=[google_dot_protobuf_dot_descriptor__pb2.DESCRIPTOR,])


CURRENT_INTERFACE_VERSION_FIELD_NUMBER = 81000
current_interface_version = _descriptor.FieldDescriptor(
  name='current_interface_version', full_name='osi3.current_interface_version', index=0,
  number=81000, type=11, cpp_type=10, label=1,
  has_default_value=False, default_value=None,
  message_type=None, enum_type=None, containing_type=None,
  is_extension=True, extension_scope=None,
  serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key)


_INTERFACEVERSION = _descriptor.Descriptor(
  name='InterfaceVersion',
  full_name='osi3.InterfaceVersion',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='version_major', full_name='osi3.InterfaceVersion.version_major', index=0,
      number=1, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='version_minor', full_name='osi3.InterfaceVersion.version_minor', index=1,
      number=2, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='version_patch', full_name='osi3.InterfaceVersion.version_patch', index=2,
      number=3, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=80,
  serialized_end=167,
)

DESCRIPTOR.message_types_by_name['InterfaceVersion'] = _INTERFACEVERSION
DESCRIPTOR.extensions_by_name['current_interface_version'] = current_interface_version
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

InterfaceVersion = _reflection.GeneratedProtocolMessageType('InterfaceVersion', (_message.Message,), {
  'DESCRIPTOR' : _INTERFACEVERSION,
  '__module__' : 'simian.public.osi3.osi_version_pb2'
  # @@protoc_insertion_point(class_scope:osi3.InterfaceVersion)
  })
_sym_db.RegisterMessage(InterfaceVersion)

current_interface_version.message_type = _INTERFACEVERSION
google_dot_protobuf_dot_descriptor__pb2.FileOptions.RegisterExtension(current_interface_version)

DESCRIPTOR._options = None
# @@protoc_insertion_point(module_scope)
