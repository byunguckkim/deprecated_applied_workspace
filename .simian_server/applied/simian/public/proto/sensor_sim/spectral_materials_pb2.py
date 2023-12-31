# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: simian/public/proto/sensor_sim/spectral_materials.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from simian.public.proto import common_pb2 as simian_dot_public_dot_proto_dot_common__pb2


# @@protoc_insertion_point(post_imports)

_sym_db = _symbol_database.Default()


DESCRIPTOR = _descriptor.FileDescriptor(
  name='simian/public/proto/sensor_sim/spectral_materials.proto',
  package='simian_public.sensor_sim.spectral_materials',
  syntax='proto3',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n7simian/public/proto/sensor_sim/spectral_materials.proto\x12+simian_public.sensor_sim.spectral_materials\x1a simian/public/proto/common.proto\"\x8d\x03\n\x15MaterialColorModifier\x12\x31\n\x0bscalar_tint\x18\x01 \x01(\x0b\x32\x1a.simian_public.common.RGBAH\x00\x12\x35\n\x0fscalar_override\x18\x02 \x01(\x0b\x32\x1a.simian_public.common.RGBAH\x00\x12^\n\narray_tint\x18\x03 \x01(\x0b\x32H.simian_public.sensor_sim.spectral_materials.MaterialColorModifier.ArrayH\x00\x12\x62\n\x0e\x61rray_override\x18\x04 \x01(\x0b\x32H.simian_public.sensor_sim.spectral_materials.MaterialColorModifier.ArrayH\x00\x1a\x33\n\x05\x41rray\x12*\n\x06values\x18\x01 \x03(\x0b\x32\x1a.simian_public.common.RGBAB\x11\n\x0fmodifier_methodb\x06proto3'
  ,
  dependencies=[simian_dot_public_dot_proto_dot_common__pb2.DESCRIPTOR,])




_MATERIALCOLORMODIFIER_ARRAY = _descriptor.Descriptor(
  name='Array',
  full_name='simian_public.sensor_sim.spectral_materials.MaterialColorModifier.Array',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='values', full_name='simian_public.sensor_sim.spectral_materials.MaterialColorModifier.Array.values', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
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
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=466,
  serialized_end=517,
)

_MATERIALCOLORMODIFIER = _descriptor.Descriptor(
  name='MaterialColorModifier',
  full_name='simian_public.sensor_sim.spectral_materials.MaterialColorModifier',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='scalar_tint', full_name='simian_public.sensor_sim.spectral_materials.MaterialColorModifier.scalar_tint', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='scalar_override', full_name='simian_public.sensor_sim.spectral_materials.MaterialColorModifier.scalar_override', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='array_tint', full_name='simian_public.sensor_sim.spectral_materials.MaterialColorModifier.array_tint', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='array_override', full_name='simian_public.sensor_sim.spectral_materials.MaterialColorModifier.array_override', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[_MATERIALCOLORMODIFIER_ARRAY, ],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
    _descriptor.OneofDescriptor(
      name='modifier_method', full_name='simian_public.sensor_sim.spectral_materials.MaterialColorModifier.modifier_method',
      index=0, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
  ],
  serialized_start=139,
  serialized_end=536,
)

_MATERIALCOLORMODIFIER_ARRAY.fields_by_name['values'].message_type = simian_dot_public_dot_proto_dot_common__pb2._RGBA
_MATERIALCOLORMODIFIER_ARRAY.containing_type = _MATERIALCOLORMODIFIER
_MATERIALCOLORMODIFIER.fields_by_name['scalar_tint'].message_type = simian_dot_public_dot_proto_dot_common__pb2._RGBA
_MATERIALCOLORMODIFIER.fields_by_name['scalar_override'].message_type = simian_dot_public_dot_proto_dot_common__pb2._RGBA
_MATERIALCOLORMODIFIER.fields_by_name['array_tint'].message_type = _MATERIALCOLORMODIFIER_ARRAY
_MATERIALCOLORMODIFIER.fields_by_name['array_override'].message_type = _MATERIALCOLORMODIFIER_ARRAY
_MATERIALCOLORMODIFIER.oneofs_by_name['modifier_method'].fields.append(
  _MATERIALCOLORMODIFIER.fields_by_name['scalar_tint'])
_MATERIALCOLORMODIFIER.fields_by_name['scalar_tint'].containing_oneof = _MATERIALCOLORMODIFIER.oneofs_by_name['modifier_method']
_MATERIALCOLORMODIFIER.oneofs_by_name['modifier_method'].fields.append(
  _MATERIALCOLORMODIFIER.fields_by_name['scalar_override'])
_MATERIALCOLORMODIFIER.fields_by_name['scalar_override'].containing_oneof = _MATERIALCOLORMODIFIER.oneofs_by_name['modifier_method']
_MATERIALCOLORMODIFIER.oneofs_by_name['modifier_method'].fields.append(
  _MATERIALCOLORMODIFIER.fields_by_name['array_tint'])
_MATERIALCOLORMODIFIER.fields_by_name['array_tint'].containing_oneof = _MATERIALCOLORMODIFIER.oneofs_by_name['modifier_method']
_MATERIALCOLORMODIFIER.oneofs_by_name['modifier_method'].fields.append(
  _MATERIALCOLORMODIFIER.fields_by_name['array_override'])
_MATERIALCOLORMODIFIER.fields_by_name['array_override'].containing_oneof = _MATERIALCOLORMODIFIER.oneofs_by_name['modifier_method']
DESCRIPTOR.message_types_by_name['MaterialColorModifier'] = _MATERIALCOLORMODIFIER
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

MaterialColorModifier = _reflection.GeneratedProtocolMessageType('MaterialColorModifier', (_message.Message,), {

  'Array' : _reflection.GeneratedProtocolMessageType('Array', (_message.Message,), {
    'DESCRIPTOR' : _MATERIALCOLORMODIFIER_ARRAY,
    '__module__' : 'simian.public.proto.sensor_sim.spectral_materials_pb2'
    # @@protoc_insertion_point(class_scope:simian_public.sensor_sim.spectral_materials.MaterialColorModifier.Array)
    })
  ,
  'DESCRIPTOR' : _MATERIALCOLORMODIFIER,
  '__module__' : 'simian.public.proto.sensor_sim.spectral_materials_pb2'
  # @@protoc_insertion_point(class_scope:simian_public.sensor_sim.spectral_materials.MaterialColorModifier)
  })
_sym_db.RegisterMessage(MaterialColorModifier)
_sym_db.RegisterMessage(MaterialColorModifier.Array)


# @@protoc_insertion_point(module_scope)
