# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: simian/public/proto/sensor_sim/pedestrian_props.proto
"""Generated protocol buffer code."""
from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




# @@protoc_insertion_point(post_imports)

_sym_db = _symbol_database.Default()


DESCRIPTOR = _descriptor.FileDescriptor(
  name='simian/public/proto/sensor_sim/pedestrian_props.proto',
  package='simian_public.proto.sensor_sim.pedestrian_props',
  syntax='proto3',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n5simian/public/proto/sensor_sim/pedestrian_props.proto\x12/simian_public.proto.sensor_sim.pedestrian_props*[\n\tOneHanded\x12\x14\n\x10NO_ONE_HAND_PROP\x10\x00\x12\x0c\n\x08UMBRELLA\x10\x01\x12\x11\n\rPARAMEDIC_BAG\x10\x02\x12\x17\n\x13ROLLERSUITCASE_GREY\x10\x03*\x89\x01\n\tTwoHanded\x12\x14\n\x10NO_TWO_HAND_PROP\x10\x00\x12\x11\n\rSHOPPING_CART\x10\x01\x12\x0e\n\nWHEELCHAIR\x10\x02\x12\x11\n\rBABY_STROLLER\x10\x03\x12\x17\n\x13\x42\x41\x42Y_STROLLER_EMPTY\x10\x04\x12\x17\n\x13WHEELCHAIR_OCCUPIED\x10\x05\x62\x06proto3'
)

_ONEHANDED = _descriptor.EnumDescriptor(
  name='OneHanded',
  full_name='simian_public.proto.sensor_sim.pedestrian_props.OneHanded',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='NO_ONE_HAND_PROP', index=0, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='UMBRELLA', index=1, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='PARAMEDIC_BAG', index=2, number=2,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='ROLLERSUITCASE_GREY', index=3, number=3,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=106,
  serialized_end=197,
)
_sym_db.RegisterEnumDescriptor(_ONEHANDED)

OneHanded = enum_type_wrapper.EnumTypeWrapper(_ONEHANDED)
_TWOHANDED = _descriptor.EnumDescriptor(
  name='TwoHanded',
  full_name='simian_public.proto.sensor_sim.pedestrian_props.TwoHanded',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='NO_TWO_HAND_PROP', index=0, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='SHOPPING_CART', index=1, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='WHEELCHAIR', index=2, number=2,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='BABY_STROLLER', index=3, number=3,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='BABY_STROLLER_EMPTY', index=4, number=4,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='WHEELCHAIR_OCCUPIED', index=5, number=5,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=200,
  serialized_end=337,
)
_sym_db.RegisterEnumDescriptor(_TWOHANDED)

TwoHanded = enum_type_wrapper.EnumTypeWrapper(_TWOHANDED)
NO_ONE_HAND_PROP = 0
UMBRELLA = 1
PARAMEDIC_BAG = 2
ROLLERSUITCASE_GREY = 3
NO_TWO_HAND_PROP = 0
SHOPPING_CART = 1
WHEELCHAIR = 2
BABY_STROLLER = 3
BABY_STROLLER_EMPTY = 4
WHEELCHAIR_OCCUPIED = 5


DESCRIPTOR.enum_types_by_name['OneHanded'] = _ONEHANDED
DESCRIPTOR.enum_types_by_name['TwoHanded'] = _TWOHANDED
_sym_db.RegisterFileDescriptor(DESCRIPTOR)


# @@protoc_insertion_point(module_scope)
