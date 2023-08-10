# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: simian/public/proto/motion_state_modifier.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from simian.public.proto.scenario import common_pb2 as simian_dot_public_dot_proto_dot_scenario_dot_common__pb2
from simian.public.proto import spatial_pb2 as simian_dot_public_dot_proto_dot_spatial__pb2


# @@protoc_insertion_point(post_imports)

_sym_db = _symbol_database.Default()


DESCRIPTOR = _descriptor.FileDescriptor(
  name='simian/public/proto/motion_state_modifier.proto',
  package='simian_public.motion_state_modifier',
  syntax='proto3',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n/simian/public/proto/motion_state_modifier.proto\x12#simian_public.motion_state_modifier\x1a)simian/public/proto/scenario/common.proto\x1a!simian/public/proto/spatial.proto\"\x91\x05\n\x17MotionStateModifierSpec\x12\x63\n\x0bpassthrough\x18\x01 \x01(\x0b\x32L.simian_public.motion_state_modifier.MotionStateModifierSpec.PassThroughSpecH\x00\x12h\n\x0emulti_modifier\x18\x02 \x01(\x0b\x32N.simian_public.motion_state_modifier.MotionStateModifierSpec.MultiModifierSpecH\x00\x12Z\n\x13suspension_modifier\x18\x03 \x01(\x0b\x32;.simian_public.motion_state_modifier.SuspensionModifierSpecH\x00\x12\x90\x01\n\x1f\x66irst_order_backward_difference\x18\x04 \x01(\x0b\x32\x65.simian_public.motion_state_modifier.MotionStateModifierSpec.FirstOrderBackwardDifferenceModifierSpecH\x00\x1a\x11\n\x0fPassThroughSpec\x1ah\n\x11MultiModifierSpec\x12S\n\rmodifier_list\x18\x01 \x03(\x0b\x32<.simian_public.motion_state_modifier.MotionStateModifierSpec\x1a*\n(FirstOrderBackwardDifferenceModifierSpecB\x0f\n\rmodifier_type\"u\n\x16SuspensionModifierSpec\x12H\n\nroll_pitch\x18\x01 \x01(\x0b\x32\x32.simian_public.motion_state_modifier.RollPitchSpecH\x00\x42\x11\n\x0fsuspension_type\"\xcb\x03\n\rRollPitchSpec\x12\x1e\n\x16spring_stiffness_front\x18\x01 \x01(\x01\x12\x1d\n\x15spring_stiffness_rear\x18\x02 \x01(\x01\x12!\n\x19\x64\x61mping_coefficient_front\x18\x03 \x01(\x01\x12 \n\x18\x64\x61mping_coefficient_rear\x18\x04 \x01(\x01\x12\x13\n\x0bsprung_mass\x18\x05 \x01(\x01\x12\x1a\n\x12spring_free_length\x18\x06 \x01(\x01\x12!\n\x19static_roll_center_height\x18\x07 \x01(\x01\x12\"\n\x1astatic_pitch_center_height\x18\x08 \x01(\x01\x12?\n\x13sprung_mass_inertia\x18\t \x01(\x0b\x32\".simian_public.spatial.InertiaSpec\x12\x10\n\x08max_roll\x18\x0b \x01(\x01\x12\x11\n\tmax_pitch\x18\x0c \x01(\x01\x12\x39\n\x0cwheel_config\x18\n \x03(\x0b\x32#.simian_public.scenario.WheelConfig\x12\x1d\n\x15num_integration_steps\x18\r \x01(\rb\x06proto3'
  ,
  dependencies=[simian_dot_public_dot_proto_dot_scenario_dot_common__pb2.DESCRIPTOR,simian_dot_public_dot_proto_dot_spatial__pb2.DESCRIPTOR,])




_MOTIONSTATEMODIFIERSPEC_PASSTHROUGHSPEC = _descriptor.Descriptor(
  name='PassThroughSpec',
  full_name='simian_public.motion_state_modifier.MotionStateModifierSpec.PassThroughSpec',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
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
  serialized_start=640,
  serialized_end=657,
)

_MOTIONSTATEMODIFIERSPEC_MULTIMODIFIERSPEC = _descriptor.Descriptor(
  name='MultiModifierSpec',
  full_name='simian_public.motion_state_modifier.MotionStateModifierSpec.MultiModifierSpec',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='modifier_list', full_name='simian_public.motion_state_modifier.MotionStateModifierSpec.MultiModifierSpec.modifier_list', index=0,
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
  serialized_start=659,
  serialized_end=763,
)

_MOTIONSTATEMODIFIERSPEC_FIRSTORDERBACKWARDDIFFERENCEMODIFIERSPEC = _descriptor.Descriptor(
  name='FirstOrderBackwardDifferenceModifierSpec',
  full_name='simian_public.motion_state_modifier.MotionStateModifierSpec.FirstOrderBackwardDifferenceModifierSpec',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
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
  serialized_start=765,
  serialized_end=807,
)

_MOTIONSTATEMODIFIERSPEC = _descriptor.Descriptor(
  name='MotionStateModifierSpec',
  full_name='simian_public.motion_state_modifier.MotionStateModifierSpec',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='passthrough', full_name='simian_public.motion_state_modifier.MotionStateModifierSpec.passthrough', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='multi_modifier', full_name='simian_public.motion_state_modifier.MotionStateModifierSpec.multi_modifier', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='suspension_modifier', full_name='simian_public.motion_state_modifier.MotionStateModifierSpec.suspension_modifier', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='first_order_backward_difference', full_name='simian_public.motion_state_modifier.MotionStateModifierSpec.first_order_backward_difference', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[_MOTIONSTATEMODIFIERSPEC_PASSTHROUGHSPEC, _MOTIONSTATEMODIFIERSPEC_MULTIMODIFIERSPEC, _MOTIONSTATEMODIFIERSPEC_FIRSTORDERBACKWARDDIFFERENCEMODIFIERSPEC, ],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
    _descriptor.OneofDescriptor(
      name='modifier_type', full_name='simian_public.motion_state_modifier.MotionStateModifierSpec.modifier_type',
      index=0, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
  ],
  serialized_start=167,
  serialized_end=824,
)


_SUSPENSIONMODIFIERSPEC = _descriptor.Descriptor(
  name='SuspensionModifierSpec',
  full_name='simian_public.motion_state_modifier.SuspensionModifierSpec',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='roll_pitch', full_name='simian_public.motion_state_modifier.SuspensionModifierSpec.roll_pitch', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
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
    _descriptor.OneofDescriptor(
      name='suspension_type', full_name='simian_public.motion_state_modifier.SuspensionModifierSpec.suspension_type',
      index=0, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
  ],
  serialized_start=826,
  serialized_end=943,
)


_ROLLPITCHSPEC = _descriptor.Descriptor(
  name='RollPitchSpec',
  full_name='simian_public.motion_state_modifier.RollPitchSpec',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='spring_stiffness_front', full_name='simian_public.motion_state_modifier.RollPitchSpec.spring_stiffness_front', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='spring_stiffness_rear', full_name='simian_public.motion_state_modifier.RollPitchSpec.spring_stiffness_rear', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='damping_coefficient_front', full_name='simian_public.motion_state_modifier.RollPitchSpec.damping_coefficient_front', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='damping_coefficient_rear', full_name='simian_public.motion_state_modifier.RollPitchSpec.damping_coefficient_rear', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='sprung_mass', full_name='simian_public.motion_state_modifier.RollPitchSpec.sprung_mass', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='spring_free_length', full_name='simian_public.motion_state_modifier.RollPitchSpec.spring_free_length', index=5,
      number=6, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='static_roll_center_height', full_name='simian_public.motion_state_modifier.RollPitchSpec.static_roll_center_height', index=6,
      number=7, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='static_pitch_center_height', full_name='simian_public.motion_state_modifier.RollPitchSpec.static_pitch_center_height', index=7,
      number=8, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='sprung_mass_inertia', full_name='simian_public.motion_state_modifier.RollPitchSpec.sprung_mass_inertia', index=8,
      number=9, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='max_roll', full_name='simian_public.motion_state_modifier.RollPitchSpec.max_roll', index=9,
      number=11, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='max_pitch', full_name='simian_public.motion_state_modifier.RollPitchSpec.max_pitch', index=10,
      number=12, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='wheel_config', full_name='simian_public.motion_state_modifier.RollPitchSpec.wheel_config', index=11,
      number=10, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='num_integration_steps', full_name='simian_public.motion_state_modifier.RollPitchSpec.num_integration_steps', index=12,
      number=13, type=13, cpp_type=3, label=1,
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
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=946,
  serialized_end=1405,
)

_MOTIONSTATEMODIFIERSPEC_PASSTHROUGHSPEC.containing_type = _MOTIONSTATEMODIFIERSPEC
_MOTIONSTATEMODIFIERSPEC_MULTIMODIFIERSPEC.fields_by_name['modifier_list'].message_type = _MOTIONSTATEMODIFIERSPEC
_MOTIONSTATEMODIFIERSPEC_MULTIMODIFIERSPEC.containing_type = _MOTIONSTATEMODIFIERSPEC
_MOTIONSTATEMODIFIERSPEC_FIRSTORDERBACKWARDDIFFERENCEMODIFIERSPEC.containing_type = _MOTIONSTATEMODIFIERSPEC
_MOTIONSTATEMODIFIERSPEC.fields_by_name['passthrough'].message_type = _MOTIONSTATEMODIFIERSPEC_PASSTHROUGHSPEC
_MOTIONSTATEMODIFIERSPEC.fields_by_name['multi_modifier'].message_type = _MOTIONSTATEMODIFIERSPEC_MULTIMODIFIERSPEC
_MOTIONSTATEMODIFIERSPEC.fields_by_name['suspension_modifier'].message_type = _SUSPENSIONMODIFIERSPEC
_MOTIONSTATEMODIFIERSPEC.fields_by_name['first_order_backward_difference'].message_type = _MOTIONSTATEMODIFIERSPEC_FIRSTORDERBACKWARDDIFFERENCEMODIFIERSPEC
_MOTIONSTATEMODIFIERSPEC.oneofs_by_name['modifier_type'].fields.append(
  _MOTIONSTATEMODIFIERSPEC.fields_by_name['passthrough'])
_MOTIONSTATEMODIFIERSPEC.fields_by_name['passthrough'].containing_oneof = _MOTIONSTATEMODIFIERSPEC.oneofs_by_name['modifier_type']
_MOTIONSTATEMODIFIERSPEC.oneofs_by_name['modifier_type'].fields.append(
  _MOTIONSTATEMODIFIERSPEC.fields_by_name['multi_modifier'])
_MOTIONSTATEMODIFIERSPEC.fields_by_name['multi_modifier'].containing_oneof = _MOTIONSTATEMODIFIERSPEC.oneofs_by_name['modifier_type']
_MOTIONSTATEMODIFIERSPEC.oneofs_by_name['modifier_type'].fields.append(
  _MOTIONSTATEMODIFIERSPEC.fields_by_name['suspension_modifier'])
_MOTIONSTATEMODIFIERSPEC.fields_by_name['suspension_modifier'].containing_oneof = _MOTIONSTATEMODIFIERSPEC.oneofs_by_name['modifier_type']
_MOTIONSTATEMODIFIERSPEC.oneofs_by_name['modifier_type'].fields.append(
  _MOTIONSTATEMODIFIERSPEC.fields_by_name['first_order_backward_difference'])
_MOTIONSTATEMODIFIERSPEC.fields_by_name['first_order_backward_difference'].containing_oneof = _MOTIONSTATEMODIFIERSPEC.oneofs_by_name['modifier_type']
_SUSPENSIONMODIFIERSPEC.fields_by_name['roll_pitch'].message_type = _ROLLPITCHSPEC
_SUSPENSIONMODIFIERSPEC.oneofs_by_name['suspension_type'].fields.append(
  _SUSPENSIONMODIFIERSPEC.fields_by_name['roll_pitch'])
_SUSPENSIONMODIFIERSPEC.fields_by_name['roll_pitch'].containing_oneof = _SUSPENSIONMODIFIERSPEC.oneofs_by_name['suspension_type']
_ROLLPITCHSPEC.fields_by_name['sprung_mass_inertia'].message_type = simian_dot_public_dot_proto_dot_spatial__pb2._INERTIASPEC
_ROLLPITCHSPEC.fields_by_name['wheel_config'].message_type = simian_dot_public_dot_proto_dot_scenario_dot_common__pb2._WHEELCONFIG
DESCRIPTOR.message_types_by_name['MotionStateModifierSpec'] = _MOTIONSTATEMODIFIERSPEC
DESCRIPTOR.message_types_by_name['SuspensionModifierSpec'] = _SUSPENSIONMODIFIERSPEC
DESCRIPTOR.message_types_by_name['RollPitchSpec'] = _ROLLPITCHSPEC
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

MotionStateModifierSpec = _reflection.GeneratedProtocolMessageType('MotionStateModifierSpec', (_message.Message,), {

  'PassThroughSpec' : _reflection.GeneratedProtocolMessageType('PassThroughSpec', (_message.Message,), {
    'DESCRIPTOR' : _MOTIONSTATEMODIFIERSPEC_PASSTHROUGHSPEC,
    '__module__' : 'simian.public.proto.motion_state_modifier_pb2'
    # @@protoc_insertion_point(class_scope:simian_public.motion_state_modifier.MotionStateModifierSpec.PassThroughSpec)
    })
  ,

  'MultiModifierSpec' : _reflection.GeneratedProtocolMessageType('MultiModifierSpec', (_message.Message,), {
    'DESCRIPTOR' : _MOTIONSTATEMODIFIERSPEC_MULTIMODIFIERSPEC,
    '__module__' : 'simian.public.proto.motion_state_modifier_pb2'
    # @@protoc_insertion_point(class_scope:simian_public.motion_state_modifier.MotionStateModifierSpec.MultiModifierSpec)
    })
  ,

  'FirstOrderBackwardDifferenceModifierSpec' : _reflection.GeneratedProtocolMessageType('FirstOrderBackwardDifferenceModifierSpec', (_message.Message,), {
    'DESCRIPTOR' : _MOTIONSTATEMODIFIERSPEC_FIRSTORDERBACKWARDDIFFERENCEMODIFIERSPEC,
    '__module__' : 'simian.public.proto.motion_state_modifier_pb2'
    # @@protoc_insertion_point(class_scope:simian_public.motion_state_modifier.MotionStateModifierSpec.FirstOrderBackwardDifferenceModifierSpec)
    })
  ,
  'DESCRIPTOR' : _MOTIONSTATEMODIFIERSPEC,
  '__module__' : 'simian.public.proto.motion_state_modifier_pb2'
  # @@protoc_insertion_point(class_scope:simian_public.motion_state_modifier.MotionStateModifierSpec)
  })
_sym_db.RegisterMessage(MotionStateModifierSpec)
_sym_db.RegisterMessage(MotionStateModifierSpec.PassThroughSpec)
_sym_db.RegisterMessage(MotionStateModifierSpec.MultiModifierSpec)
_sym_db.RegisterMessage(MotionStateModifierSpec.FirstOrderBackwardDifferenceModifierSpec)

SuspensionModifierSpec = _reflection.GeneratedProtocolMessageType('SuspensionModifierSpec', (_message.Message,), {
  'DESCRIPTOR' : _SUSPENSIONMODIFIERSPEC,
  '__module__' : 'simian.public.proto.motion_state_modifier_pb2'
  # @@protoc_insertion_point(class_scope:simian_public.motion_state_modifier.SuspensionModifierSpec)
  })
_sym_db.RegisterMessage(SuspensionModifierSpec)

RollPitchSpec = _reflection.GeneratedProtocolMessageType('RollPitchSpec', (_message.Message,), {
  'DESCRIPTOR' : _ROLLPITCHSPEC,
  '__module__' : 'simian.public.proto.motion_state_modifier_pb2'
  # @@protoc_insertion_point(class_scope:simian_public.motion_state_modifier.RollPitchSpec)
  })
_sym_db.RegisterMessage(RollPitchSpec)


# @@protoc_insertion_point(module_scope)
