# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: simian/public/proto/function_mapping.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from simian.public.proto import transfer_function_pb2 as simian_dot_public_dot_proto_dot_transfer__function__pb2


# @@protoc_insertion_point(post_imports)

_sym_db = _symbol_database.Default()


DESCRIPTOR = _descriptor.FileDescriptor(
  name='simian/public/proto/function_mapping.proto',
  package='simian_public.function_mapping',
  syntax='proto3',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n*simian/public/proto/function_mapping.proto\x12\x1esimian_public.function_mapping\x1a+simian/public/proto/transfer_function.proto\"\xfd\x01\n\x0f\x46unctionMapping\x12@\n\npolynomial\x18\x01 \x01(\x0b\x32*.simian_public.function_mapping.PolynomialH\x00\x12R\n\x11transfer_function\x18\x02 \x01(\x0b\x32\x35.simian_public.transfer_function.TransferFunctionSpecH\x00\x12\x43\n\x0clookup_table\x18\x03 \x01(\x0b\x32+.simian_public.function_mapping.LookupTableH\x00\x42\x0f\n\rfunction_type\"S\n\nPolynomial\x12\x45\n\x05terms\x18\x01 \x03(\x0b\x32\x36.simian_public.function_mapping.PolynomialCoefficients\"?\n\x16PolynomialCoefficients\x12\x13\n\x0b\x63oefficient\x18\x01 \x01(\x01\x12\x10\n\x08\x65xponent\x18\x02 \x01(\r\"\xaa\x01\n\x0bLookupTable\x12\x85\x01\n0ruled_steering_torque_and_velocity_to_tire_angle\x18\x01 \x01(\x0b\x32I.simian_public.function_mapping.RuledSteeringTorqueAndVelocityToTireAngleH\x00\x42\x13\n\x11lookup_table_type\"\xe7\x01\n)RuledSteeringTorqueAndVelocityToTireAngle\x12\x63\n\x08\x65lements\x18\x01 \x03(\x0b\x32Q.simian_public.function_mapping.RuledSteeringTorqueAndVelocityToTireAngle.Element\x1aU\n\x07\x45lement\x12\x17\n\x0fsteering_torque\x18\x01 \x01(\x01\x12\x1d\n\x15longitudinal_velocity\x18\x02 \x01(\x01\x12\x12\n\ntire_angle\x18\x03 \x01(\x01\x62\x06proto3'
  ,
  dependencies=[simian_dot_public_dot_proto_dot_transfer__function__pb2.DESCRIPTOR,])




_FUNCTIONMAPPING = _descriptor.Descriptor(
  name='FunctionMapping',
  full_name='simian_public.function_mapping.FunctionMapping',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='polynomial', full_name='simian_public.function_mapping.FunctionMapping.polynomial', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='transfer_function', full_name='simian_public.function_mapping.FunctionMapping.transfer_function', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='lookup_table', full_name='simian_public.function_mapping.FunctionMapping.lookup_table', index=2,
      number=3, type=11, cpp_type=10, label=1,
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
      name='function_type', full_name='simian_public.function_mapping.FunctionMapping.function_type',
      index=0, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
  ],
  serialized_start=124,
  serialized_end=377,
)


_POLYNOMIAL = _descriptor.Descriptor(
  name='Polynomial',
  full_name='simian_public.function_mapping.Polynomial',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='terms', full_name='simian_public.function_mapping.Polynomial.terms', index=0,
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
  serialized_start=379,
  serialized_end=462,
)


_POLYNOMIALCOEFFICIENTS = _descriptor.Descriptor(
  name='PolynomialCoefficients',
  full_name='simian_public.function_mapping.PolynomialCoefficients',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='coefficient', full_name='simian_public.function_mapping.PolynomialCoefficients.coefficient', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='exponent', full_name='simian_public.function_mapping.PolynomialCoefficients.exponent', index=1,
      number=2, type=13, cpp_type=3, label=1,
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
  serialized_start=464,
  serialized_end=527,
)


_LOOKUPTABLE = _descriptor.Descriptor(
  name='LookupTable',
  full_name='simian_public.function_mapping.LookupTable',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='ruled_steering_torque_and_velocity_to_tire_angle', full_name='simian_public.function_mapping.LookupTable.ruled_steering_torque_and_velocity_to_tire_angle', index=0,
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
      name='lookup_table_type', full_name='simian_public.function_mapping.LookupTable.lookup_table_type',
      index=0, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
  ],
  serialized_start=530,
  serialized_end=700,
)


_RULEDSTEERINGTORQUEANDVELOCITYTOTIREANGLE_ELEMENT = _descriptor.Descriptor(
  name='Element',
  full_name='simian_public.function_mapping.RuledSteeringTorqueAndVelocityToTireAngle.Element',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='steering_torque', full_name='simian_public.function_mapping.RuledSteeringTorqueAndVelocityToTireAngle.Element.steering_torque', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='longitudinal_velocity', full_name='simian_public.function_mapping.RuledSteeringTorqueAndVelocityToTireAngle.Element.longitudinal_velocity', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='tire_angle', full_name='simian_public.function_mapping.RuledSteeringTorqueAndVelocityToTireAngle.Element.tire_angle', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
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
  serialized_start=849,
  serialized_end=934,
)

_RULEDSTEERINGTORQUEANDVELOCITYTOTIREANGLE = _descriptor.Descriptor(
  name='RuledSteeringTorqueAndVelocityToTireAngle',
  full_name='simian_public.function_mapping.RuledSteeringTorqueAndVelocityToTireAngle',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='elements', full_name='simian_public.function_mapping.RuledSteeringTorqueAndVelocityToTireAngle.elements', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[_RULEDSTEERINGTORQUEANDVELOCITYTOTIREANGLE_ELEMENT, ],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=703,
  serialized_end=934,
)

_FUNCTIONMAPPING.fields_by_name['polynomial'].message_type = _POLYNOMIAL
_FUNCTIONMAPPING.fields_by_name['transfer_function'].message_type = simian_dot_public_dot_proto_dot_transfer__function__pb2._TRANSFERFUNCTIONSPEC
_FUNCTIONMAPPING.fields_by_name['lookup_table'].message_type = _LOOKUPTABLE
_FUNCTIONMAPPING.oneofs_by_name['function_type'].fields.append(
  _FUNCTIONMAPPING.fields_by_name['polynomial'])
_FUNCTIONMAPPING.fields_by_name['polynomial'].containing_oneof = _FUNCTIONMAPPING.oneofs_by_name['function_type']
_FUNCTIONMAPPING.oneofs_by_name['function_type'].fields.append(
  _FUNCTIONMAPPING.fields_by_name['transfer_function'])
_FUNCTIONMAPPING.fields_by_name['transfer_function'].containing_oneof = _FUNCTIONMAPPING.oneofs_by_name['function_type']
_FUNCTIONMAPPING.oneofs_by_name['function_type'].fields.append(
  _FUNCTIONMAPPING.fields_by_name['lookup_table'])
_FUNCTIONMAPPING.fields_by_name['lookup_table'].containing_oneof = _FUNCTIONMAPPING.oneofs_by_name['function_type']
_POLYNOMIAL.fields_by_name['terms'].message_type = _POLYNOMIALCOEFFICIENTS
_LOOKUPTABLE.fields_by_name['ruled_steering_torque_and_velocity_to_tire_angle'].message_type = _RULEDSTEERINGTORQUEANDVELOCITYTOTIREANGLE
_LOOKUPTABLE.oneofs_by_name['lookup_table_type'].fields.append(
  _LOOKUPTABLE.fields_by_name['ruled_steering_torque_and_velocity_to_tire_angle'])
_LOOKUPTABLE.fields_by_name['ruled_steering_torque_and_velocity_to_tire_angle'].containing_oneof = _LOOKUPTABLE.oneofs_by_name['lookup_table_type']
_RULEDSTEERINGTORQUEANDVELOCITYTOTIREANGLE_ELEMENT.containing_type = _RULEDSTEERINGTORQUEANDVELOCITYTOTIREANGLE
_RULEDSTEERINGTORQUEANDVELOCITYTOTIREANGLE.fields_by_name['elements'].message_type = _RULEDSTEERINGTORQUEANDVELOCITYTOTIREANGLE_ELEMENT
DESCRIPTOR.message_types_by_name['FunctionMapping'] = _FUNCTIONMAPPING
DESCRIPTOR.message_types_by_name['Polynomial'] = _POLYNOMIAL
DESCRIPTOR.message_types_by_name['PolynomialCoefficients'] = _POLYNOMIALCOEFFICIENTS
DESCRIPTOR.message_types_by_name['LookupTable'] = _LOOKUPTABLE
DESCRIPTOR.message_types_by_name['RuledSteeringTorqueAndVelocityToTireAngle'] = _RULEDSTEERINGTORQUEANDVELOCITYTOTIREANGLE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

FunctionMapping = _reflection.GeneratedProtocolMessageType('FunctionMapping', (_message.Message,), {
  'DESCRIPTOR' : _FUNCTIONMAPPING,
  '__module__' : 'simian.public.proto.function_mapping_pb2'
  # @@protoc_insertion_point(class_scope:simian_public.function_mapping.FunctionMapping)
  })
_sym_db.RegisterMessage(FunctionMapping)

Polynomial = _reflection.GeneratedProtocolMessageType('Polynomial', (_message.Message,), {
  'DESCRIPTOR' : _POLYNOMIAL,
  '__module__' : 'simian.public.proto.function_mapping_pb2'
  # @@protoc_insertion_point(class_scope:simian_public.function_mapping.Polynomial)
  })
_sym_db.RegisterMessage(Polynomial)

PolynomialCoefficients = _reflection.GeneratedProtocolMessageType('PolynomialCoefficients', (_message.Message,), {
  'DESCRIPTOR' : _POLYNOMIALCOEFFICIENTS,
  '__module__' : 'simian.public.proto.function_mapping_pb2'
  # @@protoc_insertion_point(class_scope:simian_public.function_mapping.PolynomialCoefficients)
  })
_sym_db.RegisterMessage(PolynomialCoefficients)

LookupTable = _reflection.GeneratedProtocolMessageType('LookupTable', (_message.Message,), {
  'DESCRIPTOR' : _LOOKUPTABLE,
  '__module__' : 'simian.public.proto.function_mapping_pb2'
  # @@protoc_insertion_point(class_scope:simian_public.function_mapping.LookupTable)
  })
_sym_db.RegisterMessage(LookupTable)

RuledSteeringTorqueAndVelocityToTireAngle = _reflection.GeneratedProtocolMessageType('RuledSteeringTorqueAndVelocityToTireAngle', (_message.Message,), {

  'Element' : _reflection.GeneratedProtocolMessageType('Element', (_message.Message,), {
    'DESCRIPTOR' : _RULEDSTEERINGTORQUEANDVELOCITYTOTIREANGLE_ELEMENT,
    '__module__' : 'simian.public.proto.function_mapping_pb2'
    # @@protoc_insertion_point(class_scope:simian_public.function_mapping.RuledSteeringTorqueAndVelocityToTireAngle.Element)
    })
  ,
  'DESCRIPTOR' : _RULEDSTEERINGTORQUEANDVELOCITYTOTIREANGLE,
  '__module__' : 'simian.public.proto.function_mapping_pb2'
  # @@protoc_insertion_point(class_scope:simian_public.function_mapping.RuledSteeringTorqueAndVelocityToTireAngle)
  })
_sym_db.RegisterMessage(RuledSteeringTorqueAndVelocityToTireAngle)
_sym_db.RegisterMessage(RuledSteeringTorqueAndVelocityToTireAngle.Element)


# @@protoc_insertion_point(module_scope)
