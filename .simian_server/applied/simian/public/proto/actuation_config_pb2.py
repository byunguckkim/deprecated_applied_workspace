# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: simian/public/proto/actuation_config.proto
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
  name='simian/public/proto/actuation_config.proto',
  package='simian_public.actuation_config',
  syntax='proto3',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n*simian/public/proto/actuation_config.proto\x12\x1esimian_public.actuation_config\"\xcb\x10\n\x0f\x41\x63tuationConfig\x12K\n\tactuators\x18\x01 \x03(\x0b\x32\x38.simian_public.actuation_config.ActuationConfig.Actuator\x1a\xeb\x01\n\x08\x41\x63tuator\x12\x0c\n\x04name\x18\x01 \x01(\t\x12V\n\thydraulic\x18\x02 \x01(\x0b\x32\x41.simian_public.actuation_config.ActuationConfig.HydraulicActuatorH\x00\x12h\n\x13speed_state_machine\x18\x03 \x01(\x0b\x32I.simian_public.actuation_config.ActuationConfig.SpeedStateMachineActuatorH\x00\x42\x0f\n\ractuator_type\x1a\xfa\x01\n\x19SpeedStateMachineActuator\x12\x11\n\tmax_speed\x18\x01 \x01(\x01\x12\x18\n\x10wind_up_duration\x18\x02 \x01(\x01\x12\x19\n\x11speed_up_duration\x18\x03 \x01(\x01\x12\x1a\n\x12slow_down_duration\x18\x04 \x01(\x01\x12\x1a\n\x12zero_input_damping\x18\x05 \x01(\x01\x12\x19\n\x11max_input_damping\x18\x06 \x01(\x01\x12\x14\n\x0cmin_position\x18\x07 \x01(\x01\x12\x14\n\x0cmax_position\x18\x08 \x01(\x01\x12\x16\n\x0e\x65nforce_limits\x18\t \x01(\x08\x1a\xff\x0b\n\x11HydraulicActuator\x12\x62\n\ncoordinate\x18\x01 \x01(\x0b\x32L.simian_public.actuation_config.ActuationConfig.HydraulicActuator.CoordinateH\x00\x12g\n\x0eoil_properties\x18\x02 \x01(\x0b\x32O.simian_public.actuation_config.ActuationConfig.HydraulicActuator.OilProperties\x12u\n\x15mechanical_properties\x18\x03 \x01(\x0b\x32V.simian_public.actuation_config.ActuationConfig.HydraulicActuator.MechanicalProperties\x12\x65\n\rcontrol_valve\x18\x04 \x01(\x0b\x32N.simian_public.actuation_config.ActuationConfig.HydraulicActuator.ControlValve\x12o\n\x12hydraulic_pressure\x18\x05 \x01(\x0b\x32S.simian_public.actuation_config.ActuationConfig.HydraulicActuator.HydraulicPressure\x12\x63\n\x0crelief_valve\x18\x06 \x01(\x0b\x32M.simian_public.actuation_config.ActuationConfig.HydraulicActuator.ReliefValve\x12\x1f\n\x17\x63ylinder_stop_stiffness\x18\x07 \x01(\x01\x12q\n\x13\x63ylinder_properties\x18\x08 \x01(\x0b\x32T.simian_public.actuation_config.ActuationConfig.HydraulicActuator.CylinderProperties\x1a\x1a\n\nCoordinate\x12\x0c\n\x04name\x18\x01 \x01(\t\x1aX\n\rOilProperties\x12 \n\x18neutral_spring_stiffness\x18\x01 \x01(\x01\x12\x0f\n\x07\x64\x65nsity\x18\x02 \x01(\x01\x12\x14\n\x0c\x62ulk_modulus\x18\x03 \x01(\x01\x1a\x90\x01\n\x14MechanicalProperties\x12+\n#piston_viscous_friction_coefficient\x18\x01 \x01(\x01\x12%\n\x1dpiston_coulomb_friction_force\x18\x02 \x01(\x01\x12$\n\x1cpiston_static_friction_force\x18\x03 \x01(\x01\x1aJ\n\x0c\x43ontrolValve\x12\x1d\n\x15\x64ischarge_coefficient\x18\x01 \x01(\x01\x12\x1b\n\x13\x63ontrol_coefficient\x18\x02 \x01(\x01\x1a\x32\n\x11HydraulicPressure\x12\x0e\n\x06supply\x18\x01 \x01(\x01\x12\r\n\x05\x64rain\x18\x02 \x01(\x01\x1a\x43\n\x0bReliefValve\x12\x19\n\x11\x63racking_pressure\x18\x01 \x01(\x01\x12\x19\n\x11slope_coefficient\x18\x02 \x01(\x01\x1a\xf8\x01\n\x12\x43ylinderProperties\x12)\n!pressurized_piston_area_extension\x18\x01 \x01(\x01\x12*\n\"pressurized_piston_area_retraction\x18\x02 \x01(\x01\x12\x1c\n\x14piston_stroke_length\x18\x03 \x01(\x01\x12%\n\x1d\x63onstant_oil_volume_extension\x18\x04 \x01(\x01\x12&\n\x1e\x63onstant_oil_volume_retraction\x18\x05 \x01(\x01\x12\x1e\n\x16minimum_mount_distance\x18\x06 \x01(\x01\x42\x0c\n\nattachmentb\x06proto3'
)




_ACTUATIONCONFIG_ACTUATOR = _descriptor.Descriptor(
  name='Actuator',
  full_name='simian_public.actuation_config.ActuationConfig.Actuator',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='name', full_name='simian_public.actuation_config.ActuationConfig.Actuator.name', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='hydraulic', full_name='simian_public.actuation_config.ActuationConfig.Actuator.hydraulic', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='speed_state_machine', full_name='simian_public.actuation_config.ActuationConfig.Actuator.speed_state_machine', index=2,
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
      name='actuator_type', full_name='simian_public.actuation_config.ActuationConfig.Actuator.actuator_type',
      index=0, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
  ],
  serialized_start=176,
  serialized_end=411,
)

_ACTUATIONCONFIG_SPEEDSTATEMACHINEACTUATOR = _descriptor.Descriptor(
  name='SpeedStateMachineActuator',
  full_name='simian_public.actuation_config.ActuationConfig.SpeedStateMachineActuator',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='max_speed', full_name='simian_public.actuation_config.ActuationConfig.SpeedStateMachineActuator.max_speed', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='wind_up_duration', full_name='simian_public.actuation_config.ActuationConfig.SpeedStateMachineActuator.wind_up_duration', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='speed_up_duration', full_name='simian_public.actuation_config.ActuationConfig.SpeedStateMachineActuator.speed_up_duration', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='slow_down_duration', full_name='simian_public.actuation_config.ActuationConfig.SpeedStateMachineActuator.slow_down_duration', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='zero_input_damping', full_name='simian_public.actuation_config.ActuationConfig.SpeedStateMachineActuator.zero_input_damping', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='max_input_damping', full_name='simian_public.actuation_config.ActuationConfig.SpeedStateMachineActuator.max_input_damping', index=5,
      number=6, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='min_position', full_name='simian_public.actuation_config.ActuationConfig.SpeedStateMachineActuator.min_position', index=6,
      number=7, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='max_position', full_name='simian_public.actuation_config.ActuationConfig.SpeedStateMachineActuator.max_position', index=7,
      number=8, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='enforce_limits', full_name='simian_public.actuation_config.ActuationConfig.SpeedStateMachineActuator.enforce_limits', index=8,
      number=9, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
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
  serialized_start=414,
  serialized_end=664,
)

_ACTUATIONCONFIG_HYDRAULICACTUATOR_COORDINATE = _descriptor.Descriptor(
  name='Coordinate',
  full_name='simian_public.actuation_config.ActuationConfig.HydraulicActuator.Coordinate',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='name', full_name='simian_public.actuation_config.ActuationConfig.HydraulicActuator.Coordinate.name', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
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
  serialized_start=1477,
  serialized_end=1503,
)

_ACTUATIONCONFIG_HYDRAULICACTUATOR_OILPROPERTIES = _descriptor.Descriptor(
  name='OilProperties',
  full_name='simian_public.actuation_config.ActuationConfig.HydraulicActuator.OilProperties',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='neutral_spring_stiffness', full_name='simian_public.actuation_config.ActuationConfig.HydraulicActuator.OilProperties.neutral_spring_stiffness', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='density', full_name='simian_public.actuation_config.ActuationConfig.HydraulicActuator.OilProperties.density', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='bulk_modulus', full_name='simian_public.actuation_config.ActuationConfig.HydraulicActuator.OilProperties.bulk_modulus', index=2,
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
  serialized_start=1505,
  serialized_end=1593,
)

_ACTUATIONCONFIG_HYDRAULICACTUATOR_MECHANICALPROPERTIES = _descriptor.Descriptor(
  name='MechanicalProperties',
  full_name='simian_public.actuation_config.ActuationConfig.HydraulicActuator.MechanicalProperties',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='piston_viscous_friction_coefficient', full_name='simian_public.actuation_config.ActuationConfig.HydraulicActuator.MechanicalProperties.piston_viscous_friction_coefficient', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='piston_coulomb_friction_force', full_name='simian_public.actuation_config.ActuationConfig.HydraulicActuator.MechanicalProperties.piston_coulomb_friction_force', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='piston_static_friction_force', full_name='simian_public.actuation_config.ActuationConfig.HydraulicActuator.MechanicalProperties.piston_static_friction_force', index=2,
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
  serialized_start=1596,
  serialized_end=1740,
)

_ACTUATIONCONFIG_HYDRAULICACTUATOR_CONTROLVALVE = _descriptor.Descriptor(
  name='ControlValve',
  full_name='simian_public.actuation_config.ActuationConfig.HydraulicActuator.ControlValve',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='discharge_coefficient', full_name='simian_public.actuation_config.ActuationConfig.HydraulicActuator.ControlValve.discharge_coefficient', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='control_coefficient', full_name='simian_public.actuation_config.ActuationConfig.HydraulicActuator.ControlValve.control_coefficient', index=1,
      number=2, type=1, cpp_type=5, label=1,
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
  serialized_start=1742,
  serialized_end=1816,
)

_ACTUATIONCONFIG_HYDRAULICACTUATOR_HYDRAULICPRESSURE = _descriptor.Descriptor(
  name='HydraulicPressure',
  full_name='simian_public.actuation_config.ActuationConfig.HydraulicActuator.HydraulicPressure',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='supply', full_name='simian_public.actuation_config.ActuationConfig.HydraulicActuator.HydraulicPressure.supply', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='drain', full_name='simian_public.actuation_config.ActuationConfig.HydraulicActuator.HydraulicPressure.drain', index=1,
      number=2, type=1, cpp_type=5, label=1,
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
  serialized_start=1818,
  serialized_end=1868,
)

_ACTUATIONCONFIG_HYDRAULICACTUATOR_RELIEFVALVE = _descriptor.Descriptor(
  name='ReliefValve',
  full_name='simian_public.actuation_config.ActuationConfig.HydraulicActuator.ReliefValve',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='cracking_pressure', full_name='simian_public.actuation_config.ActuationConfig.HydraulicActuator.ReliefValve.cracking_pressure', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='slope_coefficient', full_name='simian_public.actuation_config.ActuationConfig.HydraulicActuator.ReliefValve.slope_coefficient', index=1,
      number=2, type=1, cpp_type=5, label=1,
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
  serialized_start=1870,
  serialized_end=1937,
)

_ACTUATIONCONFIG_HYDRAULICACTUATOR_CYLINDERPROPERTIES = _descriptor.Descriptor(
  name='CylinderProperties',
  full_name='simian_public.actuation_config.ActuationConfig.HydraulicActuator.CylinderProperties',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='pressurized_piston_area_extension', full_name='simian_public.actuation_config.ActuationConfig.HydraulicActuator.CylinderProperties.pressurized_piston_area_extension', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='pressurized_piston_area_retraction', full_name='simian_public.actuation_config.ActuationConfig.HydraulicActuator.CylinderProperties.pressurized_piston_area_retraction', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='piston_stroke_length', full_name='simian_public.actuation_config.ActuationConfig.HydraulicActuator.CylinderProperties.piston_stroke_length', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='constant_oil_volume_extension', full_name='simian_public.actuation_config.ActuationConfig.HydraulicActuator.CylinderProperties.constant_oil_volume_extension', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='constant_oil_volume_retraction', full_name='simian_public.actuation_config.ActuationConfig.HydraulicActuator.CylinderProperties.constant_oil_volume_retraction', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='minimum_mount_distance', full_name='simian_public.actuation_config.ActuationConfig.HydraulicActuator.CylinderProperties.minimum_mount_distance', index=5,
      number=6, type=1, cpp_type=5, label=1,
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
  serialized_start=1940,
  serialized_end=2188,
)

_ACTUATIONCONFIG_HYDRAULICACTUATOR = _descriptor.Descriptor(
  name='HydraulicActuator',
  full_name='simian_public.actuation_config.ActuationConfig.HydraulicActuator',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='coordinate', full_name='simian_public.actuation_config.ActuationConfig.HydraulicActuator.coordinate', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='oil_properties', full_name='simian_public.actuation_config.ActuationConfig.HydraulicActuator.oil_properties', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='mechanical_properties', full_name='simian_public.actuation_config.ActuationConfig.HydraulicActuator.mechanical_properties', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='control_valve', full_name='simian_public.actuation_config.ActuationConfig.HydraulicActuator.control_valve', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='hydraulic_pressure', full_name='simian_public.actuation_config.ActuationConfig.HydraulicActuator.hydraulic_pressure', index=4,
      number=5, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='relief_valve', full_name='simian_public.actuation_config.ActuationConfig.HydraulicActuator.relief_valve', index=5,
      number=6, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='cylinder_stop_stiffness', full_name='simian_public.actuation_config.ActuationConfig.HydraulicActuator.cylinder_stop_stiffness', index=6,
      number=7, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='cylinder_properties', full_name='simian_public.actuation_config.ActuationConfig.HydraulicActuator.cylinder_properties', index=7,
      number=8, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[_ACTUATIONCONFIG_HYDRAULICACTUATOR_COORDINATE, _ACTUATIONCONFIG_HYDRAULICACTUATOR_OILPROPERTIES, _ACTUATIONCONFIG_HYDRAULICACTUATOR_MECHANICALPROPERTIES, _ACTUATIONCONFIG_HYDRAULICACTUATOR_CONTROLVALVE, _ACTUATIONCONFIG_HYDRAULICACTUATOR_HYDRAULICPRESSURE, _ACTUATIONCONFIG_HYDRAULICACTUATOR_RELIEFVALVE, _ACTUATIONCONFIG_HYDRAULICACTUATOR_CYLINDERPROPERTIES, ],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
    _descriptor.OneofDescriptor(
      name='attachment', full_name='simian_public.actuation_config.ActuationConfig.HydraulicActuator.attachment',
      index=0, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
  ],
  serialized_start=667,
  serialized_end=2202,
)

_ACTUATIONCONFIG = _descriptor.Descriptor(
  name='ActuationConfig',
  full_name='simian_public.actuation_config.ActuationConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='actuators', full_name='simian_public.actuation_config.ActuationConfig.actuators', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[_ACTUATIONCONFIG_ACTUATOR, _ACTUATIONCONFIG_SPEEDSTATEMACHINEACTUATOR, _ACTUATIONCONFIG_HYDRAULICACTUATOR, ],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=79,
  serialized_end=2202,
)

_ACTUATIONCONFIG_ACTUATOR.fields_by_name['hydraulic'].message_type = _ACTUATIONCONFIG_HYDRAULICACTUATOR
_ACTUATIONCONFIG_ACTUATOR.fields_by_name['speed_state_machine'].message_type = _ACTUATIONCONFIG_SPEEDSTATEMACHINEACTUATOR
_ACTUATIONCONFIG_ACTUATOR.containing_type = _ACTUATIONCONFIG
_ACTUATIONCONFIG_ACTUATOR.oneofs_by_name['actuator_type'].fields.append(
  _ACTUATIONCONFIG_ACTUATOR.fields_by_name['hydraulic'])
_ACTUATIONCONFIG_ACTUATOR.fields_by_name['hydraulic'].containing_oneof = _ACTUATIONCONFIG_ACTUATOR.oneofs_by_name['actuator_type']
_ACTUATIONCONFIG_ACTUATOR.oneofs_by_name['actuator_type'].fields.append(
  _ACTUATIONCONFIG_ACTUATOR.fields_by_name['speed_state_machine'])
_ACTUATIONCONFIG_ACTUATOR.fields_by_name['speed_state_machine'].containing_oneof = _ACTUATIONCONFIG_ACTUATOR.oneofs_by_name['actuator_type']
_ACTUATIONCONFIG_SPEEDSTATEMACHINEACTUATOR.containing_type = _ACTUATIONCONFIG
_ACTUATIONCONFIG_HYDRAULICACTUATOR_COORDINATE.containing_type = _ACTUATIONCONFIG_HYDRAULICACTUATOR
_ACTUATIONCONFIG_HYDRAULICACTUATOR_OILPROPERTIES.containing_type = _ACTUATIONCONFIG_HYDRAULICACTUATOR
_ACTUATIONCONFIG_HYDRAULICACTUATOR_MECHANICALPROPERTIES.containing_type = _ACTUATIONCONFIG_HYDRAULICACTUATOR
_ACTUATIONCONFIG_HYDRAULICACTUATOR_CONTROLVALVE.containing_type = _ACTUATIONCONFIG_HYDRAULICACTUATOR
_ACTUATIONCONFIG_HYDRAULICACTUATOR_HYDRAULICPRESSURE.containing_type = _ACTUATIONCONFIG_HYDRAULICACTUATOR
_ACTUATIONCONFIG_HYDRAULICACTUATOR_RELIEFVALVE.containing_type = _ACTUATIONCONFIG_HYDRAULICACTUATOR
_ACTUATIONCONFIG_HYDRAULICACTUATOR_CYLINDERPROPERTIES.containing_type = _ACTUATIONCONFIG_HYDRAULICACTUATOR
_ACTUATIONCONFIG_HYDRAULICACTUATOR.fields_by_name['coordinate'].message_type = _ACTUATIONCONFIG_HYDRAULICACTUATOR_COORDINATE
_ACTUATIONCONFIG_HYDRAULICACTUATOR.fields_by_name['oil_properties'].message_type = _ACTUATIONCONFIG_HYDRAULICACTUATOR_OILPROPERTIES
_ACTUATIONCONFIG_HYDRAULICACTUATOR.fields_by_name['mechanical_properties'].message_type = _ACTUATIONCONFIG_HYDRAULICACTUATOR_MECHANICALPROPERTIES
_ACTUATIONCONFIG_HYDRAULICACTUATOR.fields_by_name['control_valve'].message_type = _ACTUATIONCONFIG_HYDRAULICACTUATOR_CONTROLVALVE
_ACTUATIONCONFIG_HYDRAULICACTUATOR.fields_by_name['hydraulic_pressure'].message_type = _ACTUATIONCONFIG_HYDRAULICACTUATOR_HYDRAULICPRESSURE
_ACTUATIONCONFIG_HYDRAULICACTUATOR.fields_by_name['relief_valve'].message_type = _ACTUATIONCONFIG_HYDRAULICACTUATOR_RELIEFVALVE
_ACTUATIONCONFIG_HYDRAULICACTUATOR.fields_by_name['cylinder_properties'].message_type = _ACTUATIONCONFIG_HYDRAULICACTUATOR_CYLINDERPROPERTIES
_ACTUATIONCONFIG_HYDRAULICACTUATOR.containing_type = _ACTUATIONCONFIG
_ACTUATIONCONFIG_HYDRAULICACTUATOR.oneofs_by_name['attachment'].fields.append(
  _ACTUATIONCONFIG_HYDRAULICACTUATOR.fields_by_name['coordinate'])
_ACTUATIONCONFIG_HYDRAULICACTUATOR.fields_by_name['coordinate'].containing_oneof = _ACTUATIONCONFIG_HYDRAULICACTUATOR.oneofs_by_name['attachment']
_ACTUATIONCONFIG.fields_by_name['actuators'].message_type = _ACTUATIONCONFIG_ACTUATOR
DESCRIPTOR.message_types_by_name['ActuationConfig'] = _ACTUATIONCONFIG
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

ActuationConfig = _reflection.GeneratedProtocolMessageType('ActuationConfig', (_message.Message,), {

  'Actuator' : _reflection.GeneratedProtocolMessageType('Actuator', (_message.Message,), {
    'DESCRIPTOR' : _ACTUATIONCONFIG_ACTUATOR,
    '__module__' : 'simian.public.proto.actuation_config_pb2'
    # @@protoc_insertion_point(class_scope:simian_public.actuation_config.ActuationConfig.Actuator)
    })
  ,

  'SpeedStateMachineActuator' : _reflection.GeneratedProtocolMessageType('SpeedStateMachineActuator', (_message.Message,), {
    'DESCRIPTOR' : _ACTUATIONCONFIG_SPEEDSTATEMACHINEACTUATOR,
    '__module__' : 'simian.public.proto.actuation_config_pb2'
    # @@protoc_insertion_point(class_scope:simian_public.actuation_config.ActuationConfig.SpeedStateMachineActuator)
    })
  ,

  'HydraulicActuator' : _reflection.GeneratedProtocolMessageType('HydraulicActuator', (_message.Message,), {

    'Coordinate' : _reflection.GeneratedProtocolMessageType('Coordinate', (_message.Message,), {
      'DESCRIPTOR' : _ACTUATIONCONFIG_HYDRAULICACTUATOR_COORDINATE,
      '__module__' : 'simian.public.proto.actuation_config_pb2'
      # @@protoc_insertion_point(class_scope:simian_public.actuation_config.ActuationConfig.HydraulicActuator.Coordinate)
      })
    ,

    'OilProperties' : _reflection.GeneratedProtocolMessageType('OilProperties', (_message.Message,), {
      'DESCRIPTOR' : _ACTUATIONCONFIG_HYDRAULICACTUATOR_OILPROPERTIES,
      '__module__' : 'simian.public.proto.actuation_config_pb2'
      # @@protoc_insertion_point(class_scope:simian_public.actuation_config.ActuationConfig.HydraulicActuator.OilProperties)
      })
    ,

    'MechanicalProperties' : _reflection.GeneratedProtocolMessageType('MechanicalProperties', (_message.Message,), {
      'DESCRIPTOR' : _ACTUATIONCONFIG_HYDRAULICACTUATOR_MECHANICALPROPERTIES,
      '__module__' : 'simian.public.proto.actuation_config_pb2'
      # @@protoc_insertion_point(class_scope:simian_public.actuation_config.ActuationConfig.HydraulicActuator.MechanicalProperties)
      })
    ,

    'ControlValve' : _reflection.GeneratedProtocolMessageType('ControlValve', (_message.Message,), {
      'DESCRIPTOR' : _ACTUATIONCONFIG_HYDRAULICACTUATOR_CONTROLVALVE,
      '__module__' : 'simian.public.proto.actuation_config_pb2'
      # @@protoc_insertion_point(class_scope:simian_public.actuation_config.ActuationConfig.HydraulicActuator.ControlValve)
      })
    ,

    'HydraulicPressure' : _reflection.GeneratedProtocolMessageType('HydraulicPressure', (_message.Message,), {
      'DESCRIPTOR' : _ACTUATIONCONFIG_HYDRAULICACTUATOR_HYDRAULICPRESSURE,
      '__module__' : 'simian.public.proto.actuation_config_pb2'
      # @@protoc_insertion_point(class_scope:simian_public.actuation_config.ActuationConfig.HydraulicActuator.HydraulicPressure)
      })
    ,

    'ReliefValve' : _reflection.GeneratedProtocolMessageType('ReliefValve', (_message.Message,), {
      'DESCRIPTOR' : _ACTUATIONCONFIG_HYDRAULICACTUATOR_RELIEFVALVE,
      '__module__' : 'simian.public.proto.actuation_config_pb2'
      # @@protoc_insertion_point(class_scope:simian_public.actuation_config.ActuationConfig.HydraulicActuator.ReliefValve)
      })
    ,

    'CylinderProperties' : _reflection.GeneratedProtocolMessageType('CylinderProperties', (_message.Message,), {
      'DESCRIPTOR' : _ACTUATIONCONFIG_HYDRAULICACTUATOR_CYLINDERPROPERTIES,
      '__module__' : 'simian.public.proto.actuation_config_pb2'
      # @@protoc_insertion_point(class_scope:simian_public.actuation_config.ActuationConfig.HydraulicActuator.CylinderProperties)
      })
    ,
    'DESCRIPTOR' : _ACTUATIONCONFIG_HYDRAULICACTUATOR,
    '__module__' : 'simian.public.proto.actuation_config_pb2'
    # @@protoc_insertion_point(class_scope:simian_public.actuation_config.ActuationConfig.HydraulicActuator)
    })
  ,
  'DESCRIPTOR' : _ACTUATIONCONFIG,
  '__module__' : 'simian.public.proto.actuation_config_pb2'
  # @@protoc_insertion_point(class_scope:simian_public.actuation_config.ActuationConfig)
  })
_sym_db.RegisterMessage(ActuationConfig)
_sym_db.RegisterMessage(ActuationConfig.Actuator)
_sym_db.RegisterMessage(ActuationConfig.SpeedStateMachineActuator)
_sym_db.RegisterMessage(ActuationConfig.HydraulicActuator)
_sym_db.RegisterMessage(ActuationConfig.HydraulicActuator.Coordinate)
_sym_db.RegisterMessage(ActuationConfig.HydraulicActuator.OilProperties)
_sym_db.RegisterMessage(ActuationConfig.HydraulicActuator.MechanicalProperties)
_sym_db.RegisterMessage(ActuationConfig.HydraulicActuator.ControlValve)
_sym_db.RegisterMessage(ActuationConfig.HydraulicActuator.HydraulicPressure)
_sym_db.RegisterMessage(ActuationConfig.HydraulicActuator.ReliefValve)
_sym_db.RegisterMessage(ActuationConfig.HydraulicActuator.CylinderProperties)


# @@protoc_insertion_point(module_scope)
