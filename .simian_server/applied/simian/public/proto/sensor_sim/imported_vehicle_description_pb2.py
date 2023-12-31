# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: simian/public/proto/sensor_sim/imported_vehicle_description.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from simian.public.proto.sensor_sim import mesh_description_pb2 as simian_dot_public_dot_proto_dot_sensor__sim_dot_mesh__description__pb2
from simian.public.proto import spatial_pb2 as simian_dot_public_dot_proto_dot_spatial__pb2


# @@protoc_insertion_point(post_imports)

_sym_db = _symbol_database.Default()


DESCRIPTOR = _descriptor.FileDescriptor(
  name='simian/public/proto/sensor_sim/imported_vehicle_description.proto',
  package='simian_public.sensor_sim.imported_vehicle_description',
  syntax='proto3',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\nAsimian/public/proto/sensor_sim/imported_vehicle_description.proto\x12\x35simian_public.sensor_sim.imported_vehicle_description\x1a\x35simian/public/proto/sensor_sim/mesh_description.proto\x1a!simian/public/proto/spatial.proto\"\x9b\x03\n\x1aImportedVehicleDescription\x12T\n\x10mesh_description\x18\x01 \x01(\x0b\x32:.simian_public.sensor_sim.mesh_description.MeshDescription\x12i\n\x0b\x61ttachments\x18\x02 \x03(\x0b\x32T.simian_public.sensor_sim.imported_vehicle_description.ImportedAttachmentDescription\x12W\n\x06wheels\x18\x03 \x03(\x0b\x32G.simian_public.sensor_sim.imported_vehicle_description.WheelDescription\x12\x63\n\nwheel_type\x18\x04 \x01(\x0e\x32O.simian_public.sensor_sim.imported_vehicle_description.WheelTypeEnum.WheelTypes\"\xad\x01\n\x1dImportedAttachmentDescription\x12T\n\x10mesh_description\x18\x01 \x01(\x0b\x32:.simian_public.sensor_sim.mesh_description.MeshDescription\x12\x36\n\rrelative_pose\x18\x02 \x01(\x0b\x32\x1f.simian_public.spatial.PoseSpec\"\xa7\x01\n\x10WheelDescription\x12\x12\n\nwheel_name\x18\x01 \x01(\t\x12\x13\n\x0bwheel_scale\x18\x02 \x01(\x02\x12\x32\n\x0cinitial_pose\x18\x03 \x01(\x0b\x32\x1c.simian_public.spatial.Point\x12\x19\n\x11suspension_max_up\x18\x04 \x01(\x02\x12\x1b\n\x13suspension_max_down\x18\x05 \x01(\x02\"\x88\x02\n\rWheelTypeEnum\"\xf6\x01\n\nWheelTypes\x12\x0b\n\x07\x44\x45\x46\x41ULT\x10\x00\x12\x12\n\x0eONROAD_WHEEL_1\x10\x01\x12\x12\n\x0eONROAD_WHEEL_2\x10\x02\x12\x12\n\x0eONROAD_WHEEL_3\x10\x03\x12\x12\n\x0eONROAD_WHEEL_4\x10\x04\x12\x12\n\x0eONROAD_WHEEL_5\x10\x05\x12\x12\n\x0eONROAD_WHEEL_6\x10\x06\x12\x12\n\x0eSPOKED_WHEEL_1\x10\x07\x12\x13\n\x0fOFFROAD_WHEEL_1\x10\x08\x12\x1e\n\x1aSEMI_TRUCK_TRAILER_WHEEL_1\x10\t\x12\x1a\n\x16SEMI_TRUCK_CAB_WHEEL_1\x10\nb\x06proto3'
  ,
  dependencies=[simian_dot_public_dot_proto_dot_sensor__sim_dot_mesh__description__pb2.DESCRIPTOR,simian_dot_public_dot_proto_dot_spatial__pb2.DESCRIPTOR,])



_WHEELTYPEENUM_WHEELTYPES = _descriptor.EnumDescriptor(
  name='WheelTypes',
  full_name='simian_public.sensor_sim.imported_vehicle_description.WheelTypeEnum.WheelTypes',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='DEFAULT', index=0, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='ONROAD_WHEEL_1', index=1, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='ONROAD_WHEEL_2', index=2, number=2,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='ONROAD_WHEEL_3', index=3, number=3,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='ONROAD_WHEEL_4', index=4, number=4,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='ONROAD_WHEEL_5', index=5, number=5,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='ONROAD_WHEEL_6', index=6, number=6,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='SPOKED_WHEEL_1', index=7, number=7,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='OFFROAD_WHEEL_1', index=8, number=8,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='SEMI_TRUCK_TRAILER_WHEEL_1', index=9, number=9,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='SEMI_TRUCK_CAB_WHEEL_1', index=10, number=10,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=993,
  serialized_end=1239,
)
_sym_db.RegisterEnumDescriptor(_WHEELTYPEENUM_WHEELTYPES)


_IMPORTEDVEHICLEDESCRIPTION = _descriptor.Descriptor(
  name='ImportedVehicleDescription',
  full_name='simian_public.sensor_sim.imported_vehicle_description.ImportedVehicleDescription',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='mesh_description', full_name='simian_public.sensor_sim.imported_vehicle_description.ImportedVehicleDescription.mesh_description', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='attachments', full_name='simian_public.sensor_sim.imported_vehicle_description.ImportedVehicleDescription.attachments', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='wheels', full_name='simian_public.sensor_sim.imported_vehicle_description.ImportedVehicleDescription.wheels', index=2,
      number=3, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='wheel_type', full_name='simian_public.sensor_sim.imported_vehicle_description.ImportedVehicleDescription.wheel_type', index=3,
      number=4, type=14, cpp_type=8, label=1,
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
  serialized_start=215,
  serialized_end=626,
)


_IMPORTEDATTACHMENTDESCRIPTION = _descriptor.Descriptor(
  name='ImportedAttachmentDescription',
  full_name='simian_public.sensor_sim.imported_vehicle_description.ImportedAttachmentDescription',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='mesh_description', full_name='simian_public.sensor_sim.imported_vehicle_description.ImportedAttachmentDescription.mesh_description', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='relative_pose', full_name='simian_public.sensor_sim.imported_vehicle_description.ImportedAttachmentDescription.relative_pose', index=1,
      number=2, type=11, cpp_type=10, label=1,
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
  ],
  serialized_start=629,
  serialized_end=802,
)


_WHEELDESCRIPTION = _descriptor.Descriptor(
  name='WheelDescription',
  full_name='simian_public.sensor_sim.imported_vehicle_description.WheelDescription',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='wheel_name', full_name='simian_public.sensor_sim.imported_vehicle_description.WheelDescription.wheel_name', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='wheel_scale', full_name='simian_public.sensor_sim.imported_vehicle_description.WheelDescription.wheel_scale', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='initial_pose', full_name='simian_public.sensor_sim.imported_vehicle_description.WheelDescription.initial_pose', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='suspension_max_up', full_name='simian_public.sensor_sim.imported_vehicle_description.WheelDescription.suspension_max_up', index=3,
      number=4, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='suspension_max_down', full_name='simian_public.sensor_sim.imported_vehicle_description.WheelDescription.suspension_max_down', index=4,
      number=5, type=2, cpp_type=6, label=1,
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
  serialized_start=805,
  serialized_end=972,
)


_WHEELTYPEENUM = _descriptor.Descriptor(
  name='WheelTypeEnum',
  full_name='simian_public.sensor_sim.imported_vehicle_description.WheelTypeEnum',
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
    _WHEELTYPEENUM_WHEELTYPES,
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=975,
  serialized_end=1239,
)

_IMPORTEDVEHICLEDESCRIPTION.fields_by_name['mesh_description'].message_type = simian_dot_public_dot_proto_dot_sensor__sim_dot_mesh__description__pb2._MESHDESCRIPTION
_IMPORTEDVEHICLEDESCRIPTION.fields_by_name['attachments'].message_type = _IMPORTEDATTACHMENTDESCRIPTION
_IMPORTEDVEHICLEDESCRIPTION.fields_by_name['wheels'].message_type = _WHEELDESCRIPTION
_IMPORTEDVEHICLEDESCRIPTION.fields_by_name['wheel_type'].enum_type = _WHEELTYPEENUM_WHEELTYPES
_IMPORTEDATTACHMENTDESCRIPTION.fields_by_name['mesh_description'].message_type = simian_dot_public_dot_proto_dot_sensor__sim_dot_mesh__description__pb2._MESHDESCRIPTION
_IMPORTEDATTACHMENTDESCRIPTION.fields_by_name['relative_pose'].message_type = simian_dot_public_dot_proto_dot_spatial__pb2._POSESPEC
_WHEELDESCRIPTION.fields_by_name['initial_pose'].message_type = simian_dot_public_dot_proto_dot_spatial__pb2._POINT
_WHEELTYPEENUM_WHEELTYPES.containing_type = _WHEELTYPEENUM
DESCRIPTOR.message_types_by_name['ImportedVehicleDescription'] = _IMPORTEDVEHICLEDESCRIPTION
DESCRIPTOR.message_types_by_name['ImportedAttachmentDescription'] = _IMPORTEDATTACHMENTDESCRIPTION
DESCRIPTOR.message_types_by_name['WheelDescription'] = _WHEELDESCRIPTION
DESCRIPTOR.message_types_by_name['WheelTypeEnum'] = _WHEELTYPEENUM
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

ImportedVehicleDescription = _reflection.GeneratedProtocolMessageType('ImportedVehicleDescription', (_message.Message,), {
  'DESCRIPTOR' : _IMPORTEDVEHICLEDESCRIPTION,
  '__module__' : 'simian.public.proto.sensor_sim.imported_vehicle_description_pb2'
  # @@protoc_insertion_point(class_scope:simian_public.sensor_sim.imported_vehicle_description.ImportedVehicleDescription)
  })
_sym_db.RegisterMessage(ImportedVehicleDescription)

ImportedAttachmentDescription = _reflection.GeneratedProtocolMessageType('ImportedAttachmentDescription', (_message.Message,), {
  'DESCRIPTOR' : _IMPORTEDATTACHMENTDESCRIPTION,
  '__module__' : 'simian.public.proto.sensor_sim.imported_vehicle_description_pb2'
  # @@protoc_insertion_point(class_scope:simian_public.sensor_sim.imported_vehicle_description.ImportedAttachmentDescription)
  })
_sym_db.RegisterMessage(ImportedAttachmentDescription)

WheelDescription = _reflection.GeneratedProtocolMessageType('WheelDescription', (_message.Message,), {
  'DESCRIPTOR' : _WHEELDESCRIPTION,
  '__module__' : 'simian.public.proto.sensor_sim.imported_vehicle_description_pb2'
  # @@protoc_insertion_point(class_scope:simian_public.sensor_sim.imported_vehicle_description.WheelDescription)
  })
_sym_db.RegisterMessage(WheelDescription)

WheelTypeEnum = _reflection.GeneratedProtocolMessageType('WheelTypeEnum', (_message.Message,), {
  'DESCRIPTOR' : _WHEELTYPEENUM,
  '__module__' : 'simian.public.proto.sensor_sim.imported_vehicle_description_pb2'
  # @@protoc_insertion_point(class_scope:simian_public.sensor_sim.imported_vehicle_description.WheelTypeEnum)
  })
_sym_db.RegisterMessage(WheelTypeEnum)


# @@protoc_insertion_point(module_scope)
