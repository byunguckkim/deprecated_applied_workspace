# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: simian/public/proto/map/map_parking_space.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from simian.public.proto.map import map_geometry_pb2 as simian_dot_public_dot_proto_dot_map_dot_map__geometry__pb2
from simian.public.proto.map import map_id_pb2 as simian_dot_public_dot_proto_dot_map_dot_map__id__pb2


# @@protoc_insertion_point(post_imports)

_sym_db = _symbol_database.Default()


DESCRIPTOR = _descriptor.FileDescriptor(
  name='simian/public/proto/map/map_parking_space.proto',
  package='simian_public.hdmap',
  syntax='proto2',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n/simian/public/proto/map/map_parking_space.proto\x12\x13simian_public.hdmap\x1a*simian/public/proto/map/map_geometry.proto\x1a$simian/public/proto/map/map_id.proto\"\x85\x01\n\x0cParkingSpace\x12#\n\x02id\x18\x01 \x01(\x0b\x32\x17.simian_public.hdmap.Id\x12-\n\x07polygon\x18\x02 \x01(\x0b\x32\x1c.simian_public.hdmap.Polygon\x12\x0f\n\x07heading\x18\x04 \x01(\x01J\x04\x08\x03\x10\x04R\noverlap_id\"r\n\nParkingLot\x12#\n\x02id\x18\x01 \x01(\x0b\x32\x17.simian_public.hdmap.Id\x12-\n\x07polygon\x18\x02 \x01(\x0b\x32\x1c.simian_public.hdmap.PolygonJ\x04\x08\x03\x10\x04R\noverlap_id'
  ,
  dependencies=[simian_dot_public_dot_proto_dot_map_dot_map__geometry__pb2.DESCRIPTOR,simian_dot_public_dot_proto_dot_map_dot_map__id__pb2.DESCRIPTOR,])




_PARKINGSPACE = _descriptor.Descriptor(
  name='ParkingSpace',
  full_name='simian_public.hdmap.ParkingSpace',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='simian_public.hdmap.ParkingSpace.id', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='polygon', full_name='simian_public.hdmap.ParkingSpace.polygon', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='heading', full_name='simian_public.hdmap.ParkingSpace.heading', index=2,
      number=4, type=1, cpp_type=5, label=1,
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
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=155,
  serialized_end=288,
)


_PARKINGLOT = _descriptor.Descriptor(
  name='ParkingLot',
  full_name='simian_public.hdmap.ParkingLot',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='simian_public.hdmap.ParkingLot.id', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='polygon', full_name='simian_public.hdmap.ParkingLot.polygon', index=1,
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
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=290,
  serialized_end=404,
)

_PARKINGSPACE.fields_by_name['id'].message_type = simian_dot_public_dot_proto_dot_map_dot_map__id__pb2._ID
_PARKINGSPACE.fields_by_name['polygon'].message_type = simian_dot_public_dot_proto_dot_map_dot_map__geometry__pb2._POLYGON
_PARKINGLOT.fields_by_name['id'].message_type = simian_dot_public_dot_proto_dot_map_dot_map__id__pb2._ID
_PARKINGLOT.fields_by_name['polygon'].message_type = simian_dot_public_dot_proto_dot_map_dot_map__geometry__pb2._POLYGON
DESCRIPTOR.message_types_by_name['ParkingSpace'] = _PARKINGSPACE
DESCRIPTOR.message_types_by_name['ParkingLot'] = _PARKINGLOT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

ParkingSpace = _reflection.GeneratedProtocolMessageType('ParkingSpace', (_message.Message,), {
  'DESCRIPTOR' : _PARKINGSPACE,
  '__module__' : 'simian.public.proto.map.map_parking_space_pb2'
  # @@protoc_insertion_point(class_scope:simian_public.hdmap.ParkingSpace)
  })
_sym_db.RegisterMessage(ParkingSpace)

ParkingLot = _reflection.GeneratedProtocolMessageType('ParkingLot', (_message.Message,), {
  'DESCRIPTOR' : _PARKINGLOT,
  '__module__' : 'simian.public.proto.map.map_parking_space_pb2'
  # @@protoc_insertion_point(class_scope:simian_public.hdmap.ParkingLot)
  })
_sym_db.RegisterMessage(ParkingLot)


# @@protoc_insertion_point(module_scope)
