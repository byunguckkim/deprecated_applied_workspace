# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: simian/public/proto/map/map_road.proto
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
  name='simian/public/proto/map/map_road.proto',
  package='simian_public.hdmap',
  syntax='proto2',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n&simian/public/proto/map/map_road.proto\x12\x13simian_public.hdmap\x1a*simian/public/proto/map/map_geometry.proto\x1a$simian/public/proto/map/map_id.proto\"\xb7\x01\n\x0c\x42oundaryEdge\x12)\n\x05\x63urve\x18\x01 \x01(\x0b\x32\x1a.simian_public.hdmap.Curve\x12\x34\n\x04type\x18\x02 \x01(\x0e\x32&.simian_public.hdmap.BoundaryEdge.Type\"F\n\x04Type\x12\x0b\n\x07UNKNOWN\x10\x00\x12\n\n\x06NORMAL\x10\x01\x12\x11\n\rLEFT_BOUNDARY\x10\x02\x12\x12\n\x0eRIGHT_BOUNDARY\x10\x03\"B\n\x0f\x42oundaryPolygon\x12/\n\x04\x65\x64ge\x18\x01 \x03(\x0b\x32!.simian_public.hdmap.BoundaryEdge\"\x7f\n\x0cRoadBoundary\x12;\n\router_polygon\x18\x01 \x01(\x0b\x32$.simian_public.hdmap.BoundaryPolygon\x12\x32\n\x04hole\x18\x02 \x03(\x0b\x32$.simian_public.hdmap.BoundaryPolygon\"\xf1\x01\n\x0bRoadSection\x12#\n\x02id\x18\x01 \x01(\x0b\x32\x17.simian_public.hdmap.Id\x12(\n\x07lane_id\x18\x02 \x03(\x0b\x32\x17.simian_public.hdmap.Id\x12\x33\n\x08\x62oundary\x18\x03 \x01(\x0b\x32!.simian_public.hdmap.RoadBoundary\x12/\n\x0epredecessor_id\x18\x04 \x03(\x0b\x32\x17.simian_public.hdmap.Id\x12-\n\x0csuccessor_id\x18\x05 \x03(\x0b\x32\x17.simian_public.hdmap.Id\"\xd5\x02\n\x04Road\x12#\n\x02id\x18\x01 \x01(\x0b\x32\x17.simian_public.hdmap.Id\x12\x31\n\x07section\x18\x02 \x03(\x0b\x32 .simian_public.hdmap.RoadSection\x12,\n\x0bjunction_id\x18\x03 \x01(\x0b\x32\x17.simian_public.hdmap.Id\x12,\n\x04type\x18\x04 \x01(\x0e\x32\x1e.simian_public.hdmap.Road.Type\x12/\n\x0epredecessor_id\x18\x05 \x03(\x0b\x32\x17.simian_public.hdmap.Id\x12-\n\x0csuccessor_id\x18\x06 \x03(\x0b\x32\x17.simian_public.hdmap.Id\"9\n\x04Type\x12\x0b\n\x07UNKNOWN\x10\x00\x12\x0b\n\x07HIGHWAY\x10\x01\x12\r\n\tCITY_ROAD\x10\x02\x12\x08\n\x04PARK\x10\x03'
  ,
  dependencies=[simian_dot_public_dot_proto_dot_map_dot_map__geometry__pb2.DESCRIPTOR,simian_dot_public_dot_proto_dot_map_dot_map__id__pb2.DESCRIPTOR,])



_BOUNDARYEDGE_TYPE = _descriptor.EnumDescriptor(
  name='Type',
  full_name='simian_public.hdmap.BoundaryEdge.Type',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='UNKNOWN', index=0, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='NORMAL', index=1, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='LEFT_BOUNDARY', index=2, number=2,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='RIGHT_BOUNDARY', index=3, number=3,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=259,
  serialized_end=329,
)
_sym_db.RegisterEnumDescriptor(_BOUNDARYEDGE_TYPE)

_ROAD_TYPE = _descriptor.EnumDescriptor(
  name='Type',
  full_name='simian_public.hdmap.Road.Type',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='UNKNOWN', index=0, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='HIGHWAY', index=1, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='CITY_ROAD', index=2, number=2,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='PARK', index=3, number=3,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=1057,
  serialized_end=1114,
)
_sym_db.RegisterEnumDescriptor(_ROAD_TYPE)


_BOUNDARYEDGE = _descriptor.Descriptor(
  name='BoundaryEdge',
  full_name='simian_public.hdmap.BoundaryEdge',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='curve', full_name='simian_public.hdmap.BoundaryEdge.curve', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='type', full_name='simian_public.hdmap.BoundaryEdge.type', index=1,
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
    _BOUNDARYEDGE_TYPE,
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=146,
  serialized_end=329,
)


_BOUNDARYPOLYGON = _descriptor.Descriptor(
  name='BoundaryPolygon',
  full_name='simian_public.hdmap.BoundaryPolygon',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='edge', full_name='simian_public.hdmap.BoundaryPolygon.edge', index=0,
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
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=331,
  serialized_end=397,
)


_ROADBOUNDARY = _descriptor.Descriptor(
  name='RoadBoundary',
  full_name='simian_public.hdmap.RoadBoundary',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='outer_polygon', full_name='simian_public.hdmap.RoadBoundary.outer_polygon', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='hole', full_name='simian_public.hdmap.RoadBoundary.hole', index=1,
      number=2, type=11, cpp_type=10, label=3,
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
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=399,
  serialized_end=526,
)


_ROADSECTION = _descriptor.Descriptor(
  name='RoadSection',
  full_name='simian_public.hdmap.RoadSection',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='simian_public.hdmap.RoadSection.id', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='lane_id', full_name='simian_public.hdmap.RoadSection.lane_id', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='boundary', full_name='simian_public.hdmap.RoadSection.boundary', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='predecessor_id', full_name='simian_public.hdmap.RoadSection.predecessor_id', index=3,
      number=4, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='successor_id', full_name='simian_public.hdmap.RoadSection.successor_id', index=4,
      number=5, type=11, cpp_type=10, label=3,
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
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=529,
  serialized_end=770,
)


_ROAD = _descriptor.Descriptor(
  name='Road',
  full_name='simian_public.hdmap.Road',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='simian_public.hdmap.Road.id', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='section', full_name='simian_public.hdmap.Road.section', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='junction_id', full_name='simian_public.hdmap.Road.junction_id', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='type', full_name='simian_public.hdmap.Road.type', index=3,
      number=4, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='predecessor_id', full_name='simian_public.hdmap.Road.predecessor_id', index=4,
      number=5, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='successor_id', full_name='simian_public.hdmap.Road.successor_id', index=5,
      number=6, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _ROAD_TYPE,
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=773,
  serialized_end=1114,
)

_BOUNDARYEDGE.fields_by_name['curve'].message_type = simian_dot_public_dot_proto_dot_map_dot_map__geometry__pb2._CURVE
_BOUNDARYEDGE.fields_by_name['type'].enum_type = _BOUNDARYEDGE_TYPE
_BOUNDARYEDGE_TYPE.containing_type = _BOUNDARYEDGE
_BOUNDARYPOLYGON.fields_by_name['edge'].message_type = _BOUNDARYEDGE
_ROADBOUNDARY.fields_by_name['outer_polygon'].message_type = _BOUNDARYPOLYGON
_ROADBOUNDARY.fields_by_name['hole'].message_type = _BOUNDARYPOLYGON
_ROADSECTION.fields_by_name['id'].message_type = simian_dot_public_dot_proto_dot_map_dot_map__id__pb2._ID
_ROADSECTION.fields_by_name['lane_id'].message_type = simian_dot_public_dot_proto_dot_map_dot_map__id__pb2._ID
_ROADSECTION.fields_by_name['boundary'].message_type = _ROADBOUNDARY
_ROADSECTION.fields_by_name['predecessor_id'].message_type = simian_dot_public_dot_proto_dot_map_dot_map__id__pb2._ID
_ROADSECTION.fields_by_name['successor_id'].message_type = simian_dot_public_dot_proto_dot_map_dot_map__id__pb2._ID
_ROAD.fields_by_name['id'].message_type = simian_dot_public_dot_proto_dot_map_dot_map__id__pb2._ID
_ROAD.fields_by_name['section'].message_type = _ROADSECTION
_ROAD.fields_by_name['junction_id'].message_type = simian_dot_public_dot_proto_dot_map_dot_map__id__pb2._ID
_ROAD.fields_by_name['type'].enum_type = _ROAD_TYPE
_ROAD.fields_by_name['predecessor_id'].message_type = simian_dot_public_dot_proto_dot_map_dot_map__id__pb2._ID
_ROAD.fields_by_name['successor_id'].message_type = simian_dot_public_dot_proto_dot_map_dot_map__id__pb2._ID
_ROAD_TYPE.containing_type = _ROAD
DESCRIPTOR.message_types_by_name['BoundaryEdge'] = _BOUNDARYEDGE
DESCRIPTOR.message_types_by_name['BoundaryPolygon'] = _BOUNDARYPOLYGON
DESCRIPTOR.message_types_by_name['RoadBoundary'] = _ROADBOUNDARY
DESCRIPTOR.message_types_by_name['RoadSection'] = _ROADSECTION
DESCRIPTOR.message_types_by_name['Road'] = _ROAD
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

BoundaryEdge = _reflection.GeneratedProtocolMessageType('BoundaryEdge', (_message.Message,), {
  'DESCRIPTOR' : _BOUNDARYEDGE,
  '__module__' : 'simian.public.proto.map.map_road_pb2'
  # @@protoc_insertion_point(class_scope:simian_public.hdmap.BoundaryEdge)
  })
_sym_db.RegisterMessage(BoundaryEdge)

BoundaryPolygon = _reflection.GeneratedProtocolMessageType('BoundaryPolygon', (_message.Message,), {
  'DESCRIPTOR' : _BOUNDARYPOLYGON,
  '__module__' : 'simian.public.proto.map.map_road_pb2'
  # @@protoc_insertion_point(class_scope:simian_public.hdmap.BoundaryPolygon)
  })
_sym_db.RegisterMessage(BoundaryPolygon)

RoadBoundary = _reflection.GeneratedProtocolMessageType('RoadBoundary', (_message.Message,), {
  'DESCRIPTOR' : _ROADBOUNDARY,
  '__module__' : 'simian.public.proto.map.map_road_pb2'
  # @@protoc_insertion_point(class_scope:simian_public.hdmap.RoadBoundary)
  })
_sym_db.RegisterMessage(RoadBoundary)

RoadSection = _reflection.GeneratedProtocolMessageType('RoadSection', (_message.Message,), {
  'DESCRIPTOR' : _ROADSECTION,
  '__module__' : 'simian.public.proto.map.map_road_pb2'
  # @@protoc_insertion_point(class_scope:simian_public.hdmap.RoadSection)
  })
_sym_db.RegisterMessage(RoadSection)

Road = _reflection.GeneratedProtocolMessageType('Road', (_message.Message,), {
  'DESCRIPTOR' : _ROAD,
  '__module__' : 'simian.public.proto.map.map_road_pb2'
  # @@protoc_insertion_point(class_scope:simian_public.hdmap.Road)
  })
_sym_db.RegisterMessage(Road)


# @@protoc_insertion_point(module_scope)
