# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: simian/public/proto/map/map_region.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from simian.public.proto import common_pb2 as simian_dot_public_dot_proto_dot_common__pb2
from simian.public.proto.map import map_common_pb2 as simian_dot_public_dot_proto_dot_map_dot_map__common__pb2
from simian.public.proto.map import map_geometry_pb2 as simian_dot_public_dot_proto_dot_map_dot_map__geometry__pb2
from simian.public.proto.map import map_id_pb2 as simian_dot_public_dot_proto_dot_map_dot_map__id__pb2
from simian.public.proto.map import map_validation_pb2 as simian_dot_public_dot_proto_dot_map_dot_map__validation__pb2


# @@protoc_insertion_point(post_imports)

_sym_db = _symbol_database.Default()


DESCRIPTOR = _descriptor.FileDescriptor(
  name='simian/public/proto/map/map_region.proto',
  package='simian_public.hdmap',
  syntax='proto2',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n(simian/public/proto/map/map_region.proto\x12\x13simian_public.hdmap\x1a simian/public/proto/common.proto\x1a(simian/public/proto/map/map_common.proto\x1a*simian/public/proto/map/map_geometry.proto\x1a$simian/public/proto/map/map_id.proto\x1a,simian/public/proto/map/map_validation.proto\"\xf9\x07\n\x06Region\x12#\n\x02id\x18\x01 \x01(\x0b\x32\x17.simian_public.hdmap.Id\x12-\n\x07polygon\x18\x02 \x01(\x0b\x32\x1c.simian_public.hdmap.Polygon\x12\x30\n\x0f\x61ssociated_lane\x18\x05 \x03(\x0b\x32\x17.simian_public.hdmap.Id\x12\x34\n\x04type\x18\x03 \x01(\x0e\x32&.simian_public.hdmap.Region.RegionType\x12\x13\n\x0b\x63ustom_type\x18\x04 \x01(\t\x12I\n\x15parking_space_details\x18\x1f \x01(\x0b\x32(.simian_public.hdmap.ParkingSpaceDetailsH\x00\x12@\n\x10\x62uilding_details\x18  \x01(\x0b\x32$.simian_public.hdmap.BuildingDetailsH\x00\x12-\n\x0csuccessor_id\x18\x06 \x03(\x0b\x32\x17.simian_public.hdmap.Id\x12/\n\x0epredecessor_id\x18\x07 \x03(\x0b\x32\x17.simian_public.hdmap.Id\x12\x13\n\x0bspeed_limit\x18\x08 \x01(\x01\x12\x11\n\x07heading\x18\x1d \x01(\x01H\x01\x12\x33\n\x0b\x63ustom_tags\x18\x1c \x03(\x0b\x32\x1e.simian_public.hdmap.CustomTag\x12\x41\n\x13\x63onversion_failures\x18\x1e \x03(\x0b\x32$.simian_public.hdmap.ValidationCheck\"\xb4\x02\n\nRegionType\x12\x0b\n\x07GENERIC\x10\x00\x12\n\n\x06\x43USTOM\x10\x01\x12\x0e\n\nCLEAR_AREA\x10\x02\x12\r\n\tCROSSWALK\x10\x03\x12\x0c\n\x08JUNCTION\x10\x04\x12\x11\n\rPARKING_SPACE\x10\x05\x12\x0f\n\x0bPARKING_LOT\x10\x06\x12\x0c\n\x08SIDEWALK\x10\x07\x12\x17\n\x13TRAVERSABLE_SURFACE\x10\x08\x12\x0e\n\nNO_GO_ZONE\x10\t\x12\x12\n\x0ePICKUP_DROPOFF\x10\n\x12\x0e\n\nSPEED_BUMP\x10\x0b\x12\x07\n\x03\x44IP\x10\x0c\x12\x0f\n\x0bSTOP_REGION\x10\r\x12\x10\n\x0cROAD_SURFACE\x10\x0e\x12\r\n\tROAD_MARK\x10\x0f\x12\x18\n\x14TRAFFIC_LIGHT_REGION\x10\x10\x12\x0c\n\x08\x42UILDING\x10\x11\x42\x10\n\x0eregion_detailsB\x16\n\x14semantic_orientationJ\x04\x08\x1a\x10\x1bJ\x04\x08\x1b\x10\x1cR\x10\x63onversion_errorR\x12\x63onversion_warning\"\xce\x04\n\x13ParkingSpaceDetails\x12X\n\rlocation_type\x18\x01 \x01(\x0e\x32\x41.simian_public.hdmap.ParkingSpaceDetails.ParkingSpaceLocationType\x12P\n\tedge_type\x18\x02 \x01(\x0e\x32=.simian_public.hdmap.ParkingSpaceDetails.ParkingSpaceEdgeType\x12?\n\x1bparking_space_marking_color\x18\x03 \x01(\x0b\x32\x1a.simian_public.common.RGBA\x12?\n\x1bparking_space_surface_color\x18\x04 \x01(\x0b\x32\x1a.simian_public.common.RGBA\"a\n\x18ParkingSpaceLocationType\x12\x12\n\x0eUNSET_LOCATION\x10\x00\x12\t\n\x05STALL\x10\x01\x12\x0c\n\x08\x44RIVEWAY\x10\x02\x12\x0c\n\x08ROADSIDE\x10\x03\x12\n\n\x06ONROAD\x10\x04\"\xa5\x01\n\x14ParkingSpaceEdgeType\x12\x13\n\x0fUNSET_EDGE_TYPE\x10\x00\x12\x12\n\x0eUNPAINTED_EDGE\x10\x01\x12\x14\n\x10ROAD_EDGE_DASHED\x10\x02\x12\x13\n\x0fROAD_EDGE_SOLID\x10\x03\x12\x0f\n\x0b\x43ORNER_EDGE\x10\x04\x12\x17\n\x13\x43ORNER_TO_CURB_EDGE\x10\x05\x12\x0f\n\x0bTHREE_EDGES\x10\x06\"!\n\x0f\x42uildingDetails\x12\x0e\n\x06height\x18\x01 \x01(\x02'
  ,
  dependencies=[simian_dot_public_dot_proto_dot_common__pb2.DESCRIPTOR,simian_dot_public_dot_proto_dot_map_dot_map__common__pb2.DESCRIPTOR,simian_dot_public_dot_proto_dot_map_dot_map__geometry__pb2.DESCRIPTOR,simian_dot_public_dot_proto_dot_map_dot_map__id__pb2.DESCRIPTOR,simian_dot_public_dot_proto_dot_map_dot_map__validation__pb2.DESCRIPTOR,])



_REGION_REGIONTYPE = _descriptor.EnumDescriptor(
  name='RegionType',
  full_name='simian_public.hdmap.Region.RegionType',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='GENERIC', index=0, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='CUSTOM', index=1, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='CLEAR_AREA', index=2, number=2,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='CROSSWALK', index=3, number=3,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='JUNCTION', index=4, number=4,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='PARKING_SPACE', index=5, number=5,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='PARKING_LOT', index=6, number=6,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='SIDEWALK', index=7, number=7,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='TRAVERSABLE_SURFACE', index=8, number=8,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='NO_GO_ZONE', index=9, number=9,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='PICKUP_DROPOFF', index=10, number=10,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='SPEED_BUMP', index=11, number=11,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='DIP', index=12, number=12,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='STOP_REGION', index=13, number=13,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='ROAD_SURFACE', index=14, number=14,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='ROAD_MARK', index=15, number=15,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='TRAFFIC_LIGHT_REGION', index=16, number=16,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='BUILDING', index=17, number=17,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=887,
  serialized_end=1195,
)
_sym_db.RegisterEnumDescriptor(_REGION_REGIONTYPE)

_PARKINGSPACEDETAILS_PARKINGSPACELOCATIONTYPE = _descriptor.EnumDescriptor(
  name='ParkingSpaceLocationType',
  full_name='simian_public.hdmap.ParkingSpaceDetails.ParkingSpaceLocationType',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='UNSET_LOCATION', index=0, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='STALL', index=1, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='DRIVEWAY', index=2, number=2,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='ROADSIDE', index=3, number=3,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='ONROAD', index=4, number=4,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=1615,
  serialized_end=1712,
)
_sym_db.RegisterEnumDescriptor(_PARKINGSPACEDETAILS_PARKINGSPACELOCATIONTYPE)

_PARKINGSPACEDETAILS_PARKINGSPACEEDGETYPE = _descriptor.EnumDescriptor(
  name='ParkingSpaceEdgeType',
  full_name='simian_public.hdmap.ParkingSpaceDetails.ParkingSpaceEdgeType',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='UNSET_EDGE_TYPE', index=0, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='UNPAINTED_EDGE', index=1, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='ROAD_EDGE_DASHED', index=2, number=2,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='ROAD_EDGE_SOLID', index=3, number=3,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='CORNER_EDGE', index=4, number=4,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='CORNER_TO_CURB_EDGE', index=5, number=5,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='THREE_EDGES', index=6, number=6,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=1715,
  serialized_end=1880,
)
_sym_db.RegisterEnumDescriptor(_PARKINGSPACEDETAILS_PARKINGSPACEEDGETYPE)


_REGION = _descriptor.Descriptor(
  name='Region',
  full_name='simian_public.hdmap.Region',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='simian_public.hdmap.Region.id', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='polygon', full_name='simian_public.hdmap.Region.polygon', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='associated_lane', full_name='simian_public.hdmap.Region.associated_lane', index=2,
      number=5, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='type', full_name='simian_public.hdmap.Region.type', index=3,
      number=3, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='custom_type', full_name='simian_public.hdmap.Region.custom_type', index=4,
      number=4, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='parking_space_details', full_name='simian_public.hdmap.Region.parking_space_details', index=5,
      number=31, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='building_details', full_name='simian_public.hdmap.Region.building_details', index=6,
      number=32, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='successor_id', full_name='simian_public.hdmap.Region.successor_id', index=7,
      number=6, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='predecessor_id', full_name='simian_public.hdmap.Region.predecessor_id', index=8,
      number=7, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='speed_limit', full_name='simian_public.hdmap.Region.speed_limit', index=9,
      number=8, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='heading', full_name='simian_public.hdmap.Region.heading', index=10,
      number=29, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='custom_tags', full_name='simian_public.hdmap.Region.custom_tags', index=11,
      number=28, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='conversion_failures', full_name='simian_public.hdmap.Region.conversion_failures', index=12,
      number=30, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _REGION_REGIONTYPE,
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
    _descriptor.OneofDescriptor(
      name='region_details', full_name='simian_public.hdmap.Region.region_details',
      index=0, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
    _descriptor.OneofDescriptor(
      name='semantic_orientation', full_name='simian_public.hdmap.Region.semantic_orientation',
      index=1, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
  ],
  serialized_start=270,
  serialized_end=1287,
)


_PARKINGSPACEDETAILS = _descriptor.Descriptor(
  name='ParkingSpaceDetails',
  full_name='simian_public.hdmap.ParkingSpaceDetails',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='location_type', full_name='simian_public.hdmap.ParkingSpaceDetails.location_type', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='edge_type', full_name='simian_public.hdmap.ParkingSpaceDetails.edge_type', index=1,
      number=2, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='parking_space_marking_color', full_name='simian_public.hdmap.ParkingSpaceDetails.parking_space_marking_color', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='parking_space_surface_color', full_name='simian_public.hdmap.ParkingSpaceDetails.parking_space_surface_color', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _PARKINGSPACEDETAILS_PARKINGSPACELOCATIONTYPE,
    _PARKINGSPACEDETAILS_PARKINGSPACEEDGETYPE,
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=1290,
  serialized_end=1880,
)


_BUILDINGDETAILS = _descriptor.Descriptor(
  name='BuildingDetails',
  full_name='simian_public.hdmap.BuildingDetails',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='height', full_name='simian_public.hdmap.BuildingDetails.height', index=0,
      number=1, type=2, cpp_type=6, label=1,
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
  serialized_start=1882,
  serialized_end=1915,
)

_REGION.fields_by_name['id'].message_type = simian_dot_public_dot_proto_dot_map_dot_map__id__pb2._ID
_REGION.fields_by_name['polygon'].message_type = simian_dot_public_dot_proto_dot_map_dot_map__geometry__pb2._POLYGON
_REGION.fields_by_name['associated_lane'].message_type = simian_dot_public_dot_proto_dot_map_dot_map__id__pb2._ID
_REGION.fields_by_name['type'].enum_type = _REGION_REGIONTYPE
_REGION.fields_by_name['parking_space_details'].message_type = _PARKINGSPACEDETAILS
_REGION.fields_by_name['building_details'].message_type = _BUILDINGDETAILS
_REGION.fields_by_name['successor_id'].message_type = simian_dot_public_dot_proto_dot_map_dot_map__id__pb2._ID
_REGION.fields_by_name['predecessor_id'].message_type = simian_dot_public_dot_proto_dot_map_dot_map__id__pb2._ID
_REGION.fields_by_name['custom_tags'].message_type = simian_dot_public_dot_proto_dot_map_dot_map__common__pb2._CUSTOMTAG
_REGION.fields_by_name['conversion_failures'].message_type = simian_dot_public_dot_proto_dot_map_dot_map__validation__pb2._VALIDATIONCHECK
_REGION_REGIONTYPE.containing_type = _REGION
_REGION.oneofs_by_name['region_details'].fields.append(
  _REGION.fields_by_name['parking_space_details'])
_REGION.fields_by_name['parking_space_details'].containing_oneof = _REGION.oneofs_by_name['region_details']
_REGION.oneofs_by_name['region_details'].fields.append(
  _REGION.fields_by_name['building_details'])
_REGION.fields_by_name['building_details'].containing_oneof = _REGION.oneofs_by_name['region_details']
_REGION.oneofs_by_name['semantic_orientation'].fields.append(
  _REGION.fields_by_name['heading'])
_REGION.fields_by_name['heading'].containing_oneof = _REGION.oneofs_by_name['semantic_orientation']
_PARKINGSPACEDETAILS.fields_by_name['location_type'].enum_type = _PARKINGSPACEDETAILS_PARKINGSPACELOCATIONTYPE
_PARKINGSPACEDETAILS.fields_by_name['edge_type'].enum_type = _PARKINGSPACEDETAILS_PARKINGSPACEEDGETYPE
_PARKINGSPACEDETAILS.fields_by_name['parking_space_marking_color'].message_type = simian_dot_public_dot_proto_dot_common__pb2._RGBA
_PARKINGSPACEDETAILS.fields_by_name['parking_space_surface_color'].message_type = simian_dot_public_dot_proto_dot_common__pb2._RGBA
_PARKINGSPACEDETAILS_PARKINGSPACELOCATIONTYPE.containing_type = _PARKINGSPACEDETAILS
_PARKINGSPACEDETAILS_PARKINGSPACEEDGETYPE.containing_type = _PARKINGSPACEDETAILS
DESCRIPTOR.message_types_by_name['Region'] = _REGION
DESCRIPTOR.message_types_by_name['ParkingSpaceDetails'] = _PARKINGSPACEDETAILS
DESCRIPTOR.message_types_by_name['BuildingDetails'] = _BUILDINGDETAILS
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Region = _reflection.GeneratedProtocolMessageType('Region', (_message.Message,), {
  'DESCRIPTOR' : _REGION,
  '__module__' : 'simian.public.proto.map.map_region_pb2'
  # @@protoc_insertion_point(class_scope:simian_public.hdmap.Region)
  })
_sym_db.RegisterMessage(Region)

ParkingSpaceDetails = _reflection.GeneratedProtocolMessageType('ParkingSpaceDetails', (_message.Message,), {
  'DESCRIPTOR' : _PARKINGSPACEDETAILS,
  '__module__' : 'simian.public.proto.map.map_region_pb2'
  # @@protoc_insertion_point(class_scope:simian_public.hdmap.ParkingSpaceDetails)
  })
_sym_db.RegisterMessage(ParkingSpaceDetails)

BuildingDetails = _reflection.GeneratedProtocolMessageType('BuildingDetails', (_message.Message,), {
  'DESCRIPTOR' : _BUILDINGDETAILS,
  '__module__' : 'simian.public.proto.map.map_region_pb2'
  # @@protoc_insertion_point(class_scope:simian_public.hdmap.BuildingDetails)
  })
_sym_db.RegisterMessage(BuildingDetails)


# @@protoc_insertion_point(module_scope)
