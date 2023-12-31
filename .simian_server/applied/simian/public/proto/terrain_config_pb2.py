# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: simian/public/proto/terrain_config.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from simian.public.proto import common_pb2 as simian_dot_public_dot_proto_dot_common__pb2
from simian.public.proto import field_options_pb2 as simian_dot_public_dot_proto_dot_field__options__pb2
from simian.public.proto import spatial_pb2 as simian_dot_public_dot_proto_dot_spatial__pb2


# @@protoc_insertion_point(post_imports)

_sym_db = _symbol_database.Default()


DESCRIPTOR = _descriptor.FileDescriptor(
  name='simian/public/proto/terrain_config.proto',
  package='simian_public.common',
  syntax='proto3',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n(simian/public/proto/terrain_config.proto\x12\x14simian_public.common\x1a simian/public/proto/common.proto\x1a\'simian/public/proto/field_options.proto\x1a!simian/public/proto/spatial.proto\"\xb3\x01\n\x11TerrainProperties\x12)\n\x1bstatic_friction_coefficient\x18\x01 \x01(\x01\x42\x04\xd0\xb5\x18\x01\x12*\n\x1c\x64ynamic_friction_coefficient\x18\x02 \x01(\x01\x42\x04\xd0\xb5\x18\x01\x12%\n\x17restitution_coefficient\x18\x03 \x01(\x01\x42\x04\xd0\xb5\x18\x01\x12 \n\x12rolling_resistance\x18\x04 \x01(\x01\x42\x04\xd0\xb5\x18\x01\"g\n\x11RegionDescription\x12\x43\n\x11vertical_cylinder\x18\x01 \x01(\x0b\x32&.simian_public.common.VerticalCylinderH\x00\x42\r\n\x0bregion_type\"l\n\x10VerticalCylinder\x12,\n\x06\x63\x65nter\x18\x01 \x01(\x0b\x32\x1c.simian_public.spatial.Point\x12\x14\n\x06radius\x18\x02 \x01(\x01\x42\x04\xd0\xb5\x18\x02\x12\x14\n\x06height\x18\x03 \x01(\x01\x42\x04\xd0\xb5\x18\x02\"\x93\x01\n\x1bRegionPropertiesDescription\x12\x37\n\x06region\x18\x01 \x01(\x0b\x32\'.simian_public.common.RegionDescription\x12;\n\nproperties\x18\x02 \x01(\x0b\x32\'.simian_public.common.TerrainProperties\"\xce\x0b\n\rTerrainConfig\x12V\n\x14lane_height_delaunay\x18\x01 \x01(\x0b\x32\x36.simian_public.common.TerrainConfig.LaneHeightDelaunayH\x00\x12\x32\n\x0blane_height\x18\x02 \x01(\x0b\x32\x1b.simian_public.common.EmptyH\x00\x12N\n\x10point_and_normal\x18\x03 \x01(\x0b\x32\x32.simian_public.common.TerrainConfig.PointAndNormalH\x00\x12I\n\rtriangle_mesh\x18\x04 \x01(\x0b\x32\x30.simian_public.common.TerrainConfig.TriangleMeshH\x00\x12j\n\x1e\x65xperimental_generated_terrain\x18\x05 \x01(\x0b\x32@.simian_public.common.TerrainConfig.ExperimentalGeneratedTerrainH\x00\x12\x42\n\x11global_properties\x18\x08 \x01(\x0b\x32\'.simian_public.common.TerrainProperties\x12\x42\n\x07regions\x18\t \x03(\x0b\x32\x31.simian_public.common.RegionPropertiesDescription\x12/\n\x06strict\x18\x06 \x01(\x0b\x32\x1f.simian_public.common.BoolValue\x12>\n\x07version\x18\x07 \x01(\x0b\x32\'.simian_public.common.VersionMajorMinorB\x04\x80\xb5\x18\x01\x1a\xe2\x01\n\x0cTriangleMesh\x12\x19\n\robj_file_name\x18\x01 \x01(\tB\x02\x18\x01\x12\x16\n\x0emesh_file_name\x18\x05 \x01(\t\x12\x1f\n\nmesh_scale\x18\x02 \x01(\x01\x42\x0b\x91\xb5\x18\x00\x00\x00\x00\x00\x00\x00\x00\x12<\n\x13world_t_scaled_mesh\x18\x03 \x01(\x0b\x32\x1f.simian_public.spatial.PoseSpec\x12@\n\tprecision\x18\x04 \x01(\x0b\x32-.simian_public.common.TerrainConfig.Precision\x1a\x9e\x01\n\x1c\x45xperimentalGeneratedTerrain\x12<\n\x06source\x18\x01 \x01(\x0b\x32,.simian_public.common.GeneratedTerrainSource\x12@\n\tprecision\x18\x02 \x01(\x0b\x32-.simian_public.common.TerrainConfig.Precision\x1a\x94\x01\n\x12LaneHeightDelaunay\x12<\n\x06source\x18\x01 \x01(\x0b\x32,.simian_public.common.GeneratedTerrainSource\x12@\n\tprecision\x18\x02 \x01(\x0b\x32-.simian_public.common.TerrainConfig.Precision\x1a\x87\x01\n\x0ePointAndNormal\x12\x10\n\x02px\x18\x01 \x01(\x01\x42\x04\xd0\xb5\x18\x02\x12\x10\n\x02py\x18\x02 \x01(\x01\x42\x04\xd0\xb5\x18\x02\x12\x10\n\x02pz\x18\x03 \x01(\x01\x42\x04\xd0\xb5\x18\x02\x12\x10\n\x02nx\x18\x04 \x01(\x01\x42\x04\xd0\xb5\x18\x01\x12\x10\n\x02ny\x18\x05 \x01(\x01\x42\x04\xd0\xb5\x18\x01\x12\x1b\n\x02nz\x18\x06 \x01(\x01\x42\x0f\x91\xb5\x18\x00\x00\x00\x00\x00\x00\x00\x00\xd0\xb5\x18\x01\x1az\n\tPrecision\x12-\n\x06medium\x18\x01 \x01(\x0b\x32\x1b.simian_public.common.EmptyH\x00\x12+\n\x04high\x18\x02 \x01(\x0b\x32\x1b.simian_public.common.EmptyH\x00\x42\x11\n\x0fprecision_levelB\x0e\n\x0cterrain_type\"\x83\x01\n\x16GeneratedTerrainSource\x12.\n\x07viz_map\x18\x07 \x01(\x0b\x32\x1b.simian_public.common.EmptyH\x00\x12/\n\x08\x62\x61se_map\x18\x08 \x01(\x0b\x32\x1b.simian_public.common.EmptyH\x00\x42\x08\n\x06sourceb\x06proto3'
  ,
  dependencies=[simian_dot_public_dot_proto_dot_common__pb2.DESCRIPTOR,simian_dot_public_dot_proto_dot_field__options__pb2.DESCRIPTOR,simian_dot_public_dot_proto_dot_spatial__pb2.DESCRIPTOR,])




_TERRAINPROPERTIES = _descriptor.Descriptor(
  name='TerrainProperties',
  full_name='simian_public.common.TerrainProperties',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='static_friction_coefficient', full_name='simian_public.common.TerrainProperties.static_friction_coefficient', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\320\265\030\001', file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='dynamic_friction_coefficient', full_name='simian_public.common.TerrainProperties.dynamic_friction_coefficient', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\320\265\030\001', file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='restitution_coefficient', full_name='simian_public.common.TerrainProperties.restitution_coefficient', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\320\265\030\001', file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='rolling_resistance', full_name='simian_public.common.TerrainProperties.rolling_resistance', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\320\265\030\001', file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
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
  serialized_start=177,
  serialized_end=356,
)


_REGIONDESCRIPTION = _descriptor.Descriptor(
  name='RegionDescription',
  full_name='simian_public.common.RegionDescription',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='vertical_cylinder', full_name='simian_public.common.RegionDescription.vertical_cylinder', index=0,
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
      name='region_type', full_name='simian_public.common.RegionDescription.region_type',
      index=0, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
  ],
  serialized_start=358,
  serialized_end=461,
)


_VERTICALCYLINDER = _descriptor.Descriptor(
  name='VerticalCylinder',
  full_name='simian_public.common.VerticalCylinder',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='center', full_name='simian_public.common.VerticalCylinder.center', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='radius', full_name='simian_public.common.VerticalCylinder.radius', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\320\265\030\002', file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='height', full_name='simian_public.common.VerticalCylinder.height', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\320\265\030\002', file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
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
  serialized_start=463,
  serialized_end=571,
)


_REGIONPROPERTIESDESCRIPTION = _descriptor.Descriptor(
  name='RegionPropertiesDescription',
  full_name='simian_public.common.RegionPropertiesDescription',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='region', full_name='simian_public.common.RegionPropertiesDescription.region', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='properties', full_name='simian_public.common.RegionPropertiesDescription.properties', index=1,
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
  serialized_start=574,
  serialized_end=721,
)


_TERRAINCONFIG_TRIANGLEMESH = _descriptor.Descriptor(
  name='TriangleMesh',
  full_name='simian_public.common.TerrainConfig.TriangleMesh',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='obj_file_name', full_name='simian_public.common.TerrainConfig.TriangleMesh.obj_file_name', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\030\001', file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='mesh_file_name', full_name='simian_public.common.TerrainConfig.TriangleMesh.mesh_file_name', index=1,
      number=5, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='mesh_scale', full_name='simian_public.common.TerrainConfig.TriangleMesh.mesh_scale', index=2,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\221\265\030\000\000\000\000\000\000\000\000', file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='world_t_scaled_mesh', full_name='simian_public.common.TerrainConfig.TriangleMesh.world_t_scaled_mesh', index=3,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='precision', full_name='simian_public.common.TerrainConfig.TriangleMesh.precision', index=4,
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
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=1394,
  serialized_end=1620,
)

_TERRAINCONFIG_EXPERIMENTALGENERATEDTERRAIN = _descriptor.Descriptor(
  name='ExperimentalGeneratedTerrain',
  full_name='simian_public.common.TerrainConfig.ExperimentalGeneratedTerrain',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='source', full_name='simian_public.common.TerrainConfig.ExperimentalGeneratedTerrain.source', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='precision', full_name='simian_public.common.TerrainConfig.ExperimentalGeneratedTerrain.precision', index=1,
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
  serialized_start=1623,
  serialized_end=1781,
)

_TERRAINCONFIG_LANEHEIGHTDELAUNAY = _descriptor.Descriptor(
  name='LaneHeightDelaunay',
  full_name='simian_public.common.TerrainConfig.LaneHeightDelaunay',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='source', full_name='simian_public.common.TerrainConfig.LaneHeightDelaunay.source', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='precision', full_name='simian_public.common.TerrainConfig.LaneHeightDelaunay.precision', index=1,
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
  serialized_start=1784,
  serialized_end=1932,
)

_TERRAINCONFIG_POINTANDNORMAL = _descriptor.Descriptor(
  name='PointAndNormal',
  full_name='simian_public.common.TerrainConfig.PointAndNormal',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='px', full_name='simian_public.common.TerrainConfig.PointAndNormal.px', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\320\265\030\002', file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='py', full_name='simian_public.common.TerrainConfig.PointAndNormal.py', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\320\265\030\002', file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='pz', full_name='simian_public.common.TerrainConfig.PointAndNormal.pz', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\320\265\030\002', file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='nx', full_name='simian_public.common.TerrainConfig.PointAndNormal.nx', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\320\265\030\001', file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='ny', full_name='simian_public.common.TerrainConfig.PointAndNormal.ny', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\320\265\030\001', file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='nz', full_name='simian_public.common.TerrainConfig.PointAndNormal.nz', index=5,
      number=6, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\221\265\030\000\000\000\000\000\000\000\000\320\265\030\001', file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
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
  serialized_start=1935,
  serialized_end=2070,
)

_TERRAINCONFIG_PRECISION = _descriptor.Descriptor(
  name='Precision',
  full_name='simian_public.common.TerrainConfig.Precision',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='medium', full_name='simian_public.common.TerrainConfig.Precision.medium', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='high', full_name='simian_public.common.TerrainConfig.Precision.high', index=1,
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
    _descriptor.OneofDescriptor(
      name='precision_level', full_name='simian_public.common.TerrainConfig.Precision.precision_level',
      index=0, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
  ],
  serialized_start=2072,
  serialized_end=2194,
)

_TERRAINCONFIG = _descriptor.Descriptor(
  name='TerrainConfig',
  full_name='simian_public.common.TerrainConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='lane_height_delaunay', full_name='simian_public.common.TerrainConfig.lane_height_delaunay', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='lane_height', full_name='simian_public.common.TerrainConfig.lane_height', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='point_and_normal', full_name='simian_public.common.TerrainConfig.point_and_normal', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='triangle_mesh', full_name='simian_public.common.TerrainConfig.triangle_mesh', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='experimental_generated_terrain', full_name='simian_public.common.TerrainConfig.experimental_generated_terrain', index=4,
      number=5, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='global_properties', full_name='simian_public.common.TerrainConfig.global_properties', index=5,
      number=8, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='regions', full_name='simian_public.common.TerrainConfig.regions', index=6,
      number=9, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='strict', full_name='simian_public.common.TerrainConfig.strict', index=7,
      number=6, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='version', full_name='simian_public.common.TerrainConfig.version', index=8,
      number=7, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\200\265\030\001', file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[_TERRAINCONFIG_TRIANGLEMESH, _TERRAINCONFIG_EXPERIMENTALGENERATEDTERRAIN, _TERRAINCONFIG_LANEHEIGHTDELAUNAY, _TERRAINCONFIG_POINTANDNORMAL, _TERRAINCONFIG_PRECISION, ],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
    _descriptor.OneofDescriptor(
      name='terrain_type', full_name='simian_public.common.TerrainConfig.terrain_type',
      index=0, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
  ],
  serialized_start=724,
  serialized_end=2210,
)


_GENERATEDTERRAINSOURCE = _descriptor.Descriptor(
  name='GeneratedTerrainSource',
  full_name='simian_public.common.GeneratedTerrainSource',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='viz_map', full_name='simian_public.common.GeneratedTerrainSource.viz_map', index=0,
      number=7, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='base_map', full_name='simian_public.common.GeneratedTerrainSource.base_map', index=1,
      number=8, type=11, cpp_type=10, label=1,
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
      name='source', full_name='simian_public.common.GeneratedTerrainSource.source',
      index=0, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
  ],
  serialized_start=2213,
  serialized_end=2344,
)

_REGIONDESCRIPTION.fields_by_name['vertical_cylinder'].message_type = _VERTICALCYLINDER
_REGIONDESCRIPTION.oneofs_by_name['region_type'].fields.append(
  _REGIONDESCRIPTION.fields_by_name['vertical_cylinder'])
_REGIONDESCRIPTION.fields_by_name['vertical_cylinder'].containing_oneof = _REGIONDESCRIPTION.oneofs_by_name['region_type']
_VERTICALCYLINDER.fields_by_name['center'].message_type = simian_dot_public_dot_proto_dot_spatial__pb2._POINT
_REGIONPROPERTIESDESCRIPTION.fields_by_name['region'].message_type = _REGIONDESCRIPTION
_REGIONPROPERTIESDESCRIPTION.fields_by_name['properties'].message_type = _TERRAINPROPERTIES
_TERRAINCONFIG_TRIANGLEMESH.fields_by_name['world_t_scaled_mesh'].message_type = simian_dot_public_dot_proto_dot_spatial__pb2._POSESPEC
_TERRAINCONFIG_TRIANGLEMESH.fields_by_name['precision'].message_type = _TERRAINCONFIG_PRECISION
_TERRAINCONFIG_TRIANGLEMESH.containing_type = _TERRAINCONFIG
_TERRAINCONFIG_EXPERIMENTALGENERATEDTERRAIN.fields_by_name['source'].message_type = _GENERATEDTERRAINSOURCE
_TERRAINCONFIG_EXPERIMENTALGENERATEDTERRAIN.fields_by_name['precision'].message_type = _TERRAINCONFIG_PRECISION
_TERRAINCONFIG_EXPERIMENTALGENERATEDTERRAIN.containing_type = _TERRAINCONFIG
_TERRAINCONFIG_LANEHEIGHTDELAUNAY.fields_by_name['source'].message_type = _GENERATEDTERRAINSOURCE
_TERRAINCONFIG_LANEHEIGHTDELAUNAY.fields_by_name['precision'].message_type = _TERRAINCONFIG_PRECISION
_TERRAINCONFIG_LANEHEIGHTDELAUNAY.containing_type = _TERRAINCONFIG
_TERRAINCONFIG_POINTANDNORMAL.containing_type = _TERRAINCONFIG
_TERRAINCONFIG_PRECISION.fields_by_name['medium'].message_type = simian_dot_public_dot_proto_dot_common__pb2._EMPTY
_TERRAINCONFIG_PRECISION.fields_by_name['high'].message_type = simian_dot_public_dot_proto_dot_common__pb2._EMPTY
_TERRAINCONFIG_PRECISION.containing_type = _TERRAINCONFIG
_TERRAINCONFIG_PRECISION.oneofs_by_name['precision_level'].fields.append(
  _TERRAINCONFIG_PRECISION.fields_by_name['medium'])
_TERRAINCONFIG_PRECISION.fields_by_name['medium'].containing_oneof = _TERRAINCONFIG_PRECISION.oneofs_by_name['precision_level']
_TERRAINCONFIG_PRECISION.oneofs_by_name['precision_level'].fields.append(
  _TERRAINCONFIG_PRECISION.fields_by_name['high'])
_TERRAINCONFIG_PRECISION.fields_by_name['high'].containing_oneof = _TERRAINCONFIG_PRECISION.oneofs_by_name['precision_level']
_TERRAINCONFIG.fields_by_name['lane_height_delaunay'].message_type = _TERRAINCONFIG_LANEHEIGHTDELAUNAY
_TERRAINCONFIG.fields_by_name['lane_height'].message_type = simian_dot_public_dot_proto_dot_common__pb2._EMPTY
_TERRAINCONFIG.fields_by_name['point_and_normal'].message_type = _TERRAINCONFIG_POINTANDNORMAL
_TERRAINCONFIG.fields_by_name['triangle_mesh'].message_type = _TERRAINCONFIG_TRIANGLEMESH
_TERRAINCONFIG.fields_by_name['experimental_generated_terrain'].message_type = _TERRAINCONFIG_EXPERIMENTALGENERATEDTERRAIN
_TERRAINCONFIG.fields_by_name['global_properties'].message_type = _TERRAINPROPERTIES
_TERRAINCONFIG.fields_by_name['regions'].message_type = _REGIONPROPERTIESDESCRIPTION
_TERRAINCONFIG.fields_by_name['strict'].message_type = simian_dot_public_dot_proto_dot_common__pb2._BOOLVALUE
_TERRAINCONFIG.fields_by_name['version'].message_type = simian_dot_public_dot_proto_dot_common__pb2._VERSIONMAJORMINOR
_TERRAINCONFIG.oneofs_by_name['terrain_type'].fields.append(
  _TERRAINCONFIG.fields_by_name['lane_height_delaunay'])
_TERRAINCONFIG.fields_by_name['lane_height_delaunay'].containing_oneof = _TERRAINCONFIG.oneofs_by_name['terrain_type']
_TERRAINCONFIG.oneofs_by_name['terrain_type'].fields.append(
  _TERRAINCONFIG.fields_by_name['lane_height'])
_TERRAINCONFIG.fields_by_name['lane_height'].containing_oneof = _TERRAINCONFIG.oneofs_by_name['terrain_type']
_TERRAINCONFIG.oneofs_by_name['terrain_type'].fields.append(
  _TERRAINCONFIG.fields_by_name['point_and_normal'])
_TERRAINCONFIG.fields_by_name['point_and_normal'].containing_oneof = _TERRAINCONFIG.oneofs_by_name['terrain_type']
_TERRAINCONFIG.oneofs_by_name['terrain_type'].fields.append(
  _TERRAINCONFIG.fields_by_name['triangle_mesh'])
_TERRAINCONFIG.fields_by_name['triangle_mesh'].containing_oneof = _TERRAINCONFIG.oneofs_by_name['terrain_type']
_TERRAINCONFIG.oneofs_by_name['terrain_type'].fields.append(
  _TERRAINCONFIG.fields_by_name['experimental_generated_terrain'])
_TERRAINCONFIG.fields_by_name['experimental_generated_terrain'].containing_oneof = _TERRAINCONFIG.oneofs_by_name['terrain_type']
_GENERATEDTERRAINSOURCE.fields_by_name['viz_map'].message_type = simian_dot_public_dot_proto_dot_common__pb2._EMPTY
_GENERATEDTERRAINSOURCE.fields_by_name['base_map'].message_type = simian_dot_public_dot_proto_dot_common__pb2._EMPTY
_GENERATEDTERRAINSOURCE.oneofs_by_name['source'].fields.append(
  _GENERATEDTERRAINSOURCE.fields_by_name['viz_map'])
_GENERATEDTERRAINSOURCE.fields_by_name['viz_map'].containing_oneof = _GENERATEDTERRAINSOURCE.oneofs_by_name['source']
_GENERATEDTERRAINSOURCE.oneofs_by_name['source'].fields.append(
  _GENERATEDTERRAINSOURCE.fields_by_name['base_map'])
_GENERATEDTERRAINSOURCE.fields_by_name['base_map'].containing_oneof = _GENERATEDTERRAINSOURCE.oneofs_by_name['source']
DESCRIPTOR.message_types_by_name['TerrainProperties'] = _TERRAINPROPERTIES
DESCRIPTOR.message_types_by_name['RegionDescription'] = _REGIONDESCRIPTION
DESCRIPTOR.message_types_by_name['VerticalCylinder'] = _VERTICALCYLINDER
DESCRIPTOR.message_types_by_name['RegionPropertiesDescription'] = _REGIONPROPERTIESDESCRIPTION
DESCRIPTOR.message_types_by_name['TerrainConfig'] = _TERRAINCONFIG
DESCRIPTOR.message_types_by_name['GeneratedTerrainSource'] = _GENERATEDTERRAINSOURCE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

TerrainProperties = _reflection.GeneratedProtocolMessageType('TerrainProperties', (_message.Message,), {
  'DESCRIPTOR' : _TERRAINPROPERTIES,
  '__module__' : 'simian.public.proto.terrain_config_pb2'
  # @@protoc_insertion_point(class_scope:simian_public.common.TerrainProperties)
  })
_sym_db.RegisterMessage(TerrainProperties)

RegionDescription = _reflection.GeneratedProtocolMessageType('RegionDescription', (_message.Message,), {
  'DESCRIPTOR' : _REGIONDESCRIPTION,
  '__module__' : 'simian.public.proto.terrain_config_pb2'
  # @@protoc_insertion_point(class_scope:simian_public.common.RegionDescription)
  })
_sym_db.RegisterMessage(RegionDescription)

VerticalCylinder = _reflection.GeneratedProtocolMessageType('VerticalCylinder', (_message.Message,), {
  'DESCRIPTOR' : _VERTICALCYLINDER,
  '__module__' : 'simian.public.proto.terrain_config_pb2'
  # @@protoc_insertion_point(class_scope:simian_public.common.VerticalCylinder)
  })
_sym_db.RegisterMessage(VerticalCylinder)

RegionPropertiesDescription = _reflection.GeneratedProtocolMessageType('RegionPropertiesDescription', (_message.Message,), {
  'DESCRIPTOR' : _REGIONPROPERTIESDESCRIPTION,
  '__module__' : 'simian.public.proto.terrain_config_pb2'
  # @@protoc_insertion_point(class_scope:simian_public.common.RegionPropertiesDescription)
  })
_sym_db.RegisterMessage(RegionPropertiesDescription)

TerrainConfig = _reflection.GeneratedProtocolMessageType('TerrainConfig', (_message.Message,), {

  'TriangleMesh' : _reflection.GeneratedProtocolMessageType('TriangleMesh', (_message.Message,), {
    'DESCRIPTOR' : _TERRAINCONFIG_TRIANGLEMESH,
    '__module__' : 'simian.public.proto.terrain_config_pb2'
    # @@protoc_insertion_point(class_scope:simian_public.common.TerrainConfig.TriangleMesh)
    })
  ,

  'ExperimentalGeneratedTerrain' : _reflection.GeneratedProtocolMessageType('ExperimentalGeneratedTerrain', (_message.Message,), {
    'DESCRIPTOR' : _TERRAINCONFIG_EXPERIMENTALGENERATEDTERRAIN,
    '__module__' : 'simian.public.proto.terrain_config_pb2'
    # @@protoc_insertion_point(class_scope:simian_public.common.TerrainConfig.ExperimentalGeneratedTerrain)
    })
  ,

  'LaneHeightDelaunay' : _reflection.GeneratedProtocolMessageType('LaneHeightDelaunay', (_message.Message,), {
    'DESCRIPTOR' : _TERRAINCONFIG_LANEHEIGHTDELAUNAY,
    '__module__' : 'simian.public.proto.terrain_config_pb2'
    # @@protoc_insertion_point(class_scope:simian_public.common.TerrainConfig.LaneHeightDelaunay)
    })
  ,

  'PointAndNormal' : _reflection.GeneratedProtocolMessageType('PointAndNormal', (_message.Message,), {
    'DESCRIPTOR' : _TERRAINCONFIG_POINTANDNORMAL,
    '__module__' : 'simian.public.proto.terrain_config_pb2'
    # @@protoc_insertion_point(class_scope:simian_public.common.TerrainConfig.PointAndNormal)
    })
  ,

  'Precision' : _reflection.GeneratedProtocolMessageType('Precision', (_message.Message,), {
    'DESCRIPTOR' : _TERRAINCONFIG_PRECISION,
    '__module__' : 'simian.public.proto.terrain_config_pb2'
    # @@protoc_insertion_point(class_scope:simian_public.common.TerrainConfig.Precision)
    })
  ,
  'DESCRIPTOR' : _TERRAINCONFIG,
  '__module__' : 'simian.public.proto.terrain_config_pb2'
  # @@protoc_insertion_point(class_scope:simian_public.common.TerrainConfig)
  })
_sym_db.RegisterMessage(TerrainConfig)
_sym_db.RegisterMessage(TerrainConfig.TriangleMesh)
_sym_db.RegisterMessage(TerrainConfig.ExperimentalGeneratedTerrain)
_sym_db.RegisterMessage(TerrainConfig.LaneHeightDelaunay)
_sym_db.RegisterMessage(TerrainConfig.PointAndNormal)
_sym_db.RegisterMessage(TerrainConfig.Precision)

GeneratedTerrainSource = _reflection.GeneratedProtocolMessageType('GeneratedTerrainSource', (_message.Message,), {
  'DESCRIPTOR' : _GENERATEDTERRAINSOURCE,
  '__module__' : 'simian.public.proto.terrain_config_pb2'
  # @@protoc_insertion_point(class_scope:simian_public.common.GeneratedTerrainSource)
  })
_sym_db.RegisterMessage(GeneratedTerrainSource)


_TERRAINPROPERTIES.fields_by_name['static_friction_coefficient']._options = None
_TERRAINPROPERTIES.fields_by_name['dynamic_friction_coefficient']._options = None
_TERRAINPROPERTIES.fields_by_name['restitution_coefficient']._options = None
_TERRAINPROPERTIES.fields_by_name['rolling_resistance']._options = None
_VERTICALCYLINDER.fields_by_name['radius']._options = None
_VERTICALCYLINDER.fields_by_name['height']._options = None
_TERRAINCONFIG_TRIANGLEMESH.fields_by_name['obj_file_name']._options = None
_TERRAINCONFIG_TRIANGLEMESH.fields_by_name['mesh_scale']._options = None
_TERRAINCONFIG_POINTANDNORMAL.fields_by_name['px']._options = None
_TERRAINCONFIG_POINTANDNORMAL.fields_by_name['py']._options = None
_TERRAINCONFIG_POINTANDNORMAL.fields_by_name['pz']._options = None
_TERRAINCONFIG_POINTANDNORMAL.fields_by_name['nx']._options = None
_TERRAINCONFIG_POINTANDNORMAL.fields_by_name['ny']._options = None
_TERRAINCONFIG_POINTANDNORMAL.fields_by_name['nz']._options = None
_TERRAINCONFIG.fields_by_name['version']._options = None
# @@protoc_insertion_point(module_scope)
