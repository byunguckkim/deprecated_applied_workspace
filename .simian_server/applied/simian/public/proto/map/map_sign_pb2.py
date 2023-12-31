# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: simian/public/proto/map/map_sign.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from simian.public.proto.map import map_common_pb2 as simian_dot_public_dot_proto_dot_map_dot_map__common__pb2
from simian.public.proto.map import map_geometry_pb2 as simian_dot_public_dot_proto_dot_map_dot_map__geometry__pb2
from simian.public.proto.map import map_id_pb2 as simian_dot_public_dot_proto_dot_map_dot_map__id__pb2
from simian.public.proto.map import map_validation_pb2 as simian_dot_public_dot_proto_dot_map_dot_map__validation__pb2
from simian.public.proto import spatial_pb2 as simian_dot_public_dot_proto_dot_spatial__pb2
from simian.public.proto import spectral_assets_pb2 as simian_dot_public_dot_proto_dot_spectral__assets__pb2


# @@protoc_insertion_point(post_imports)

_sym_db = _symbol_database.Default()


DESCRIPTOR = _descriptor.FileDescriptor(
  name='simian/public/proto/map/map_sign.proto',
  package='simian_public.hdmap',
  syntax='proto2',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n&simian/public/proto/map/map_sign.proto\x12\x13simian_public.hdmap\x1a(simian/public/proto/map/map_common.proto\x1a*simian/public/proto/map/map_geometry.proto\x1a$simian/public/proto/map/map_id.proto\x1a,simian/public/proto/map/map_validation.proto\x1a!simian/public/proto/spatial.proto\x1a)simian/public/proto/spectral_assets.proto\"\xa2\x0c\n\x04Sign\x12#\n\x02id\x18\x01 \x01(\x0b\x32\x17.simian_public.hdmap.Id\x12-\n\x07polygon\x18\x02 \x01(\x0b\x32\x1c.simian_public.hdmap.Polygon\x12-\n\x04pose\x18\x0f \x01(\x0b\x32\x1f.simian_public.spatial.PoseSpec\x12\x30\n\x04type\x18\x03 \x01(\x0e\x32\".simian_public.hdmap.Sign.SignType\x12\x13\n\x0b\x63ustom_type\x18\t \x01(\t\x12\x31\n\tstop_line\x18\x04 \x03(\x0b\x32\x1a.simian_public.hdmap.CurveB\x02\x18\x01\x12\x14\n\x0csign_message\x18\x06 \x01(\t\x12\x13\n\x0bspeed_limit\x18\x07 \x01(\x01\x12\x30\n\x0f\x61ssociated_lane\x18\x08 \x03(\x0b\x32\x17.simian_public.hdmap.Id\x12\x32\n\x05\x63olor\x18\n \x01(\x0e\x32#.simian_public.hdmap.Sign.SignColor\x12\x14\n\x0c\x63ustom_color\x18\x0b \x01(\t\x12\x32\n\x05shape\x18\x0c \x01(\x0e\x32#.simian_public.hdmap.Sign.SignShape\x12\x14\n\x0c\x63ustom_shape\x18\r \x01(\t\x12\x32\n\x11\x61ssociated_region\x18\x0e \x03(\x0b\x32\x17.simian_public.hdmap.Id\x12\x33\n\x0b\x63ustom_tags\x18\x10 \x03(\x0b\x32\x1e.simian_public.hdmap.CustomTag\x12\x41\n\x13\x63onversion_failures\x18\x12 \x03(\x0b\x32$.simian_public.hdmap.ValidationCheck\x12I\n\x13physical_attributes\x18\x13 \x01(\x0b\x32,.simian_public.hdmap.Sign.PhysicalAttributes\x1a\x9f\x01\n\x12PhysicalAttributes\x12;\n\x06preset\x18\x01 \x01(\x0e\x32+.simian_public.spectral.SpectralSign.Preset\x12L\n\x08material\x18\x02 \x01(\x0b\x32:.simian_public.spectral.SpectralSign.MaterialConfiguration\"\x85\x03\n\x08SignType\x12\x0b\n\x07UNKNOWN\x10\x00\x12\n\n\x06\x43USTOM\x10\t\x12\x08\n\x04STOP\x10\x01\x12\t\n\x05YIELD\x10\x02\x12\x16\n\x12NO_PARKING_ANYTIME\x10\x03\x12\x10\n\x0cGENERAL_SIGN\x10\x04\x12\r\n\tBIKE_LANE\x10\x05\x12\x0f\n\x0bSPEED_LIMIT\x10\x06\x12\x17\n\x13NO_STOPPING_ANYTIME\x10\x07\x12\x0e\n\nNO_PARKING\x10\x08\x12\x14\n\x10LEFT_LANE_CLOSED\x10\n\x12\x15\n\x11RIGHT_LANE_CLOSED\x10\x0b\x12\x13\n\x0fROAD_WORK_AHEAD\x10\r\x12\r\n\tWORK_ZONE\x10\x0e\x12\x19\n\x15TEMPORARY_SPEED_LIMIT\x10\x0f\x12\x0e\n\nLANE_SHIFT\x10\x10\x12\x13\n\x0fSHOULDER_CLOSED\x10\x11\x12\x0f\n\x0b\x45XIT_CLOSED\x10\x12\x12\x0f\n\x0bROAD_CLOSED\x10\x13\x12\x11\n\rEND_ROAD_WORK\x10\x14\x12\x12\n\x0eROAD_WORK_ZONE\x10\x15\"e\n\tSignColor\x12\x11\n\rUNKNOWN_COLOR\x10\x00\x12\x10\n\x0c\x43USTOM_COLOR\x10\x06\x12\t\n\x05WHITE\x10\x01\x12\x07\n\x03RED\x10\x02\x12\t\n\x05GREEN\x10\x03\x12\x08\n\x04\x42LUE\x10\x04\x12\n\n\x06YELLOW\x10\x05\"z\n\tSignShape\x12\x11\n\rUNKNOWN_SHAPE\x10\x00\x12\x10\n\x0c\x43USTOM_SHAPE\x10\x06\x12\n\n\x06SQUARE\x10\x01\x12\x0c\n\x08TRIANGLE\x10\x02\x12\n\n\x06\x43IRCLE\x10\x03\x12\x0b\n\x07OCTAGON\x10\x04\x12\x15\n\x11INVERTED_TRIANGLE\x10\x05J\x04\x08\x05\x10\x06J\x04\x08\x11\x10\x12R\noverlap_idR\x10\x63onversion_error'
  ,
  dependencies=[simian_dot_public_dot_proto_dot_map_dot_map__common__pb2.DESCRIPTOR,simian_dot_public_dot_proto_dot_map_dot_map__geometry__pb2.DESCRIPTOR,simian_dot_public_dot_proto_dot_map_dot_map__id__pb2.DESCRIPTOR,simian_dot_public_dot_proto_dot_map_dot_map__validation__pb2.DESCRIPTOR,simian_dot_public_dot_proto_dot_spatial__pb2.DESCRIPTOR,simian_dot_public_dot_proto_dot_spectral__assets__pb2.DESCRIPTOR,])



_SIGN_SIGNTYPE = _descriptor.EnumDescriptor(
  name='SignType',
  full_name='simian_public.hdmap.Sign.SignType',
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
      name='CUSTOM', index=1, number=9,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='STOP', index=2, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='YIELD', index=3, number=2,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='NO_PARKING_ANYTIME', index=4, number=3,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='GENERAL_SIGN', index=5, number=4,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='BIKE_LANE', index=6, number=5,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='SPEED_LIMIT', index=7, number=6,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='NO_STOPPING_ANYTIME', index=8, number=7,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='NO_PARKING', index=9, number=8,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='LEFT_LANE_CLOSED', index=10, number=10,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='RIGHT_LANE_CLOSED', index=11, number=11,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='ROAD_WORK_AHEAD', index=12, number=13,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='WORK_ZONE', index=13, number=14,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='TEMPORARY_SPEED_LIMIT', index=14, number=15,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='LANE_SHIFT', index=15, number=16,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='SHOULDER_CLOSED', index=16, number=17,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='EXIT_CLOSED', index=17, number=18,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='ROAD_CLOSED', index=18, number=19,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='END_ROAD_WORK', index=19, number=20,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='ROAD_WORK_ZONE', index=20, number=21,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=1224,
  serialized_end=1613,
)
_sym_db.RegisterEnumDescriptor(_SIGN_SIGNTYPE)

_SIGN_SIGNCOLOR = _descriptor.EnumDescriptor(
  name='SignColor',
  full_name='simian_public.hdmap.Sign.SignColor',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='UNKNOWN_COLOR', index=0, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='CUSTOM_COLOR', index=1, number=6,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='WHITE', index=2, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='RED', index=3, number=2,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='GREEN', index=4, number=3,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='BLUE', index=5, number=4,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='YELLOW', index=6, number=5,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=1615,
  serialized_end=1716,
)
_sym_db.RegisterEnumDescriptor(_SIGN_SIGNCOLOR)

_SIGN_SIGNSHAPE = _descriptor.EnumDescriptor(
  name='SignShape',
  full_name='simian_public.hdmap.Sign.SignShape',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='UNKNOWN_SHAPE', index=0, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='CUSTOM_SHAPE', index=1, number=6,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='SQUARE', index=2, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='TRIANGLE', index=3, number=2,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='CIRCLE', index=4, number=3,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='OCTAGON', index=5, number=4,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='INVERTED_TRIANGLE', index=6, number=5,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=1718,
  serialized_end=1840,
)
_sym_db.RegisterEnumDescriptor(_SIGN_SIGNSHAPE)


_SIGN_PHYSICALATTRIBUTES = _descriptor.Descriptor(
  name='PhysicalAttributes',
  full_name='simian_public.hdmap.Sign.PhysicalAttributes',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='preset', full_name='simian_public.hdmap.Sign.PhysicalAttributes.preset', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='material', full_name='simian_public.hdmap.Sign.PhysicalAttributes.material', index=1,
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
  serialized_start=1062,
  serialized_end=1221,
)

_SIGN = _descriptor.Descriptor(
  name='Sign',
  full_name='simian_public.hdmap.Sign',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='simian_public.hdmap.Sign.id', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='polygon', full_name='simian_public.hdmap.Sign.polygon', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='pose', full_name='simian_public.hdmap.Sign.pose', index=2,
      number=15, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='type', full_name='simian_public.hdmap.Sign.type', index=3,
      number=3, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='custom_type', full_name='simian_public.hdmap.Sign.custom_type', index=4,
      number=9, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='stop_line', full_name='simian_public.hdmap.Sign.stop_line', index=5,
      number=4, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\030\001', file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='sign_message', full_name='simian_public.hdmap.Sign.sign_message', index=6,
      number=6, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='speed_limit', full_name='simian_public.hdmap.Sign.speed_limit', index=7,
      number=7, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='associated_lane', full_name='simian_public.hdmap.Sign.associated_lane', index=8,
      number=8, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='color', full_name='simian_public.hdmap.Sign.color', index=9,
      number=10, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='custom_color', full_name='simian_public.hdmap.Sign.custom_color', index=10,
      number=11, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='shape', full_name='simian_public.hdmap.Sign.shape', index=11,
      number=12, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='custom_shape', full_name='simian_public.hdmap.Sign.custom_shape', index=12,
      number=13, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='associated_region', full_name='simian_public.hdmap.Sign.associated_region', index=13,
      number=14, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='custom_tags', full_name='simian_public.hdmap.Sign.custom_tags', index=14,
      number=16, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='conversion_failures', full_name='simian_public.hdmap.Sign.conversion_failures', index=15,
      number=18, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='physical_attributes', full_name='simian_public.hdmap.Sign.physical_attributes', index=16,
      number=19, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[_SIGN_PHYSICALATTRIBUTES, ],
  enum_types=[
    _SIGN_SIGNTYPE,
    _SIGN_SIGNCOLOR,
    _SIGN_SIGNSHAPE,
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=312,
  serialized_end=1882,
)

_SIGN_PHYSICALATTRIBUTES.fields_by_name['preset'].enum_type = simian_dot_public_dot_proto_dot_spectral__assets__pb2._SPECTRALSIGN_PRESET
_SIGN_PHYSICALATTRIBUTES.fields_by_name['material'].message_type = simian_dot_public_dot_proto_dot_spectral__assets__pb2._SPECTRALSIGN_MATERIALCONFIGURATION
_SIGN_PHYSICALATTRIBUTES.containing_type = _SIGN
_SIGN.fields_by_name['id'].message_type = simian_dot_public_dot_proto_dot_map_dot_map__id__pb2._ID
_SIGN.fields_by_name['polygon'].message_type = simian_dot_public_dot_proto_dot_map_dot_map__geometry__pb2._POLYGON
_SIGN.fields_by_name['pose'].message_type = simian_dot_public_dot_proto_dot_spatial__pb2._POSESPEC
_SIGN.fields_by_name['type'].enum_type = _SIGN_SIGNTYPE
_SIGN.fields_by_name['stop_line'].message_type = simian_dot_public_dot_proto_dot_map_dot_map__geometry__pb2._CURVE
_SIGN.fields_by_name['associated_lane'].message_type = simian_dot_public_dot_proto_dot_map_dot_map__id__pb2._ID
_SIGN.fields_by_name['color'].enum_type = _SIGN_SIGNCOLOR
_SIGN.fields_by_name['shape'].enum_type = _SIGN_SIGNSHAPE
_SIGN.fields_by_name['associated_region'].message_type = simian_dot_public_dot_proto_dot_map_dot_map__id__pb2._ID
_SIGN.fields_by_name['custom_tags'].message_type = simian_dot_public_dot_proto_dot_map_dot_map__common__pb2._CUSTOMTAG
_SIGN.fields_by_name['conversion_failures'].message_type = simian_dot_public_dot_proto_dot_map_dot_map__validation__pb2._VALIDATIONCHECK
_SIGN.fields_by_name['physical_attributes'].message_type = _SIGN_PHYSICALATTRIBUTES
_SIGN_SIGNTYPE.containing_type = _SIGN
_SIGN_SIGNCOLOR.containing_type = _SIGN
_SIGN_SIGNSHAPE.containing_type = _SIGN
DESCRIPTOR.message_types_by_name['Sign'] = _SIGN
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Sign = _reflection.GeneratedProtocolMessageType('Sign', (_message.Message,), {

  'PhysicalAttributes' : _reflection.GeneratedProtocolMessageType('PhysicalAttributes', (_message.Message,), {
    'DESCRIPTOR' : _SIGN_PHYSICALATTRIBUTES,
    '__module__' : 'simian.public.proto.map.map_sign_pb2'
    # @@protoc_insertion_point(class_scope:simian_public.hdmap.Sign.PhysicalAttributes)
    })
  ,
  'DESCRIPTOR' : _SIGN,
  '__module__' : 'simian.public.proto.map.map_sign_pb2'
  # @@protoc_insertion_point(class_scope:simian_public.hdmap.Sign)
  })
_sym_db.RegisterMessage(Sign)
_sym_db.RegisterMessage(Sign.PhysicalAttributes)


_SIGN.fields_by_name['stop_line']._options = None
# @@protoc_insertion_point(module_scope)
