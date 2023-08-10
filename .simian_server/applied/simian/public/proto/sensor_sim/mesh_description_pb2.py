# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: simian/public/proto/sensor_sim/mesh_description.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from simian.public.proto import geometry_pb2 as simian_dot_public_dot_proto_dot_geometry__pb2


# @@protoc_insertion_point(post_imports)

_sym_db = _symbol_database.Default()


DESCRIPTOR = _descriptor.FileDescriptor(
  name='simian/public/proto/sensor_sim/mesh_description.proto',
  package='simian_public.sensor_sim.mesh_description',
  syntax='proto3',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n5simian/public/proto/sensor_sim/mesh_description.proto\x12)simian_public.sensor_sim.mesh_description\x1a\"simian/public/proto/geometry.proto\"\xa3\x01\n\x0fMeshDescription\x12\x41\n\x04mesh\x18\x01 \x01(\x0b\x32\x33.simian_public.sensor_sim.mesh_description.MeshFile\x12M\n\nproperties\x18\x02 \x01(\x0b\x32\x39.simian_public.sensor_sim.mesh_description.MeshProperties\"\x1c\n\x08MeshFile\x12\x10\n\x08\x66ile_uri\x18\x01 \x01(\t\"I\n\x0eMeshProperties\x12\x37\n\x0cscale_factor\x18\x01 \x01(\x0b\x32!.simian_public.common.ScaleFactorb\x06proto3'
  ,
  dependencies=[simian_dot_public_dot_proto_dot_geometry__pb2.DESCRIPTOR,])




_MESHDESCRIPTION = _descriptor.Descriptor(
  name='MeshDescription',
  full_name='simian_public.sensor_sim.mesh_description.MeshDescription',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='mesh', full_name='simian_public.sensor_sim.mesh_description.MeshDescription.mesh', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='properties', full_name='simian_public.sensor_sim.mesh_description.MeshDescription.properties', index=1,
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
  serialized_start=137,
  serialized_end=300,
)


_MESHFILE = _descriptor.Descriptor(
  name='MeshFile',
  full_name='simian_public.sensor_sim.mesh_description.MeshFile',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='file_uri', full_name='simian_public.sensor_sim.mesh_description.MeshFile.file_uri', index=0,
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
  serialized_start=302,
  serialized_end=330,
)


_MESHPROPERTIES = _descriptor.Descriptor(
  name='MeshProperties',
  full_name='simian_public.sensor_sim.mesh_description.MeshProperties',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='scale_factor', full_name='simian_public.sensor_sim.mesh_description.MeshProperties.scale_factor', index=0,
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
  ],
  serialized_start=332,
  serialized_end=405,
)

_MESHDESCRIPTION.fields_by_name['mesh'].message_type = _MESHFILE
_MESHDESCRIPTION.fields_by_name['properties'].message_type = _MESHPROPERTIES
_MESHPROPERTIES.fields_by_name['scale_factor'].message_type = simian_dot_public_dot_proto_dot_geometry__pb2._SCALEFACTOR
DESCRIPTOR.message_types_by_name['MeshDescription'] = _MESHDESCRIPTION
DESCRIPTOR.message_types_by_name['MeshFile'] = _MESHFILE
DESCRIPTOR.message_types_by_name['MeshProperties'] = _MESHPROPERTIES
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

MeshDescription = _reflection.GeneratedProtocolMessageType('MeshDescription', (_message.Message,), {
  'DESCRIPTOR' : _MESHDESCRIPTION,
  '__module__' : 'simian.public.proto.sensor_sim.mesh_description_pb2'
  # @@protoc_insertion_point(class_scope:simian_public.sensor_sim.mesh_description.MeshDescription)
  })
_sym_db.RegisterMessage(MeshDescription)

MeshFile = _reflection.GeneratedProtocolMessageType('MeshFile', (_message.Message,), {
  'DESCRIPTOR' : _MESHFILE,
  '__module__' : 'simian.public.proto.sensor_sim.mesh_description_pb2'
  # @@protoc_insertion_point(class_scope:simian_public.sensor_sim.mesh_description.MeshFile)
  })
_sym_db.RegisterMessage(MeshFile)

MeshProperties = _reflection.GeneratedProtocolMessageType('MeshProperties', (_message.Message,), {
  'DESCRIPTOR' : _MESHPROPERTIES,
  '__module__' : 'simian.public.proto.sensor_sim.mesh_description_pb2'
  # @@protoc_insertion_point(class_scope:simian_public.sensor_sim.mesh_description.MeshProperties)
  })
_sym_db.RegisterMessage(MeshProperties)


# @@protoc_insertion_point(module_scope)
