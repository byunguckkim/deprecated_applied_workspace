# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: simian/public/proto/scenario/simulink_config.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from google.protobuf import duration_pb2 as google_dot_protobuf_dot_duration__pb2


# @@protoc_insertion_point(post_imports)

_sym_db = _symbol_database.Default()


DESCRIPTOR = _descriptor.FileDescriptor(
  name='simian/public/proto/scenario/simulink_config.proto',
  package='simian_public.proto.scenario.simulink_config',
  syntax='proto3',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n2simian/public/proto/scenario/simulink_config.proto\x12,simian_public.proto.scenario.simulink_config\x1a\x1egoogle/protobuf/duration.proto\"\xd1\x07\n\x0eSimulinkConfig\x12\x0f\n\x07verbose\x18\x01 \x01(\x08\x12\x15\n\rpause_enabled\x18\x02 \x01(\x08\x12p\n\x15network_configuration\x18\x03 \x01(\x0b\x32Q.simian_public.proto.scenario.simulink_config.SimulinkConfig.NetworkConfiguration\x12p\n\x15timeout_configuration\x18\x04 \x01(\x0b\x32Q.simian_public.proto.scenario.simulink_config.SimulinkConfig.TimeoutConfiguration\x12v\n\x18\x61utomation_configuration\x18\x05 \x01(\x0b\x32T.simian_public.proto.scenario.simulink_config.SimulinkConfig.AutomationConfiguration\x1a\xe1\x01\n\x17\x41utomationConfiguration\x12\x1c\n\x14matlab_root_absolute\x18\x01 \x01(\t\x12\x1b\n\x13model_absolute_path\x18\x02 \x01(\t\x12\x1b\n\x13model_deps_absolute\x18\x03 \x03(\t\x12\x1d\n\x15matlab_absolute_paths\x18\x04 \x03(\t\x12$\n\x1cmodel_init_scripts_filenames\x18\x05 \x03(\t\x12)\n!model_post_init_scripts_filenames\x18\x06 \x03(\t\x1a\x61\n\x14NetworkConfiguration\x12\x1b\n\x13simulink_ip_address\x18\x03 \x01(\t\x12\x14\n\x0cmanager_port\x18\x04 \x01(\r\x12\x16\n\x0e\x61utomator_port\x18\x05 \x01(\r\x1a\xf3\x01\n\x14TimeoutConfiguration\x12\x42\n\x1fsimulink_start_timeout_duration\x18\x04 \x01(\x0b\x32\x19.google.protobuf.Duration\x12H\n%adp_send_to_simulink_timeout_duration\x18\x05 \x01(\x0b\x32\x19.google.protobuf.Duration\x12M\n*adp_receive_from_simulink_timeout_duration\x18\x06 \x01(\x0b\x32\x19.google.protobuf.Durationb\x06proto3'
  ,
  dependencies=[google_dot_protobuf_dot_duration__pb2.DESCRIPTOR,])




_SIMULINKCONFIG_AUTOMATIONCONFIGURATION = _descriptor.Descriptor(
  name='AutomationConfiguration',
  full_name='simian_public.proto.scenario.simulink_config.SimulinkConfig.AutomationConfiguration',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='matlab_root_absolute', full_name='simian_public.proto.scenario.simulink_config.SimulinkConfig.AutomationConfiguration.matlab_root_absolute', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='model_absolute_path', full_name='simian_public.proto.scenario.simulink_config.SimulinkConfig.AutomationConfiguration.model_absolute_path', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='model_deps_absolute', full_name='simian_public.proto.scenario.simulink_config.SimulinkConfig.AutomationConfiguration.model_deps_absolute', index=2,
      number=3, type=9, cpp_type=9, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='matlab_absolute_paths', full_name='simian_public.proto.scenario.simulink_config.SimulinkConfig.AutomationConfiguration.matlab_absolute_paths', index=3,
      number=4, type=9, cpp_type=9, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='model_init_scripts_filenames', full_name='simian_public.proto.scenario.simulink_config.SimulinkConfig.AutomationConfiguration.model_init_scripts_filenames', index=4,
      number=5, type=9, cpp_type=9, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='model_post_init_scripts_filenames', full_name='simian_public.proto.scenario.simulink_config.SimulinkConfig.AutomationConfiguration.model_post_init_scripts_filenames', index=5,
      number=6, type=9, cpp_type=9, label=3,
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
  serialized_start=540,
  serialized_end=765,
)

_SIMULINKCONFIG_NETWORKCONFIGURATION = _descriptor.Descriptor(
  name='NetworkConfiguration',
  full_name='simian_public.proto.scenario.simulink_config.SimulinkConfig.NetworkConfiguration',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='simulink_ip_address', full_name='simian_public.proto.scenario.simulink_config.SimulinkConfig.NetworkConfiguration.simulink_ip_address', index=0,
      number=3, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='manager_port', full_name='simian_public.proto.scenario.simulink_config.SimulinkConfig.NetworkConfiguration.manager_port', index=1,
      number=4, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='automator_port', full_name='simian_public.proto.scenario.simulink_config.SimulinkConfig.NetworkConfiguration.automator_port', index=2,
      number=5, type=13, cpp_type=3, label=1,
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
  serialized_start=767,
  serialized_end=864,
)

_SIMULINKCONFIG_TIMEOUTCONFIGURATION = _descriptor.Descriptor(
  name='TimeoutConfiguration',
  full_name='simian_public.proto.scenario.simulink_config.SimulinkConfig.TimeoutConfiguration',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='simulink_start_timeout_duration', full_name='simian_public.proto.scenario.simulink_config.SimulinkConfig.TimeoutConfiguration.simulink_start_timeout_duration', index=0,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='adp_send_to_simulink_timeout_duration', full_name='simian_public.proto.scenario.simulink_config.SimulinkConfig.TimeoutConfiguration.adp_send_to_simulink_timeout_duration', index=1,
      number=5, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='adp_receive_from_simulink_timeout_duration', full_name='simian_public.proto.scenario.simulink_config.SimulinkConfig.TimeoutConfiguration.adp_receive_from_simulink_timeout_duration', index=2,
      number=6, type=11, cpp_type=10, label=1,
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
  serialized_start=867,
  serialized_end=1110,
)

_SIMULINKCONFIG = _descriptor.Descriptor(
  name='SimulinkConfig',
  full_name='simian_public.proto.scenario.simulink_config.SimulinkConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='verbose', full_name='simian_public.proto.scenario.simulink_config.SimulinkConfig.verbose', index=0,
      number=1, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='pause_enabled', full_name='simian_public.proto.scenario.simulink_config.SimulinkConfig.pause_enabled', index=1,
      number=2, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='network_configuration', full_name='simian_public.proto.scenario.simulink_config.SimulinkConfig.network_configuration', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='timeout_configuration', full_name='simian_public.proto.scenario.simulink_config.SimulinkConfig.timeout_configuration', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='automation_configuration', full_name='simian_public.proto.scenario.simulink_config.SimulinkConfig.automation_configuration', index=4,
      number=5, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[_SIMULINKCONFIG_AUTOMATIONCONFIGURATION, _SIMULINKCONFIG_NETWORKCONFIGURATION, _SIMULINKCONFIG_TIMEOUTCONFIGURATION, ],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=133,
  serialized_end=1110,
)

_SIMULINKCONFIG_AUTOMATIONCONFIGURATION.containing_type = _SIMULINKCONFIG
_SIMULINKCONFIG_NETWORKCONFIGURATION.containing_type = _SIMULINKCONFIG
_SIMULINKCONFIG_TIMEOUTCONFIGURATION.fields_by_name['simulink_start_timeout_duration'].message_type = google_dot_protobuf_dot_duration__pb2._DURATION
_SIMULINKCONFIG_TIMEOUTCONFIGURATION.fields_by_name['adp_send_to_simulink_timeout_duration'].message_type = google_dot_protobuf_dot_duration__pb2._DURATION
_SIMULINKCONFIG_TIMEOUTCONFIGURATION.fields_by_name['adp_receive_from_simulink_timeout_duration'].message_type = google_dot_protobuf_dot_duration__pb2._DURATION
_SIMULINKCONFIG_TIMEOUTCONFIGURATION.containing_type = _SIMULINKCONFIG
_SIMULINKCONFIG.fields_by_name['network_configuration'].message_type = _SIMULINKCONFIG_NETWORKCONFIGURATION
_SIMULINKCONFIG.fields_by_name['timeout_configuration'].message_type = _SIMULINKCONFIG_TIMEOUTCONFIGURATION
_SIMULINKCONFIG.fields_by_name['automation_configuration'].message_type = _SIMULINKCONFIG_AUTOMATIONCONFIGURATION
DESCRIPTOR.message_types_by_name['SimulinkConfig'] = _SIMULINKCONFIG
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

SimulinkConfig = _reflection.GeneratedProtocolMessageType('SimulinkConfig', (_message.Message,), {

  'AutomationConfiguration' : _reflection.GeneratedProtocolMessageType('AutomationConfiguration', (_message.Message,), {
    'DESCRIPTOR' : _SIMULINKCONFIG_AUTOMATIONCONFIGURATION,
    '__module__' : 'simian.public.proto.scenario.simulink_config_pb2'
    # @@protoc_insertion_point(class_scope:simian_public.proto.scenario.simulink_config.SimulinkConfig.AutomationConfiguration)
    })
  ,

  'NetworkConfiguration' : _reflection.GeneratedProtocolMessageType('NetworkConfiguration', (_message.Message,), {
    'DESCRIPTOR' : _SIMULINKCONFIG_NETWORKCONFIGURATION,
    '__module__' : 'simian.public.proto.scenario.simulink_config_pb2'
    # @@protoc_insertion_point(class_scope:simian_public.proto.scenario.simulink_config.SimulinkConfig.NetworkConfiguration)
    })
  ,

  'TimeoutConfiguration' : _reflection.GeneratedProtocolMessageType('TimeoutConfiguration', (_message.Message,), {
    'DESCRIPTOR' : _SIMULINKCONFIG_TIMEOUTCONFIGURATION,
    '__module__' : 'simian.public.proto.scenario.simulink_config_pb2'
    # @@protoc_insertion_point(class_scope:simian_public.proto.scenario.simulink_config.SimulinkConfig.TimeoutConfiguration)
    })
  ,
  'DESCRIPTOR' : _SIMULINKCONFIG,
  '__module__' : 'simian.public.proto.scenario.simulink_config_pb2'
  # @@protoc_insertion_point(class_scope:simian_public.proto.scenario.simulink_config.SimulinkConfig)
  })
_sym_db.RegisterMessage(SimulinkConfig)
_sym_db.RegisterMessage(SimulinkConfig.AutomationConfiguration)
_sym_db.RegisterMessage(SimulinkConfig.NetworkConfiguration)
_sym_db.RegisterMessage(SimulinkConfig.TimeoutConfiguration)


# @@protoc_insertion_point(module_scope)