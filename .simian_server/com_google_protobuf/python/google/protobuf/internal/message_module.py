from google.protobuf.internal._message_module import api_implementation, global_dlopen
with global_dlopen():
  if api_implementation.Type() == 'cpp':
    from google.protobuf.pyext import cpp_message as message_impl
  else:
    from google.protobuf.internal import python_message as message_impl
