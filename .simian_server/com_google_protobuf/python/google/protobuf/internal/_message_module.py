
try:
    from tools.pybind import pybind11_loader
    global_dlopen = pybind11_loader.global_dlopen
except ImportError:
    import contextlib
    global_dlopen = contextlib.contextmanager(lambda: (yield None))

with global_dlopen():

    from google.protobuf.internal import api_implementation

    if api_implementation.Type() == 'cpp':
      from google.protobuf.pyext import _message
      _USE_C_DESCRIPTORS = True
    else:
      _message = None
      _USE_C_DESCRIPTORS = False

