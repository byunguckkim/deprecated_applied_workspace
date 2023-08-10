# Boilerplate added by tool/subpar_compiler.py
from applied.tools.py.par_setup import setup as _
_(import_roots=['pypi__39__setuptools_65_5_1', 'com_google_protobuf/python', 'pypi__39__numpy_1_21_6', 'pypi__39__six_1_16_0', 'pypi__39__grpcio_1_42_0', 'pypi__39__grpcio_reflection_1_42_0', 'pypi__39__future_0_18_3', 'applied', 'bazel_tools', 'com_github_gflags_gflags', 'com_google_protobuf', 'cython_package', 'glog', 'pb_absl', 'pypi__39__future_0_18_3', 'pypi__39__grpcio_1_42_0', 'pypi__39__grpcio_reflection_1_42_0', 'pypi__39__numpy_1_21_6', 'pypi__39__setuptools_65_5_1', 'pypi__39__six_1_16_0', 'six'], zip_safe=True, remove=True)
del _
# End boilerplate
from customer.apollo.sim.scripts import customer_interface
from simian.public import customer_stack_server

if __name__ == "__main__":
    customer_stack_server.start_server(customer_interface.ApolloInterface)
