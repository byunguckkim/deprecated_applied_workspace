# Generated by the gRPC Python protocol compiler plugin. DO NOT EDIT!
"""Client and server classes corresponding to protobuf-defined services."""
import grpc

from grpc_reflection.v1alpha import reflection_pb2 as grpc__reflection_dot_v1alpha_dot_reflection__pb2


class ServerReflectionStub(object):
    """Missing associated documentation comment in .proto file."""

    def __init__(self, channel):
        """Constructor.

        Args:
            channel: A grpc.Channel.
        """
        self.ServerReflectionInfo = channel.stream_stream(
                '/grpc.reflection.v1alpha.ServerReflection/ServerReflectionInfo',
                request_serializer=grpc__reflection_dot_v1alpha_dot_reflection__pb2.ServerReflectionRequest.SerializeToString,
                response_deserializer=grpc__reflection_dot_v1alpha_dot_reflection__pb2.ServerReflectionResponse.FromString,
                )


class ServerReflectionServicer(object):
    """Missing associated documentation comment in .proto file."""

    def ServerReflectionInfo(self, request_iterator, context):
        """The reflection service is structured as a bidirectional stream, ensuring
        all related requests go to a single server.
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')


def add_ServerReflectionServicer_to_server(servicer, server):
    rpc_method_handlers = {
            'ServerReflectionInfo': grpc.stream_stream_rpc_method_handler(
                    servicer.ServerReflectionInfo,
                    request_deserializer=grpc__reflection_dot_v1alpha_dot_reflection__pb2.ServerReflectionRequest.FromString,
                    response_serializer=grpc__reflection_dot_v1alpha_dot_reflection__pb2.ServerReflectionResponse.SerializeToString,
            ),
    }
    generic_handler = grpc.method_handlers_generic_handler(
            'grpc.reflection.v1alpha.ServerReflection', rpc_method_handlers)
    server.add_generic_rpc_handlers((generic_handler,))


 # This class is part of an EXPERIMENTAL API.
class ServerReflection(object):
    """Missing associated documentation comment in .proto file."""

    @staticmethod
    def ServerReflectionInfo(request_iterator,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.stream_stream(request_iterator, target, '/grpc.reflection.v1alpha.ServerReflection/ServerReflectionInfo',
            grpc__reflection_dot_v1alpha_dot_reflection__pb2.ServerReflectionRequest.SerializeToString,
            grpc__reflection_dot_v1alpha_dot_reflection__pb2.ServerReflectionResponse.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)
