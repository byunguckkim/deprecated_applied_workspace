# Generated by the gRPC Python protocol compiler plugin. DO NOT EDIT!
"""Client and server classes corresponding to protobuf-defined services."""
import grpc

from simian.public.proto.v2 import customer_service_v2_pb2 as simian_dot_public_dot_proto_dot_v2_dot_customer__service__v2__pb2


class CustomerServiceV2Stub(object):
    """Missing associated documentation comment in .proto file."""

    def __init__(self, channel):
        """Constructor.

        Args:
            channel: A grpc.Channel.
        """
        self.Commands = channel.unary_unary(
                '/simian_public.simulator.v2.CustomerServiceV2/Commands',
                request_serializer=simian_dot_public_dot_proto_dot_v2_dot_customer__service__v2__pb2.CommandsRequest.SerializeToString,
                response_deserializer=simian_dot_public_dot_proto_dot_v2_dot_customer__service__v2__pb2.CommandsResponse.FromString,
                )
        self.StoreFinalizeCommands = channel.unary_unary(
                '/simian_public.simulator.v2.CustomerServiceV2/StoreFinalizeCommands',
                request_serializer=simian_dot_public_dot_proto_dot_v2_dot_customer__service__v2__pb2.StoreFinalizeCommandsRequest.SerializeToString,
                response_deserializer=simian_dot_public_dot_proto_dot_v2_dot_customer__service__v2__pb2.StoreFinalizeCommandsResponse.FromString,
                )
        self.Finalize = channel.unary_unary(
                '/simian_public.simulator.v2.CustomerServiceV2/Finalize',
                request_serializer=simian_dot_public_dot_proto_dot_v2_dot_customer__service__v2__pb2.FinalizeRequest.SerializeToString,
                response_deserializer=simian_dot_public_dot_proto_dot_v2_dot_customer__service__v2__pb2.FinalizeResponse.FromString,
                )
        self.GetStackServerInfo = channel.unary_unary(
                '/simian_public.simulator.v2.CustomerServiceV2/GetStackServerInfo',
                request_serializer=simian_dot_public_dot_proto_dot_v2_dot_customer__service__v2__pb2.GetStackServerInfoRequest.SerializeToString,
                response_deserializer=simian_dot_public_dot_proto_dot_v2_dot_customer__service__v2__pb2.GetStackServerInfoResponse.FromString,
                )


class CustomerServiceV2Servicer(object):
    """Missing associated documentation comment in .proto file."""

    def Commands(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def StoreFinalizeCommands(self, request, context):
        """DEPRECATED: Only for backwards compatibility with v1.11 and below. Will
        remove in v1.13 or later.
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def Finalize(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def GetStackServerInfo(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')


def add_CustomerServiceV2Servicer_to_server(servicer, server):
    rpc_method_handlers = {
            'Commands': grpc.unary_unary_rpc_method_handler(
                    servicer.Commands,
                    request_deserializer=simian_dot_public_dot_proto_dot_v2_dot_customer__service__v2__pb2.CommandsRequest.FromString,
                    response_serializer=simian_dot_public_dot_proto_dot_v2_dot_customer__service__v2__pb2.CommandsResponse.SerializeToString,
            ),
            'StoreFinalizeCommands': grpc.unary_unary_rpc_method_handler(
                    servicer.StoreFinalizeCommands,
                    request_deserializer=simian_dot_public_dot_proto_dot_v2_dot_customer__service__v2__pb2.StoreFinalizeCommandsRequest.FromString,
                    response_serializer=simian_dot_public_dot_proto_dot_v2_dot_customer__service__v2__pb2.StoreFinalizeCommandsResponse.SerializeToString,
            ),
            'Finalize': grpc.unary_unary_rpc_method_handler(
                    servicer.Finalize,
                    request_deserializer=simian_dot_public_dot_proto_dot_v2_dot_customer__service__v2__pb2.FinalizeRequest.FromString,
                    response_serializer=simian_dot_public_dot_proto_dot_v2_dot_customer__service__v2__pb2.FinalizeResponse.SerializeToString,
            ),
            'GetStackServerInfo': grpc.unary_unary_rpc_method_handler(
                    servicer.GetStackServerInfo,
                    request_deserializer=simian_dot_public_dot_proto_dot_v2_dot_customer__service__v2__pb2.GetStackServerInfoRequest.FromString,
                    response_serializer=simian_dot_public_dot_proto_dot_v2_dot_customer__service__v2__pb2.GetStackServerInfoResponse.SerializeToString,
            ),
    }
    generic_handler = grpc.method_handlers_generic_handler(
            'simian_public.simulator.v2.CustomerServiceV2', rpc_method_handlers)
    server.add_generic_rpc_handlers((generic_handler,))


 # This class is part of an EXPERIMENTAL API.
class CustomerServiceV2(object):
    """Missing associated documentation comment in .proto file."""

    @staticmethod
    def Commands(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/simian_public.simulator.v2.CustomerServiceV2/Commands',
            simian_dot_public_dot_proto_dot_v2_dot_customer__service__v2__pb2.CommandsRequest.SerializeToString,
            simian_dot_public_dot_proto_dot_v2_dot_customer__service__v2__pb2.CommandsResponse.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def StoreFinalizeCommands(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/simian_public.simulator.v2.CustomerServiceV2/StoreFinalizeCommands',
            simian_dot_public_dot_proto_dot_v2_dot_customer__service__v2__pb2.StoreFinalizeCommandsRequest.SerializeToString,
            simian_dot_public_dot_proto_dot_v2_dot_customer__service__v2__pb2.StoreFinalizeCommandsResponse.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def Finalize(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/simian_public.simulator.v2.CustomerServiceV2/Finalize',
            simian_dot_public_dot_proto_dot_v2_dot_customer__service__v2__pb2.FinalizeRequest.SerializeToString,
            simian_dot_public_dot_proto_dot_v2_dot_customer__service__v2__pb2.FinalizeResponse.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def GetStackServerInfo(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/simian_public.simulator.v2.CustomerServiceV2/GetStackServerInfo',
            simian_dot_public_dot_proto_dot_v2_dot_customer__service__v2__pb2.GetStackServerInfoRequest.SerializeToString,
            simian_dot_public_dot_proto_dot_v2_dot_customer__service__v2__pb2.GetStackServerInfoResponse.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)
