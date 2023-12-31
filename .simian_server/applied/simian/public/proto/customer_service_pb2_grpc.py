# Generated by the gRPC Python protocol compiler plugin. DO NOT EDIT!
"""Client and server classes corresponding to protobuf-defined services."""
import grpc

from simian.public.proto import customer_service_pb2 as simian_dot_public_dot_proto_dot_customer__service__pb2


class CustomerStackStub(object):
    """Missing associated documentation comment in .proto file."""

    def __init__(self, channel):
        """Constructor.

        Args:
            channel: A grpc.Channel.
        """
        self.GetStackInfo = channel.unary_unary(
                '/simian_public.simulator.CustomerStack/GetStackInfo',
                request_serializer=simian_dot_public_dot_proto_dot_customer__service__pb2.GetStackInfoRequest.SerializeToString,
                response_deserializer=simian_dot_public_dot_proto_dot_customer__service__pb2.GetStackInfoResponse.FromString,
                )
        self.Initialize = channel.unary_unary(
                '/simian_public.simulator.CustomerStack/Initialize',
                request_serializer=simian_dot_public_dot_proto_dot_customer__service__pb2.InitializeRequest.SerializeToString,
                response_deserializer=simian_dot_public_dot_proto_dot_customer__service__pb2.InitializeResponse.FromString,
                )
        self.ReceiveSimulatorTime = channel.unary_unary(
                '/simian_public.simulator.CustomerStack/ReceiveSimulatorTime',
                request_serializer=simian_dot_public_dot_proto_dot_customer__service__pb2.ReceiveSimulatorTimeRequest.SerializeToString,
                response_deserializer=simian_dot_public_dot_proto_dot_customer__service__pb2.ReceiveSimulatorTimeResponse.FromString,
                )
        self.ReceiveSimulatorOutput = channel.unary_unary(
                '/simian_public.simulator.CustomerStack/ReceiveSimulatorOutput',
                request_serializer=simian_dot_public_dot_proto_dot_customer__service__pb2.ReceiveSimulatorOutputRequest.SerializeToString,
                response_deserializer=simian_dot_public_dot_proto_dot_customer__service__pb2.ReceiveSimulatorOutputResponse.FromString,
                )
        self.SendSimulatorInput = channel.unary_unary(
                '/simian_public.simulator.CustomerStack/SendSimulatorInput',
                request_serializer=simian_dot_public_dot_proto_dot_customer__service__pb2.SendSimulatorInputRequest.SerializeToString,
                response_deserializer=simian_dot_public_dot_proto_dot_customer__service__pb2.SendSimulatorInputResponse.FromString,
                )
        self.Finalize = channel.unary_unary(
                '/simian_public.simulator.CustomerStack/Finalize',
                request_serializer=simian_dot_public_dot_proto_dot_customer__service__pb2.FinalizeRequest.SerializeToString,
                response_deserializer=simian_dot_public_dot_proto_dot_customer__service__pb2.FinalizeResponse.FromString,
                )


class CustomerStackServicer(object):
    """Missing associated documentation comment in .proto file."""

    def GetStackInfo(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def Initialize(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def ReceiveSimulatorTime(self, request, context):
        """<!-- TODO(fahhem): Support deterministic sim (where the customer stack knows
        when processing is finished and can return their response right away
        without us tracking real time.
        rpc AdvanceDeterministically (stream AdvanceRequest) returns (stream AdvanceResponse); -->
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def ReceiveSimulatorOutput(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def SendSimulatorInput(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def Finalize(self, request, context):
        """Missing associated documentation comment in .proto file."""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')


def add_CustomerStackServicer_to_server(servicer, server):
    rpc_method_handlers = {
            'GetStackInfo': grpc.unary_unary_rpc_method_handler(
                    servicer.GetStackInfo,
                    request_deserializer=simian_dot_public_dot_proto_dot_customer__service__pb2.GetStackInfoRequest.FromString,
                    response_serializer=simian_dot_public_dot_proto_dot_customer__service__pb2.GetStackInfoResponse.SerializeToString,
            ),
            'Initialize': grpc.unary_unary_rpc_method_handler(
                    servicer.Initialize,
                    request_deserializer=simian_dot_public_dot_proto_dot_customer__service__pb2.InitializeRequest.FromString,
                    response_serializer=simian_dot_public_dot_proto_dot_customer__service__pb2.InitializeResponse.SerializeToString,
            ),
            'ReceiveSimulatorTime': grpc.unary_unary_rpc_method_handler(
                    servicer.ReceiveSimulatorTime,
                    request_deserializer=simian_dot_public_dot_proto_dot_customer__service__pb2.ReceiveSimulatorTimeRequest.FromString,
                    response_serializer=simian_dot_public_dot_proto_dot_customer__service__pb2.ReceiveSimulatorTimeResponse.SerializeToString,
            ),
            'ReceiveSimulatorOutput': grpc.unary_unary_rpc_method_handler(
                    servicer.ReceiveSimulatorOutput,
                    request_deserializer=simian_dot_public_dot_proto_dot_customer__service__pb2.ReceiveSimulatorOutputRequest.FromString,
                    response_serializer=simian_dot_public_dot_proto_dot_customer__service__pb2.ReceiveSimulatorOutputResponse.SerializeToString,
            ),
            'SendSimulatorInput': grpc.unary_unary_rpc_method_handler(
                    servicer.SendSimulatorInput,
                    request_deserializer=simian_dot_public_dot_proto_dot_customer__service__pb2.SendSimulatorInputRequest.FromString,
                    response_serializer=simian_dot_public_dot_proto_dot_customer__service__pb2.SendSimulatorInputResponse.SerializeToString,
            ),
            'Finalize': grpc.unary_unary_rpc_method_handler(
                    servicer.Finalize,
                    request_deserializer=simian_dot_public_dot_proto_dot_customer__service__pb2.FinalizeRequest.FromString,
                    response_serializer=simian_dot_public_dot_proto_dot_customer__service__pb2.FinalizeResponse.SerializeToString,
            ),
    }
    generic_handler = grpc.method_handlers_generic_handler(
            'simian_public.simulator.CustomerStack', rpc_method_handlers)
    server.add_generic_rpc_handlers((generic_handler,))


 # This class is part of an EXPERIMENTAL API.
class CustomerStack(object):
    """Missing associated documentation comment in .proto file."""

    @staticmethod
    def GetStackInfo(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/simian_public.simulator.CustomerStack/GetStackInfo',
            simian_dot_public_dot_proto_dot_customer__service__pb2.GetStackInfoRequest.SerializeToString,
            simian_dot_public_dot_proto_dot_customer__service__pb2.GetStackInfoResponse.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def Initialize(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/simian_public.simulator.CustomerStack/Initialize',
            simian_dot_public_dot_proto_dot_customer__service__pb2.InitializeRequest.SerializeToString,
            simian_dot_public_dot_proto_dot_customer__service__pb2.InitializeResponse.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def ReceiveSimulatorTime(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/simian_public.simulator.CustomerStack/ReceiveSimulatorTime',
            simian_dot_public_dot_proto_dot_customer__service__pb2.ReceiveSimulatorTimeRequest.SerializeToString,
            simian_dot_public_dot_proto_dot_customer__service__pb2.ReceiveSimulatorTimeResponse.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def ReceiveSimulatorOutput(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/simian_public.simulator.CustomerStack/ReceiveSimulatorOutput',
            simian_dot_public_dot_proto_dot_customer__service__pb2.ReceiveSimulatorOutputRequest.SerializeToString,
            simian_dot_public_dot_proto_dot_customer__service__pb2.ReceiveSimulatorOutputResponse.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def SendSimulatorInput(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/simian_public.simulator.CustomerStack/SendSimulatorInput',
            simian_dot_public_dot_proto_dot_customer__service__pb2.SendSimulatorInputRequest.SerializeToString,
            simian_dot_public_dot_proto_dot_customer__service__pb2.SendSimulatorInputResponse.FromString,
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
        return grpc.experimental.unary_unary(request, target, '/simian_public.simulator.CustomerStack/Finalize',
            simian_dot_public_dot_proto_dot_customer__service__pb2.FinalizeRequest.SerializeToString,
            simian_dot_public_dot_proto_dot_customer__service__pb2.FinalizeResponse.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)
