# Copyright (C) 2018 Applied Intuition, Inc. All rights reserved.
# This source code file is distributed under and subject to the LICENSE in license.txt

import argparse
from concurrent import futures
import contextlib
import functools
import os
import signal
import subprocess
import sys
import threading
import time
import traceback

from future.utils import raise_
from google.protobuf import json_format
from google.protobuf import timestamp_pb2
import grpc

from simian.public import constants
from simian.public.proto import common_pb2
from simian.public.proto import customer_service_pb2
from simian.public.proto import customer_service_pb2_grpc
from simian.public.proto import ego_pb2
from simian.public.proto import motion_model_pb2
from simian.public.proto import perception_pb2
from simian.public.proto import route_pb2
from simian.public.proto import sensor_model_pb2
from simian.public.proto import sensor_output_pb2
from simian.public.proto.v2 import customer_service_v2_pb2
from simian.public.proto.v2 import customer_service_v2_pb2_grpc
from simian.public.proto.v2 import io_pb2


@contextlib.contextmanager
def ignore_errors():
    try:
        yield
    except (KeyboardInterrupt, Exception):
        pass


def _loop_wait_clear_event(event):
    while not event.is_set():
        # Must have a timeout to allow Ctrl-C behavior to work properly in PY2.
        event.wait(1)
    event.clear()


def _catch_and_return_exception(response_type):
    def decorator(func):
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            try:
                response = func(*args, **kwargs)
                if response.common.status == common_pb2.CommonResponse.UNDEFINED:
                    response.common.status = common_pb2.CommonResponse.SUCCESS
                return response
            except Exception as e:
                return response_type(
                    common={
                        "status": common_pb2.CommonResponse.EXCEPTION,
                        "exception": {
                            "type_name": type(e).__name__,
                            "exception_message": str(e),
                            "traceback": traceback.format_exc(),
                        },
                    }
                )

        return wrapper

    return decorator


class StartupOptions:
    """StartupOptions holder for customer_service_pb2.CustomerStartupOptions."""

    def __init__(self, options):
        self._options = options
        self._parsed_scenario_extra_data = None

    @property
    def scenario_extra_data(self):
        # Cache the parsed scenario_extra_data on first access.
        if self._parsed_scenario_extra_data is None:
            self._parsed_scenario_extra_data = json_format.MessageToDict(
                self._options.scenario_extra_data
            )
        return self._parsed_scenario_extra_data

    def __getattr__(self, attr):
        return getattr(self._options, attr)

    def __str__(self):
        return str(self._options)

    def __repr__(self):
        return repr(self._options)


class CustomerStackServer(customer_service_pb2_grpc.CustomerStackServicer):
    def __init__(self, customer_interface_cls):
        self._customer_cls = customer_interface_cls
        self._process_handler = ProcessHandler()
        self._stack = None
        self._startup_options = None

    @_catch_and_return_exception(customer_service_pb2.GetStackInfoResponse)
    def GetStackInfo(self, request, _context):
        self._startup_options = StartupOptions(request.options)
        self._stack = self._customer_cls(self._process_handler, self._startup_options)
        clock_hz = self._stack.get_clock_hz()
        sim_hz = self._stack.get_simulation_hz()

        return customer_service_pb2.GetStackInfoResponse(
            stack_version=self._stack.get_stack_version(),
            simulation_hz=sim_hz,
            clock_hz=clock_hz,
            customer_server_version=constants.CUSTOMER_SERVER_VERSION,
        )

    @_catch_and_return_exception(customer_service_pb2.InitializeResponse)
    def Initialize(self, request, _context):
        if request.previous_status.middleware_running:
            self._stack.setup_middleware()  # type: ignore[union-attr] # ignore baseline; see #61427
        else:
            self._stack.teardown_middleware()  # type: ignore[union-attr] # ignore baseline; see #61427
            self._stack.setup_middleware()  # type: ignore[union-attr] # ignore baseline; see #61427
        self._stack.publish_time(request.sim_time.ToDatetime())  # type: ignore[union-attr] # ignore baseline; see #61427

        if self._startup_options.record_stack_data:  # type: ignore[union-attr] # ignore baseline; see #61427
            self._stack.start_recording(self._startup_options.record_stack_data)  # type: ignore # ignore baseline; see #61427
        if self._startup_options.start_stack_visualization:  # type: ignore[union-attr] # ignore baseline; see #61427
            self._stack.start_visualization()  # type: ignore[union-attr] # ignore baseline; see #61427
        self._stack.setup_stack()  # type: ignore[union-attr] # ignore baseline; see #61427
        self._stack.initialize(request.simulator_output, request.sim_time.ToDatetime())  # type: ignore[union-attr] # ignore baseline; see #61427
        return customer_service_pb2.InitializeResponse()

    @_catch_and_return_exception(customer_service_pb2.ReceiveSimulatorTimeResponse)
    def ReceiveSimulatorTime(self, request, _context):
        self._stack.publish_time(request.sim_time.ToDatetime())  # type: ignore[union-attr] # ignore baseline; see #61427
        return customer_service_pb2.ReceiveSimulatorTimeResponse()

    @_catch_and_return_exception(customer_service_pb2.ReceiveSimulatorOutputResponse)
    def ReceiveSimulatorOutput(self, request, _context):
        self._stack.convert_and_publish_sim_outputs(  # type: ignore[union-attr] # ignore baseline; see #61427
            request.simulator_output, request.sim_time.ToDatetime()
        )
        return customer_service_pb2.ReceiveSimulatorOutputResponse()

    @_catch_and_return_exception(customer_service_pb2.SendSimulatorInputResponse)
    def SendSimulatorInput(self, request, _context):  # noqa: ARG002
        simulator_input = self._stack.convert_and_return_listened_sim_inputs()  # type: ignore[union-attr] # ignore baseline; see #61427

        return customer_service_pb2.SendSimulatorInputResponse(simulator_input=simulator_input)

    @_catch_and_return_exception(customer_service_pb2.FinalizeResponse)
    def Finalize(self, request, _context):
        self._stack.finalize()  # type: ignore[union-attr] # ignore baseline; see #61427
        self._process_handler.teardown()
        if not request.requested_status.stack_running:
            self._stack.teardown_stack()  # type: ignore[union-attr] # ignore baseline; see #61427
        if not request.requested_status.middleware_running:
            self._stack.teardown_middleware()  # type: ignore[union-attr] # ignore baseline; see #61427
        if self._startup_options.record_stack_data:  # type: ignore[union-attr] # ignore baseline; see #61427
            self._stack.stop_recording()  # type: ignore[union-attr] # ignore baseline; see #61427
        if self._startup_options.start_stack_visualization:  # type: ignore[union-attr] # ignore baseline; see #61427
            self._stack.stop_visualization()  # type: ignore[union-attr] # ignore baseline; see #61427
        custom_observers = self._stack.get_final_observers()  # type: ignore[union-attr] # ignore baseline; see #61427
        return customer_service_pb2.FinalizeResponse(
            custom_observer_events=custom_observers,
            status=request.requested_status,
        )

    def Interrupted(self):
        # Ignore future interruptions so we can cleanup.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        # Each line must be a separate 'with ignore_errors()': so that the
        # subsequent line executes.
        with ignore_errors():
            self._stack.finalize()  # type: ignore[union-attr] # ignore baseline; see #61427
        with ignore_errors():
            self._process_handler.teardown()
        with ignore_errors():
            self._stack.teardown_stack()  # type: ignore[union-attr] # ignore baseline; see #61427
        with ignore_errors():
            self._stack.teardown_middleware()  # type: ignore[union-attr] # ignore baseline; see #61427
        with ignore_errors():
            if self._startup_options.record_stack_data:  # type: ignore[union-attr] # ignore baseline; see #61427
                self._stack.stop_recording()  # type: ignore[union-attr] # ignore baseline; see #61427
        with ignore_errors():
            if self._startup_options.start_stack_visualization:  # type: ignore[union-attr] # ignore baseline; see #61427
                self._stack.stop_visualization()  # type: ignore[union-attr] # ignore baseline; see #61427


class CustomerStackServerV2(customer_service_v2_pb2_grpc.CustomerServiceV2Servicer):
    no_arg_no_return_types = {
        customer_service_v2_pb2.CommandRequest.INITIALIZE,
        customer_service_v2_pb2.CommandRequest.FINALIZE,
        customer_service_v2_pb2.CommandRequest.MIDDLEWARE_SETUP,
        customer_service_v2_pb2.CommandRequest.MIDDLEWARE_TEARDOWN,
        customer_service_v2_pb2.CommandRequest.RECORDING_TEARDOWN,
        customer_service_v2_pb2.CommandRequest.STACK_SETUP,
        customer_service_v2_pb2.CommandRequest.STACK_TEARDOWN,
        customer_service_v2_pb2.CommandRequest.VISUALIZATION_SETUP,
        customer_service_v2_pb2.CommandRequest.VISUALIZATION_TEARDOWN,
    }
    channel_only_no_return_types = {
        customer_service_v2_pb2.CommandRequest.LISTEN_SETUP,
        customer_service_v2_pb2.CommandRequest.LISTEN_TEARDOWN,
        customer_service_v2_pb2.CommandRequest.PUBLISH_SETUP,
        customer_service_v2_pb2.CommandRequest.PUBLISH_TEARDOWN,
        customer_service_v2_pb2.CommandRequest.PUBLISH_SEND,
    }
    command_type_to_argument_proto = {
        customer_service_v2_pb2.CommandRequest.SET_STARTUP_OPTIONS: customer_service_pb2.CustomerStartupOptions,
        customer_service_v2_pb2.CommandRequest.SET_STARTUP_OPTIONS_V2_1: io_pb2.InterfaceStartupOptions,
        customer_service_v2_pb2.CommandRequest.LOG_OPEN: io_pb2.LogOpenOptions,
        customer_service_v2_pb2.CommandRequest.LOG_CLOSE: io_pb2.LogCloseOptions,
        customer_service_v2_pb2.CommandRequest.SIMULATION_SUMMARY: common_pb2.SimulationSummary,
    }
    channel_type_to_argument_proto = {
        io_pb2.TIME: timestamp_pb2.Timestamp,
        io_pb2.POSE: io_pb2.Pose,
        io_pb2.MOTION_FEEDBACK: motion_model_pb2.Feedback,
        io_pb2.CONTROLS: motion_model_pb2.Input,
        io_pb2.TRIGGER: io_pb2.Trigger,
        io_pb2.TRIP_AGENT: route_pb2.TripAgentOutput,
        io_pb2.STACK_STATE: io_pb2.StackState,
        io_pb2.OPERATOR_OVERRIDE_STATE: ego_pb2.OperatorOverrideState,
        io_pb2.ACTORS: perception_pb2.PerceptionChannel.ActorSensor,
        io_pb2.ACTORS_2D: perception_pb2.PerceptionChannel.Actor2DSensor,
        io_pb2.LANE_SENSOR: perception_pb2.PerceptionChannel.LaneSensor,
        io_pb2.TRAFFIC_LIGHTS: perception_pb2.PerceptionChannel.TrafficLightSensor,
        io_pb2.LOCALIZATION_SENSOR: perception_pb2.PerceptionChannel.LocalizationSensor,
        io_pb2.PLANAR_LIDAR: perception_pb2.PerceptionChannel.PlanarLidarSensor,
        io_pb2.PLANAR_OCCUPANCY_GRID: perception_pb2.PerceptionChannel.PlanarOccupancyGridSensor,
        io_pb2.OCCLUSION_GRID: perception_pb2.PerceptionChannel.OcclusionGridSensor,
        io_pb2.FREE_SPACE_SENSOR: perception_pb2.PerceptionChannel.FreeSpaceSensor,
        io_pb2.TRAFFIC_LIGHT_BLOCKS: perception_pb2.PerceptionChannel.TrafficLightBlockSensor,
        io_pb2.TRAFFIC_SIGN_SENSOR: perception_pb2.PerceptionChannel.TrafficSignSensor,
        io_pb2.IMU_SENSOR: perception_pb2.PerceptionChannel.IMUSensor,
        io_pb2.WHEEL_SPEED_SENSOR: perception_pb2.PerceptionChannel.WheelSpeedSensor,
        io_pb2.MAP_SENSOR: perception_pb2.PerceptionChannel.MapSensor,
        io_pb2.TERRAIN_SENSOR: perception_pb2.PerceptionChannel.TerrainSensor,
        io_pb2.WIND_SENSOR: perception_pb2.PerceptionChannel.WindSensor,
        io_pb2.POLAR_OBSTACLE_SENSOR: perception_pb2.PerceptionChannel.PolarObstacleSensor,
        io_pb2.AGENT_TRAJECTORY_SENSOR: perception_pb2.PerceptionChannel.AgentTrajectorySensor,
        io_pb2.LIDAR: sensor_model_pb2.SensorOutput.LidarCloud,
        io_pb2.RADAR: sensor_model_pb2.SensorOutput.RadarTrack,
        io_pb2.CAMERA: sensor_model_pb2.SensorOutput.CameraImage,
        io_pb2.ULTRASOUND: sensor_model_pb2.SensorOutput.Range,
        io_pb2.PERCEPTION_SENSOR: sensor_output_pb2.SensorOutputList,
        io_pb2.VEHICLE_DATA: io_pb2.VehicleData,
    }

    def __init__(self, customer_interface_cls):
        self._customer_interface_cls = customer_interface_cls
        self._process_handler = ProcessHandler()
        self._stack_interfaces = {}
        self._startup_options = None
        self._finalize_commands = []
        self._deferred_destroys = set()

    @_catch_and_return_exception(customer_service_v2_pb2.CommandsResponse)
    def Commands(self, request, _context):
        return self.handle_commands(request.command, False)

    @_catch_and_return_exception(customer_service_v2_pb2.FinalizeResponse)
    def Finalize(self, _request, _context):
        return self.handle_commands(reversed(self._finalize_commands), True)

    @_catch_and_return_exception(customer_service_v2_pb2.FinalizeResponse)
    def Interrupted(self):
        print("Interrupted, trying to clean up now")
        return self.handle_commands(reversed(self._finalize_commands), True)

    @_catch_and_return_exception(customer_service_v2_pb2.GetStackServerInfoResponse)
    def GetStackServerInfo(self, _request, _context):
        resp = customer_service_v2_pb2.GetStackServerInfoResponse()
        resp.argv.extend(sys.argv)
        return resp

    def handle_commands(self, commands, continue_on_error):
        response = customer_service_v2_pb2.CommandsResponse()
        all_commands_successful = True
        explanation = ""
        for command in commands:
            if (
                command.finalize_command.command_type
                != customer_service_v2_pb2.CommandRequest.__UNDEFINED__
            ):
                self._finalize_commands.append(command.finalize_command)

            try:
                resp = self.handle_command(command)
                response.response.add().CopyFrom(resp)
            except Exception as e:
                all_commands_successful = False
                explanation += traceback.format_exc()
                resp = response.response.add()
                resp.common.status = common_pb2.CommonResponse.EXCEPTION
                resp.common.exception.type_name = "Python exception"
                resp.common.exception.exception_message = str(e).encode()
                if not continue_on_error:
                    break

        # Collect extras from all targets
        for target, stack in self._stack_interfaces.items():
            extras = stack.collect_extras()
            response.extras[target].MergeFrom(extras)
            stack.collect_extras_free()

        # We defer destroying interfaces until after we've pulled Extra data out of them
        for target in self._deferred_destroys:
            del self._stack_interfaces[target]
        self._deferred_destroys = set()

        if all_commands_successful:
            response.common.status = common_pb2.CommonResponse.SUCCESS
        else:
            response.common.status = common_pb2.CommonResponse.EXCEPTION
            response.common.exception.exception_message = (
                "At least 1 command was unsuccessful: (multi-rate Python server) {}".format(
                    explanation
                )
            )
        return response

    def handle_command(self, command):
        CommandType = customer_service_v2_pb2.CommandRequest
        command_name = CommandType.CommandType.Name(command.command_type).lower()
        stack = self._stack_interfaces.get(command.target)

        response = customer_service_v2_pb2.CommandResponse()
        response.command_type = command.command_type
        response.target = command.target
        response.channel_name = command.channel.name

        if command.command_type in self.no_arg_no_return_types:
            getattr(stack, command_name)()
        elif command.command_type in self.channel_only_no_return_types:
            getattr(stack, command_name)(command.channel)
        elif command.command_type in self.command_type_to_argument_proto:
            proto = self.command_type_to_argument_proto[command.command_type]()
            proto.ParseFromString(command.data)
            getattr(stack, command_name)(proto)
        elif command.command_type == CommandType.CREATE:
            self._stack_interfaces[command.target] = self._customer_interface_cls(
                self._process_handler
            )
        elif command.command_type == CommandType.DESTROY:
            self._deferred_destroys.add(command.target)
        elif command.command_type == CommandType.GET_SERVER_VERSION:
            response.data = str(constants.CUSTOMER_SERVER_VERSION).encode()
        elif command.command_type == CommandType.GET_STACK_VERSION:
            response.data = str(stack.get_stack_version()).encode()  # type: ignore[union-attr] # ignore baseline; see #61427
        elif command.command_type == CommandType.GET_INTERFACE_VERSION:
            if hasattr(stack, "get_interface_version"):
                response.data = str(stack.get_interface_version()).encode()  # type: ignore[union-attr] # ignore baseline; see #61427
            else:
                response.data = str(4).encode()  # CUSTOMER_INTERFACE_VERSION_2_2
        elif command.command_type == CommandType.RECORDING_SETUP:
            recording_path = command.data.decode("utf-8")
            stack.recording_setup(recording_path)  # type: ignore[union-attr] # ignore baseline; see #61427
        elif command.command_type in (
            CommandType.GET_DEFAULT_RATE,
            CommandType.GET_DEFAULT_PERIOD_NS,
        ):
            rate_or_period = getattr(stack, command_name)(command.channel)
            if rate_or_period == 0:
                response.data = b""
            else:
                response.data = str(rate_or_period).encode()
        elif command.command_type == CommandType.CONVERT_FROM_SIMIAN:
            proto = self.channel_type_to_argument_proto[command.channel.type]()
            proto.ParseFromString(command.data)
            stack.convert_from_simian(command.channel, proto)  # type: ignore[union-attr] # ignore baseline; see #61427
        elif command.command_type == CommandType.CONVERT_TO_SIMIAN:
            if command.channel.type != io_pb2.CUSTOM:
                response.data = stack.convert_to_simian(command.channel).SerializeToString()  # type: ignore[union-attr] # ignore baseline; see #61427
                stack.convert_to_simian_free(command.channel)  # type: ignore[union-attr] # ignore baseline; see #61427
        elif command.command_type == CommandType.LOG_READ_V2_1:
            log_read_options = io_pb2.LogReadOptions()
            log_read_options.ParseFromString(command.data)
            response.data = stack.log_read_v2_1(log_read_options).SerializeToString()  # type: ignore[union-attr] # ignore baseline; see #61427
        elif command.command_type == CommandType.LOG_OPEN_V2_2:
            log_open_options = io_pb2.LogOpenOptions()
            log_open_options.ParseFromString(command.data)
            response.data = stack.log_open_v2_2(log_open_options).SerializeToString()  # type: ignore[union-attr] # ignore baseline; see #61427
        elif command.command_type == CommandType.PATCH:
            patch_options = io_pb2.PatchOptions()
            patch_options.ParseFromString(command.data)
            stack.patch(command.channel, patch_options)  # type: ignore[union-attr] # ignore baseline; see #61427
        elif command.command_type == CommandType.LOG_FETCH:
            log_fetch_options = io_pb2.LogFetchOptions()
            log_fetch_options.ParseFromString(command.data)
            log_fetch_output = stack.log_fetch(log_fetch_options)  # type: ignore[union-attr] # ignore baseline; see #61427
            if log_fetch_output is not None:
                response.data = log_fetch_output.SerializeToString()
        elif command.command_type == CommandType.LOG_REWIND:
            log_rewind_options = io_pb2.LogRewindOptions()
            log_rewind_options.ParseFromString(command.data)
            log_rewind_output = stack.log_rewind(log_rewind_options)  # type: ignore[union-attr] # ignore baseline; see #61427
            if log_rewind_output is not None:
                response.data = log_rewind_output.SerializeToString()
        else:
            name = CommandType.CommandType.Name(command.command_type)
            raise Exception("Not implemented for: {}".format(name))

        response.common.status = common_pb2.CommonResponse.SUCCESS
        return response


class _MainThreadReflector:
    """Reflects certain functions on a class into the main thread."""

    def __init__(self, cls, main_thread_funcs, reflect_all):
        self._cls = cls
        self._main_thread_funcs = main_thread_funcs
        self._reflect_all = reflect_all
        self._instance = None
        # Only one call is allowed at a time.
        self._call_ready = threading.Event()
        self._result_ready = threading.Event()
        self._call = None
        self._result = None
        self._is_error = False

    def __call__(self, *args, **kwargs):
        if "__init__" not in self._main_thread_funcs:
            self._instance = self._cls(*args, **kwargs)
        else:
            self._instance = self.__create_callable(self._cls)(*args, **kwargs)
        return self

    def __create_callable(self, func):
        def attr_callable(*args, **kwargs):
            self._call = (func, args, kwargs)
            self._call_ready.set()
            _loop_wait_clear_event(self._result_ready)
            if self._is_error:
                raise_(self._result[0], self._result[1], self._result[2])  # type: ignore[index] # ignore baseline; see #61427
            return self._result

        return attr_callable

    def __getattr__(self, attr):
        if isinstance(threading.current_thread(), threading._MainThread):  # type: ignore[attr-defined] # ignore baseline; see #61427
            # If we're already in the main thread, just call directly.
            return getattr(self._instance, attr)

        if attr not in self._main_thread_funcs and not self._reflect_all:
            return getattr(self._instance, attr)

        def attr_callable(*args, **kwargs):
            func = getattr(self._instance, attr)
            return self.__create_callable(func)(*args, **kwargs)

        return attr_callable

    def reflector_has_instance(self):
        return self._instance is not None

    def reflector_run_in_main_thread(self):
        while True:
            # Must loop-wait instead of wait without a timeout to allow Ctrl-C
            # to happen here.
            _loop_wait_clear_event(self._call_ready)

            func, args, kwargs = self._call  # type: ignore[misc] # ignore baseline; see #61427
            try:
                self._result = func(*args, **kwargs)
                self._is_error = False
            except KeyboardInterrupt:
                print("Ctrl-C!")
                self._result = sys.exc_info()
                self._is_error = True
                raise
            except Exception:
                self._result = sys.exc_info()
                self._is_error = True
            finally:
                self._result_ready.set()


class ProcessHandler:
    def __init__(self):
        self._procs = []

    def spawn(self, *args, **kwargs):
        proc = subprocess.Popen(*args, **kwargs)
        self._procs.append(proc)
        return proc

    def teardown(self):
        for proc in self._procs:
            proc.send_signal(signal.SIGINT)
        # Only sleep once if any process is alive
        if any(proc.poll() is None for proc in self._procs):
            time.sleep(1)
        for proc in self._procs:
            if proc.poll() is None:
                proc.terminate()
        if any(proc.poll() is None for proc in self._procs):
            time.sleep(0.1)
        for proc in self._procs:
            proc.wait()


def create_argparser():
    """Creates the argparser with our flags.

    Users of this can either add their flags to this parser, call
    parse_known_args() to handle unknown flags, or ignore this to use the
    default behavior.

    Example of adding flags:
        parser = customer_stack_server.create_argparser()
        parser.add_argument('--fake_flag')
        args = parser.parse_args()
        customer_stack_server.start_server(CustomerInterface, args)
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True, type=int)
    parser.add_argument("--pidfile", required=True, type=argparse.FileType("w"))
    parser.add_argument("--v2api", action="store_true")
    return parser


def start_server(customer_interface_cls, args=None, main_thread_required=False):
    # Reopen to avoid buffering the output.
    sys.stdout.flush()
    sys.stderr.flush()
    sys.stdout = os.fdopen(os.dup(sys.stdout.fileno()), sys.stdout.mode, 1)  # type: ignore[assignment] # ignore baseline; see #61427
    sys.stderr = os.fdopen(os.dup(sys.stderr.fileno()), sys.stderr.mode, 1)  # type: ignore[assignment] # ignore baseline; see #61427

    if not args:
        args = create_argparser().parse_args()

    args.pidfile.write(str(os.getpid()))
    args.pidfile.close()

    server_options = [
        ("grpc.max_receive_message_length", 200 * 1024 * 1024),
        ("grpc.max_send_message_length", 200 * 1024 * 1024),
    ]
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=1), options=server_options)
    if not args.v2api:
        reflector = _MainThreadReflector(
            customer_interface_cls,
            {
                "__init__",
                "setup_middleware",
                "teardown_middleware",
                "setup_stack",
                "initialize",
                "teardown_stack",
                "finalize",
            },
            reflect_all=main_thread_required,
        )
        stack_server = CustomerStackServer(reflector)
        customer_service_pb2_grpc.add_CustomerStackServicer_to_server(stack_server, server)
    else:
        reflector = _MainThreadReflector(
            customer_interface_cls,
            {
                "__init__",
                "create",
                "destroy",
                "set_startup_options",
                "get_stack_version",
                "initialize",
                "finalize",
                "get_default_rate",
                "get_default_period",
                "convert_to_simian",
                "log_open",
                "log_close",
            },
            reflect_all=main_thread_required,
        )
        stack_server = CustomerStackServerV2(reflector)  # type: ignore # ignore mypy-protobuf-baseline
        customer_service_v2_pb2_grpc.add_CustomerServiceV2Servicer_to_server(stack_server, server)  # type: ignore[arg-type] # ignore mypy-protobuf-baseline
    try:
        server.add_insecure_port("[::]:%d" % args.port)
        server.start()
        reflector.reflector_run_in_main_thread()
    except (KeyboardInterrupt, SystemExit):
        server.stop(0)
        stack_server.Interrupted()
