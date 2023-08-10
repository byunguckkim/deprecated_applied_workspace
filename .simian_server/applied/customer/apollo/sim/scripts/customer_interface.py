import datetime
import os
import subprocess
import time

# Apollo-internal
from modules.canbus.proto import chassis_pb2
from modules.common.proto import error_code_pb2
from modules.control.proto import control_cmd_pb2
from modules.control.proto import pad_msg_pb2
from modules.localization.proto import localization_pb2
from modules.perception.proto import perception_obstacle_pb2
from modules.perception.proto import traffic_light_detection_pb2
from modules.planning.proto import planning_pb2
from modules.routing.proto import routing_pb2
import rospy

# Simian-Apollo conversion
from customer.apollo.converters import convert_actors
from customer.apollo.converters import convert_ego
from customer.apollo.converters import convert_traffic_lights

# Simian integration
from simian.public import motion_command_sequencer
from simian.public import stack_interface_ros
from simian.public import trajectory_utils
from simian.public.proto import common_pb2
from simian.public.proto import drawing_pb2
from simian.public.proto import motion_model_pb2
from simian.public.proto import planar_pb2
from simian.public.proto import sim_data_pb2
from simian.public.transforms import proto_util
from tools.py import resources

_SIM_LAUNCH = resources.GetResourceFilename("//customer/apollo/sim/scripts/sim.launch")
# Various modules used by Apollo
_ROUTER = "./bazel-bin/modules/routing/routing"
_PLANNER = "./bazel-bin/modules/planning/planning"
_PREDICTION = "./bazel-bin/modules/prediction/prediction"
_CONTROLLER = "./bazel-bin/modules/control/control"
_HOME_PATH = "/apollo"
_MAX_WAIT_TIME_S = 30.0
_EPS = 1e-6

# A customer's AV stack gets integrated into Simian by subclassing
# from an interface helper that takes care of the general work. This
# subclass gets instantiated inside a grpc server that runs inside the
# customer docker container. The Simian simulation engine, which runs
# in the Applied docker, calls that grpcs server as appropriate during
# a simulation run. The subclass can override a number of methods to
# extend or customize some of the generic behavior, and there are a
# few methods that must be provided. For more details, see
# stack_interface.py and stack_interface_ros.py.
#
# Methods that must be provided by this subclass:
#
# - get_clock_hz(): returns the desired clock update rate.
# - get_simulation_hz(): returns the desired simulation update rate.
# - convert_and_publish_sim_outputs(): receives the simulator state,
#   has to convert and pass it on to the customer stack.
# - convert_and_return_listened_sim_inputs(): collects the data
#   received from the customer stack in the various formats used by
#   Simian, and returns the input for the next simulation tick.
# - initialize(): called right after the Simian core has been
#   initialized, receives the initial simulator state such that the
#   customer stack can be initialized.
#
# Other commonly overriden methods are:
#
# - get_roslaunch_file(): ROS specific, return the path to the
#   roslaunch file that should be used.
# - setup_stack(): hook for performing customer-specific actions
#   during bringup of the AV stack.
# - start_visualization(): hook for launching customer-specific
#   visualization tools in the customer container.
#
# When extending superclass functionality, you usually need to call
# the corresponding superclass method at some point in your
# implementation. In most situations, that will happen as the first
# line in the subclass immplementation.


def _create_routing_request(start_pos, dest_pos):
    """Helper to create routing request in Apollo format"""
    routing_request = routing_pb2.RoutingRequest()
    start_point = routing_request.waypoint.add()
    dest_point = routing_request.waypoint.add()
    start_point.pose.x = start_pos.x
    start_point.pose.y = start_pos.y
    dest_point.pose.x = dest_pos.x
    dest_point.pose.y = dest_pos.y
    routing_request.header.timestamp_sec = 0
    routing_request.header.module_name = "router"
    routing_request.header.sequence_num = 1
    return routing_request


def _set_dbw_command(control_command):
    """Set drive-by-wire command to send to ego.

    This function is used for motion models that use a drive-by-wire command to control ego motion.
    It is useful when the scenario is meant to test controls.
    """
    ego_input = motion_model_pb2.Input()
    ego_input.normalized_dbw.throttle = control_command[0]
    ego_input.normalized_dbw.brake = control_command[1]
    ego_input.normalized_dbw.steering = control_command[2]
    return ego_input


def _set_composite_vehicle_command(control_command):
    """Set composite vehicle command to send to ego.

    This function is used for motion models that use the composite vehicle command to control ego motion.
    It is useful when the scenario is meant to test controls.
    """
    ego_input = motion_model_pb2.Input()
    ego_input.composite_vehicle_command.powertrain_input.normalized_throttle = control_command[0]
    ego_input.composite_vehicle_command.brake_input.normalized_brake = control_command[1]
    ego_input.composite_vehicle_command.steering_input.normalized_angle = control_command[2]
    return ego_input


def _force_ego_state(ego_state):
    """Override the ego state based on the customer model (external in Manual)
    This state is used by the simulator as is without modification to move the ego for
    visualization and all calculations. This will also be reported back to customer in
    next localization message
    """
    ego_input = motion_model_pb2.Input()
    ego_input.override_state.acceleration = ego_state.acceleration
    ego_input.override_state.velocity = ego_state.velocity
    ego_input.override_state.yawrate = ego_state.yawrate
    ego_input.override_state.pose.x = ego_state.pose.x
    ego_input.override_state.pose.y = ego_state.pose.y
    ego_input.override_state.pose.heading = ego_state.pose.heading
    return ego_input


# This is the main "glue" between Simian and the Apollo stack
class ApolloInterface(stack_interface_ros.StackInterfaceRos):
    def __init__(self, proc_handler, startup_options):
        """
        Called via simian.public.customer_stack_server.start_server() from the
        customer_stack_server's main function.

        The proc_handler is a utility for spawning sub-processes such
        that they get cleaned up at the end of the simulation run.

        The startup_options are (basically) a CustomerStartupOptions
        protobuf object, see customer_service.proto for details.
        """
        super().__init__(proc_handler, startup_options)

        self._map_key = startup_options.map_key

        extra_data = startup_options.scenario_extra_data

        self._sim_output = None
        self._use_ego_behavior = extra_data is not None and "ego_behavior_passthrough" in extra_data

        self._sequencer = None
        if extra_data is not None and "motion_command_sequence" in extra_data:
            self._sequencer = motion_command_sequencer.Sequencer()
            self._sequencer.parse_list_of_dicts(extra_data["motion_command_sequence"])

        self._abort_requested = False
        self._router_is_ready = False

        self._routing_request_pub = None
        self._localization_pub = None
        self._chassis_pub = None
        self._perception_obstacles_pub = None
        self._perception_traffic_lights_pub = None
        self._pad_msg_pub = None
        self._message_sequence_number = 0
        self._last_timestamp = None

        # The controller must have advancing time in order to initialize properly. However, the
        # simulation itself doesn't count that as 'sim time' and will send time starting from 0,
        # ignoring the time it takes to initialize. Therefore, use a clock offset to accumulate the
        # initialization time and include it in the time reported to the Apollo stack.
        self._clock_offset = datetime.timedelta()

        # Control handling
        self._control_command = None
        self._published_pad_once = False
        self._controller_is_ready = False
        max_accel, max_decel, max_steer_angle, wheelbase = self._set_control_params(extra_data)
        self._ego_converter = convert_ego.AppliedToApollo(
            max_accel, max_decel, max_steer_angle, wheelbase
        )

        # Trajectory handling
        self._last_convert_plan_time = None
        self._planned_trajectory = None
        self._state2d = None
        max_jump = 0.5
        self._cursor = trajectory_utils.StitchingCursor(max_decel, max_jump)

        # Change path to HOMEPATH since other config files/binaries are relative to this
        if os.getcwd() is not _HOME_PATH:
            rospy.loginfo("Changing directory from %s to %s" % (os.getcwd(), _HOME_PATH))
            os.chdir(_HOME_PATH)

        self._custom_observers = _check_custom_observers(extra_data)

    def _set_control_params(self, extra_data):
        if extra_data is not None and "use_controller" in extra_data:
            self._using_controller = extra_data["use_controller"]
            if "use_composite_vehicle" in extra_data:
                self._using_composite_vehicle = extra_data["use_composite_vehicle"]
            else:
                self._using_composite_vehicle = False
            if "wheelbase" in extra_data:
                wheelbase = extra_data["wheelbase"]
            else:
                print(
                    "WARNING: Using controls, but wheelbase not specified in extra data."
                    " Setting wheelbase to %s." % (2.8448)
                )
                wheelbase = 2.8448
            if "max_acceleration" in extra_data:
                max_accel = extra_data["max_acceleration"]
            else:
                print(
                    "WARNING: Using controls, but max_acceleration not specified in extra data."
                    " Setting max_acceleration to %s." % (2.0)
                )
                max_accel = 2.0
            if "max_deceleration" in extra_data:
                max_decel = extra_data["max_deceleration"]
            else:
                print(
                    "WARNING: Using controls, but max_deceleration not specified in extra data."
                    " Setting max_deceleration to %s." % (6.0)
                )
                max_decel = 6.0
            if "max_steering_angle" in extra_data:
                max_steer_angle = extra_data["max_steering_angle"]
            else:
                print(
                    "WARNING: Using controls, but max_steering_angle not specified in extra"
                    " data. Setting max_steering_angle to %s." % (8.20304748437 / 16.0)
                )
                max_steer_angle = 8.20304748437 / 16.0
        else:
            self._using_controller = False
            self._using_composite_vehicle = False
            max_accel = 2.0
            max_decel = 6.0
            max_steer_angle = 8.20304748437 / 16.0
            wheelbase = 2.8448

        return max_accel, max_decel, max_steer_angle, wheelbase

    def start_visualization(self):
        """
        This gets called during init if requested by startup options,
        i.e. when passing --simulation_flags="--viz" to adp.par.
        """

    def _publish_dt(self, dt):
        self._clock_offset += dt
        self.publish_time(self._last_timestamp)

    def publish_time(self, timestamp):
        self._last_timestamp = timestamp
        super().publish_time(timestamp + self._clock_offset)

    def get_clock_hz(self):
        """Set the Hz with which sim clock is advanced. 100Hz is a reasonable value."""
        return 100

    def get_simulation_hz(self):
        """Set the Hz with which simulation interacts with customer stack."""
        return 10

    def get_roslaunch_file(self):
        return _SIM_LAUNCH

    def rosbag_record_args(self):
        """Pass any additional args required for rosbag"""
        return ["--all", "-x", "/rosout"]

    def setup_stack(self):
        """This routine launches the required processes on customer stack to be ready for simulation
        For Apollo, this includes starting up of required modules, and setting publishers/subscribers
        """
        super().setup_stack()

        ##################################################
        # spawn modules

        # fmt: off
        global_flags = [
            '--log_dir=/apollo/data/log',
            '--vehicle_config_path=modules/common/data/mkz_config.pb.txt',
            '--map_dir=modules/map/data/{}'.format(self._map_key),
            '--use_ros_time=true' ]
        start_route_script = [_ROUTER] + global_flags + [
            '--routing_conf_file=modules/routing/conf/routing_config.pb.txt',
            '--use_road_id=false',
            '--min_length_for_lane_change=5.0',
            '--enable_change_lane_in_result' ]
        start_predictor_script = [_PREDICTION] + global_flags + [
            '--prediction_conf_file=modules/prediction/conf/prediction_conf.pb.txt',
            '--noenable_kf_tracking',
            '--enable_trim_prediction_trajectory',
            '--lane_change_dist=50.0',
            '--junction_search_radius=1.0',
            '--distance_beyond_junction=0.5',
            '--adc_trajectory_search_length=10.0' ]
        start_planning_script = [_PLANNER] + global_flags + [
            # '--nouse_ros_time',
            '--planning_upper_speed_limit=15.65',
            '--noenable_spiral_reference_line',
            '--prioritize_change_lane',
            '--enable_reference_line_stitching',
            '--min_length_for_lane_change=5.0' ]
        start_controller_script = [_CONTROLLER] + global_flags
        # fmt: on

        self._proc_handler.spawn(start_route_script)
        self._proc_handler.spawn(
            start_predictor_script, stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )
        self._proc_handler.spawn(start_planning_script)
        if self._using_controller:
            self._proc_handler.spawn(start_controller_script)

        ##################################################
        # set up publishers

        self._routing_request_pub = rospy.Publisher(
            "/apollo/routing_request", routing_pb2.RoutingRequest, queue_size=10
        )
        self._localization_pub = rospy.Publisher(
            "/apollo/localization/pose", localization_pb2.LocalizationEstimate, queue_size=10
        )
        self._chassis_pub = rospy.Publisher(
            "/apollo/canbus/chassis", chassis_pb2.Chassis, queue_size=10
        )
        self._perception_obstacles_pub = rospy.Publisher(
            "/apollo/perception/obstacles",
            perception_obstacle_pb2.PerceptionObstacles,
            queue_size=10,
        )
        self._perception_traffic_lights_pub = rospy.Publisher(
            "/apollo/perception/traffic_light",
            traffic_light_detection_pb2.TrafficLightDetection,
            queue_size=10,
        )
        if self._using_controller:
            self._pad_msg_pub = rospy.Publisher(
                "/apollo/control/pad", pad_msg_pb2.PadMessage, queue_size=10
            )

        ##################################################
        # set up subscribers

        rospy.Subscriber(
            "/apollo/routing_response", routing_pb2.RoutingResponse, self._routing_response_cb
        )
        rospy.Subscriber("/apollo/planning", planning_pb2.ADCTrajectory, self._planning_response_cb)
        if self._using_controller:
            rospy.Subscriber(
                "/apollo/control", control_cmd_pb2.ControlCommand, self._control_command_cb
            )

    def initialize(self, initial_sim_state, timestamp):
        """
        This method gets called at the very end of the initialization
        sequence. It receives the initial state of the simulation based
        on the active scenario, and a timestamp that indicates the
        current simulated clock. The timestamp is a datetime object that
        contains zero at the start of simulation.

        For Apollo, this involves setting up the routing module by broadcasting ego position
        """
        super().initialize(initial_sim_state, timestamp)
        self._sim_output = initial_sim_state
        self._last_timestamp = timestamp
        start_time = time.time()

        self._state2d = proto_util.state3d_proto_to_state2d(initial_sim_state.ego.sections[0].state)

        # Convert the route points from simulator
        # Note that these are provided for both start and destination
        assert initial_sim_state.tripagent.start is not None
        assert initial_sim_state.tripagent.dest is not None
        routing_request = _create_routing_request(
            initial_sim_state.tripagent.start, initial_sim_state.tripagent.dest
        )

        # Wait for routing and controller. The controller will start correctly only if the time
        # increases during intialization. The time will be reset during the actual sim loop after
        # this initialize function.
        start_try = time.time()
        # sleep_time should be roughly equal to the Apollo control period. It must be less than the
        # Apollo localization period multiplied by the Apollo max_localization_miss parameter.
        sleep_time = 0.01
        while not self._router_is_ready or (
            not self._controller_is_ready and self._using_controller
        ):
            time.sleep(sleep_time)
            ros_time_sec = rospy.Time.now().to_sec()
            self._publish_dt(datetime.timedelta(seconds=sleep_time))
            self._chassis_pub.publish(
                self._update_msg_header(
                    self._ego_converter.create_chassis_message(initial_sim_state), ros_time_sec
                )
            )
            self._localization_pub.publish(
                self._update_msg_header(
                    self._ego_converter.create_localization_message(initial_sim_state),
                    ros_time_sec - _EPS,
                )
            )
            if not self._router_is_ready:
                self._routing_request_pub.publish(routing_request)
            if time.time() - start_try > _MAX_WAIT_TIME_S:
                error_msg = "After %s seconds: router ready: %s" % (
                    _MAX_WAIT_TIME_S,
                    self._router_is_ready,
                )
                if self._using_controller:
                    error_msg += "; controller ready: %s" % (self._controller_is_ready)
                raise RuntimeError(error_msg)

        self._last_convert_plan_time = rospy.Time.now()
        rospy.loginfo("Trip server available now")
        rospy.loginfo(
            "Ego engaged, stack initialized in %4.2f seconds" % (time.time() - start_time)
        )

    # Apollo callbacks
    ################################
    def _routing_response_cb(self, route_response):
        """Callback when the routing service is ready"""
        if route_response.status.error_code == error_code_pb2.OK and not self._router_is_ready:
            rospy.loginfo("Router is ready now.")
            self._router_is_ready = True
        elif not route_response.status.error_code == error_code_pb2.OK:
            rospy.logwarn("Router error: %s" % route_response.status)
            self._router_is_ready = False

    def _planning_response_cb(self, new_plan):
        """Callback when a new planning msg is received"""
        if new_plan.decision is None or (new_plan.HasField("estop") and new_plan.estop.is_estop):
            self._planned_trajectory = None
            return

        main_decision = new_plan.decision.main_decision.WhichOneof("task")
        if main_decision in ["estop", "mission_complete", "not_ready"]:
            self._planned_trajectory = None
            return

        if main_decision in ["stop", "cruise", "parking"]:
            # Because we're in a separate callback thread, first fill
            # a new list instance, then store it in
            # self._planned_trajectory. Otherwise we risk a race
            # condition when iterating over the trajectory in the main
            # thread.
            new_trajectory = list()
            for tp in new_plan.trajectory_point:
                new_trajectory.append(
                    planar_pb2.Trajectory2d.Sample(
                        time=tp.relative_time,
                        state=planar_pb2.State2d(
                            pose=planar_pb2.Pose2d(
                                x=tp.path_point.x,
                                y=tp.path_point.y,
                                heading=tp.path_point.theta,
                            ),
                            acceleration=tp.a,
                            velocity=tp.v,
                            yawrate=tp.path_point.kappa * tp.v,
                        ),
                    )
                )
            self._planned_trajectory = new_trajectory
            return

        rospy.logfatal(
            'Unhandled main_decision "%s" in plan with header %s' % (main_decision, new_plan.header)
        )
        self._abort_requested = True

    def _control_command_cb(self, control_command):
        if (
            control_command.header.status.error_code == error_code_pb2.OK
            and not self._controller_is_ready
        ):
            rospy.loginfo("Controller is ready now.")
            self._controller_is_ready = True
        elif not control_command.header.status.error_code == error_code_pb2.OK:
            rospy.logwarn("Controller error: %s" % (control_command.header.status))
            self._controller_is_ready = False
        self._control_command = control_command

    def _convert_plan_to_motion_command(self):
        """Stitch new plan trajectory to state, and return new state for simulator to advance to."""
        assert self._state2d is not None
        now = rospy.Time.now()
        dt = now - self._last_convert_plan_time
        self._last_convert_plan_time = now
        # Force ego state to be the one computed by trajectory stitching
        return _force_ego_state(
            self._cursor.compute_next_state(self._state2d, self._planned_trajectory, dt.to_sec())
        )

    def _convert_apollo_controls_to_motion_command(self):
        """Convert Apollo control command into format used by Simian motion model.

        Returns:
            motion_model_pb2.Input() with normalized_dbw filled out
        """
        if self._control_command is not None:
            apollo_cmd = (
                self._control_command.throttle,
                self._control_command.brake,
                self._control_command.steering_target,
            )
            # Note: Apollo uses [0, 100] while Simian uses [0, 1]
            cmd = [elm / 100.0 for elm in apollo_cmd]
        else:
            cmd = (0.0, 0.0, 0.0)
        if self._using_composite_vehicle:
            return _set_composite_vehicle_command(cmd)
        return _set_dbw_command(cmd)

    def _update_msg_header(self, msg, ros_time_sec):
        """Update header of message published back to Apollo"""
        self._message_sequence_number += 1
        msg.header.sequence_num = self._message_sequence_number
        msg.header.timestamp_sec = ros_time_sec
        return msg

    def convert_and_return_listened_sim_inputs(self):
        """
        This gets called before each simulator update, to get the
        commands that the customer AV stack needs to send to the
        simulated ego vehicle.

        Returns a sim_data_pb2.SimulatorInput protobuf. The fields that
        need to be filled out here are:
         - bool abort: whether you want to stop this simulation run.
         - EgoInput ego: the command to send to the motion model. The
           type of command needs to be compatible with the motion model.

        The options and formats for EgoInput are explained in more
        detail in the Simian Manual, in section "Ego Vehicle
        Configuration" => "Ego Vehicle Kinematics and Dynamics". You can
        also read the sim_data.proto file for the exact message
        definitions.
        """
        sim_input = sim_data_pb2.SimulatorInput()
        # Note: Pre-canned commands or ego behaviors sent via extra_data override all other command sources
        if self._use_ego_behavior:
            ego_input = motion_model_pb2.Input()
            if (
                self._sim_output
                and self._sim_output.ego
                and self._sim_output.ego.behavior_predicted_control
                and self._sim_output.ego.behavior_predicted_control.WhichOneof("motion_command")
                == "override_state"
            ):
                ego_input.CopyFrom(self._sim_output.ego.behavior_predicted_control)
            elif self._sim_output:
                state2d = proto_util.state3d_proto_to_state2d(
                    self._sim_output.ego.sections[0].state
                )
                ego_input.override_state.pose.x = state2d.pose.x
                ego_input.override_state.pose.y = state2d.pose.y
                ego_input.override_state.pose.heading = state2d.pose.heading
            else:
                ego_input.override_state.pose.x = 0.0
            sim_input.ego.CopyFrom(ego_input)
        elif self._sequencer is not None:
            sim_input.ego.MergeFrom(
                self._sequencer.create_command_absolute_time(self._last_timestamp)
            )
        elif self._using_controller:
            sim_input.ego.CopyFrom(self._convert_apollo_controls_to_motion_command())
        else:
            sim_input.ego.CopyFrom(self._convert_plan_to_motion_command())
        sim_input.abort = self._abort_requested
        if self._planned_trajectory is not None and self._sequencer is None:
            plan_drawing = _draw_motion_plan(self._planned_trajectory)
            sim_input.drawings.extend([plan_drawing])

        custom_observer_events = []
        for observer in self._custom_observers:
            custom_observer_events.append(observer.result())
        sim_input.custom_observer_events.extend(custom_observer_events)

        return sim_input

    def convert_and_publish_sim_outputs(self, sim_state, timestamp):
        """
        This gets called after every update of the simulation. The
        sim_output passed here needs to be converted to messages in the
        customer format, and published in a way that makes them
        available to the running AV stack. See sim_data.proto for the
        detailed definition of the sim_output argument. The timestamp is
        a datetime object encoding the current sim_time.

        Overview of the SimulatorOutput message:
        - sim_output.ego contains the simulated state of the simulated
          ego vehicle. Mainly pose, velocity, acceleration, and
          perceived_actors. See sim_data.proto for more details.
        - sim_output.actors contains ground-truth information about the
          non-ego actors in the scene, most importantly the actor_type,
          id, pose, and velocity. See actor.proto for more details.
        """
        self._sim_output = sim_state

        # Publish time to customer stack
        self.publish_time(timestamp)
        ros_time_sec = rospy.Time.now().to_sec()

        # Send messages to Apollo stack

        if self._using_controller and not self._published_pad_once:
            # The controller will not allow vehicle motion without at least one pad message
            msg = pad_msg_pb2.PadMessage()
            msg.header.timestamp_sec = ros_time_sec
            msg.driving_mode = chassis_pb2.Chassis.COMPLETE_AUTO_DRIVE
            msg.action = pad_msg_pb2.RESET
            self._pad_msg_pub.publish(msg)
            self._published_pad_once = True

        # Localization message, converted from simulator sim_state
        # Note: Apollo uses ROS time, which sometimes suffers from floating point precision errors.
        # If the vehicle state (populated via the localization message) has a time greater than the
        # ROS clock time, the simulation will fail. This happens non-deterministically when the
        # Apollo stack reads the ROS time as e.g. 8.199999999 and the vehicle state is set to e.g.
        # 8.200000000. To avoid this, pre-emptively remove a negligible amount of time from the
        # localization message via _EPS so that the vehicle state will be set to e.g. 8.199999900.
        loc_msg = self._update_msg_header(
            self._ego_converter.create_localization_message(sim_state), ros_time_sec - _EPS
        )
        self._localization_pub.publish(loc_msg)

        # Chassis message, converted from simulator sim_state
        chassis_msg = self._update_msg_header(
            self._ego_converter.create_chassis_message(sim_state), ros_time_sec - _EPS
        )
        self._chassis_pub.publish(chassis_msg)

        # Perception objects perceived by Simian ego, converted into Apollo format
        self._perception_obstacles_pub.publish(
            self._update_msg_header(
                convert_actors.create_perception_obstacles_msg(
                    sim_state.sim_time, sim_state.ego.perceived_actors
                ),
                ros_time_sec - _EPS,
            )
        )

        # Traffic lights detected by an optionally equipped traffic light sensor,
        # converted into Apollo format
        for perception_channel in sim_state.ego.perception_channels:
            if perception_channel.WhichOneof("perception_data") == "traffic_light_sensor":
                self._perception_traffic_lights_pub.publish(
                    self._update_msg_header(
                        convert_traffic_lights.create_traffic_light_detection_msg(
                            sim_state.sim_time,
                            perception_channel.traffic_light_sensor.traffic_lights,
                        ),
                        ros_time_sec - _EPS,
                    )
                )
                break  # Only take one sensor

        # Update local tracked state2d for trajectory stitching
        self._state2d = proto_util.state3d_proto_to_state2d(sim_state.ego.sections[0].state)

        for observer in self._custom_observers:
            observer.process_sim_output(sim_state, self._planned_trajectory)


def _check_custom_observers(extra_data):
    if extra_data is None:
        return []
    custom_observers = []
    if "motion_plan_observer" in extra_data:
        custom_observers.append(MotionPlanObserver(extra_data["motion_plan_observer"]))
    return custom_observers


def _draw_motion_plan(plan):
    """Draw motion trajectory"""
    drawing_proto = drawing_pb2.Drawing()
    drawing_proto.name = "ego_motion_plan"
    drawing_proto.color.red = 229
    drawing_proto.color.green = 55
    drawing_proto.color.blue = 51
    drawing_proto.color.alpha = 1
    drawing_proto.spline.line_width = 1

    for i in range(len(plan)):
        point_proto = drawing_proto.spline.points.add()
        point_proto.x = plan[i].state.pose.x
        point_proto.y = plan[i].state.pose.y
    return drawing_proto


class MotionPlanObserver:
    """
    A custom observer which checks if the path planned by the stack deviates significantly from
    the previous path. Returns False if the endpoint of the trajectory is a given distance from
    the previous endpoint.
    """

    def __init__(self, data):
        self._max_dist = data["threshold_distance"] ** 2
        self._result = True
        self._prev_point = None
        self._last_distance = 0

    def process_sim_output(self, _sim_output, planned_trajectory):
        if planned_trajectory is None:
            self._prev_point = None
            return

        new_point = self._get_last_plan_point(planned_trajectory)
        if self._prev_point is None:
            self._prev_point = new_point
            return

        self._last_distance = self._sq_distance(self._prev_point, new_point)
        if self._last_distance >= self._max_dist:
            self._result = False
        else:
            self._result = True
        self._prev_point = new_point

    @staticmethod
    def _sq_distance(p1, p2):
        return (p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2

    @staticmethod
    def _get_last_plan_point(plan):
        if len(plan) == 0:
            return None
        return (plan[-1].state.pose.x, plan[-1].state.pose.y)

    def result(self):
        return common_pb2.ObserverEvent(
            name="Motion Plan Uncertain",
            value=self._last_distance,
            passed=self._result,
            threshold=self._max_dist,
        )
