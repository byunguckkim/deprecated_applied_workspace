# Copyright (C) 2018 Applied Intuition, Inc. All rights reserved.
# This source code file is distributed under and subject to the LICENSE in license.txt

import collections
import math
import numbers

from simian.public.proto import motion_model_pb2
from simian.public.transforms import proto_util

_Phase = collections.namedtuple("_Phase", "end_time command")


# General support for pre-canned motion commands. Skipping trajectory
# and sequence mode commands for now.
class Sequencer:
    @classmethod
    def FromSpec(cls, motion_command_sequence_proto):
        """Converts a MotionCommandSequence proto to a Sequencer object with specified commands."""
        sequencer = cls()
        for single_command in motion_command_sequence_proto.command_list:
            sequencer._total_duration += single_command.duration
            sequencer._phases.append(_Phase(sequencer._total_duration, single_command.command))
        return sequencer

    def __init__(self):
        self._time = 0.0
        self._total_duration = 0.0
        self._phases = []

    def get_total_duration(self):
        return self._total_duration

    def parse_list_of_dicts(self, specs):
        _dispatch = {
            "velocity_yawrate": self.parse_velocity_yawrate,
            "acceleration_yawrate": self.parse_acceleration_yawrate,
            "normalized_dbw": self.parse_normalized_dbw,
            "velocity_steering": self.parse_velocity_steering,
            "normalized_torque_steering": self.parse_normalized_torque_steering,
            "force_brake_steering": self.parse_force_brake_steering,
            "normalized_dbw_gear": self.parse_normalized_dbw_gear,
            "rover_wheel_command": self.parse_rover_wheel_command,
            "composite_vehicle_command": self.parse_and_append_composite_vehicle_command,
        }
        for spec in specs:
            for key, value in spec.items():
                if key not in _dispatch:
                    raise ValueError('Invalid motion command type "%s".' % key)
                _dispatch[key](value)

    def parse_velocity_yawrate(self, spec):
        for phase in spec:
            self.append_velocity_yawrate(phase["duration"], phase["velocity"], phase["yawrate"])

    def append_velocity_yawrate(self, duration, velocity, yawrate):
        if duration <= 0.0:
            raise ValueError("Invalid duration %f (must be positive)" % duration)
        self._total_duration += duration
        command = motion_model_pb2.Input()
        command.velocity_yawrate.velocity = velocity
        command.velocity_yawrate.yawrate = yawrate
        self._phases.append(_Phase(self._total_duration, command))

    def parse_acceleration_yawrate(self, spec):
        for phase in spec:
            self.append_acceleration_yawrate(
                phase["duration"], phase["acceleration"], phase["yawrate"]
            )

    def append_acceleration_yawrate(self, duration, acceleration, yawrate):
        if duration <= 0.0:
            raise ValueError("Invalid duration %f (must be positive)" % duration)
        self._total_duration += duration
        command = motion_model_pb2.Input()
        command.acceleration_yawrate.acceleration = acceleration
        command.acceleration_yawrate.yawrate = yawrate
        self._phases.append(_Phase(self._total_duration, command))

    def parse_normalized_dbw(self, spec):
        for phase in spec:
            self.append_normalized_dbw(
                phase["duration"], phase["brake"], phase["throttle"], phase["steering"]
            )

    def append_normalized_dbw(self, duration, brake, throttle, steering):
        if duration <= 0.0:
            raise ValueError("Invalid duration %f (must be positive)" % duration)
        if brake < 0.0 or brake > 1.0:
            raise ValueError("Invalid brake %f (must be between 0 and 1)" % brake)
        if throttle < 0.0 or throttle > 1.0:
            raise ValueError("Invalid throttle %f (must be between 0 and 1)" % throttle)
        if abs(steering) > 1.0:
            raise ValueError("Invalid steering %f (must be between -1 and 1)" % steering)
        self._total_duration += duration
        command = motion_model_pb2.Input()
        command.normalized_dbw.brake = brake
        command.normalized_dbw.throttle = throttle
        command.normalized_dbw.steering = steering
        self._phases.append(_Phase(self._total_duration, command))

    def parse_velocity_steering(self, spec):
        for phase in spec:
            self.append_velocity_steering(
                phase["duration"], phase["velocity"], phase["steering_angle"]
            )

    def append_velocity_steering(self, duration, velocity, steering_angle):
        if duration <= 0.0:
            raise ValueError("Invalid duration %f (must be positive)" % duration)
        self._total_duration += duration
        command = motion_model_pb2.Input()
        command.velocity_steering.velocity = velocity
        command.velocity_steering.steering_angle = steering_angle
        self._phases.append(_Phase(self._total_duration, command))

    def parse_normalized_torque_steering(self, spec):
        for phase in spec:
            self.append_normalized_torque_steering(
                phase["duration"], phase["torque"], phase["steering"], phase.get("brake", 0.0)
            )

    def append_normalized_torque_steering(self, duration, torque, steering, brake):
        if duration <= 0.0:
            raise ValueError("Invalid duration %f (must be positive)" % duration)
        if abs(torque) > 1.0:
            raise ValueError("Invalid torque %f (must be between -1 and 1)" % torque)
        if abs(steering) > 1.0:
            raise ValueError("Invalid steering %f (must be between -1 and 1)" % steering)
        self._total_duration += duration
        command = motion_model_pb2.Input()
        command.normalized_torque_steering.torque = torque
        command.normalized_torque_steering.steering = steering
        command.normalized_torque_steering.brake = brake
        self._phases.append(_Phase(self._total_duration, command))

    def parse_force_brake_steering(self, spec):
        for phase in spec:
            self.append_force_brake_steering(
                phase["duration"], phase["motor_force"], phase["brake"], phase["steering"]
            )

    def append_force_brake_steering(self, duration, motor_force, brake, steering):
        if duration <= 0.0:
            raise ValueError("Invalid duration %f (must be positive)" % duration)
        if motor_force < 0.0:
            raise ValueError("Invalid motor_force %f (must be positive)" % motor_force)
        if brake > 1.0 or brake < 0.0:
            raise ValueError("Invalid brake %f (must be between 0 and 1)" % brake)
        if abs(steering) > 1.0:
            raise ValueError("Invalid steering %f (must be between -1 and 1)" % steering)
        self._total_duration += duration
        command = motion_model_pb2.Input()
        command.force_brake_steering.motor_force = motor_force
        command.force_brake_steering.brake = brake
        command.force_brake_steering.steering = steering
        self._phases.append(_Phase(self._total_duration, command))

    def parse_normalized_dbw_gear(self, spec):
        for phase in spec:
            self.append_normalized_dbw_gear(
                phase["duration"],
                phase["brake"],
                phase["throttle"],
                phase["steering"],
                phase["gear"],
            )

    def append_normalized_dbw_gear(self, duration, brake, throttle, steering, gear):
        if duration <= 0.0:
            raise ValueError("Invalid duration %f (must be positive)" % duration)
        if brake < 0.0 or brake > 1.0:
            raise ValueError("Invalid brake %f (must be between 0 and 1)" % brake)
        if throttle < 0.0 or throttle > 1.0:
            raise ValueError("Invalid throttle %f (must be between 0 and 1)" % throttle)
        if abs(steering) > 1.0:
            raise ValueError("Invalid steering %f (must be between -1 and 1)" % steering)
        if gear < 1:
            raise ValueError("Invalid gear %f (must be greater than 0)" % gear)
        self._total_duration += duration
        command = motion_model_pb2.Input()
        command.normalized_dbw_gear.brake = brake
        command.normalized_dbw_gear.throttle = throttle
        command.normalized_dbw_gear.steering = steering
        command.normalized_dbw_gear.gear = int(gear)
        self._phases.append(_Phase(self._total_duration, command))

    def parse_rover_wheel_command(self, spec):
        for phase in spec:
            self.append_rover_wheel_command(
                phase["duration"], phase["fl"], phase["fr"], phase["rl"], phase["rr"]
            )

    def append_rover_wheel_command(self, duration, fl, fr, rl, rr):
        if duration <= 0.0:
            raise ValueError("Invalid duration %f (must be positive)" % duration)
        self._total_duration += duration
        command = motion_model_pb2.Input()

        command.rover_wheel_command.fl.steering = fl["steering"]
        command.rover_wheel_command.fl.ang_vel = fl["ang_vel"]

        command.rover_wheel_command.fr.steering = fr["steering"]
        command.rover_wheel_command.fr.ang_vel = fr["ang_vel"]

        command.rover_wheel_command.rl.steering = rl["steering"]
        command.rover_wheel_command.rl.ang_vel = rl["ang_vel"]

        command.rover_wheel_command.rr.steering = rr["steering"]
        command.rover_wheel_command.rr.ang_vel = rr["ang_vel"]

        self._phases.append(_Phase(self._total_duration, command))

    def parse_and_append_composite_vehicle_command(self, spec):
        for phase in spec:
            command = motion_model_pb2.Input()
            cmd_short = command.composite_vehicle_command

            # Duration
            duration = phase["duration"]
            if duration <= 0.0:
                raise ValueError("Invalid duration %f (must be positive)" % duration)
            self._total_duration += duration

            # Brake input
            if "normalized_brake" in phase:
                cmd_short.brake_input.normalized_brake = phase["normalized_brake"]
            else:
                raise ValueError("Received unexpected or no brake_input.")

            # Powertrain input
            if "normalized_throttle" in phase:
                cmd_short.powertrain_input.normalized_throttle = phase["normalized_throttle"]
            elif "motor_torque" in phase:
                cmd_short.powertrain_input.motor_torque = phase["motor_torque"]
            elif "normalized_motor_torque" in phase:
                cmd_short.powertrain_input.normalized_motor_torque = phase[
                    "normalized_motor_torque"
                ]
            else:
                raise ValueError("Received unexpected or no powertrain_input.")

            # Steering input
            if "angle_rate" in phase:
                cmd_short.steering_input.angle_rate = phase["angle_rate"]
            elif "normalized_angle" in phase:
                cmd_short.steering_input.normalized_angle = phase["normalized_angle"]
            elif "normalized_angle_rate" in phase:
                cmd_short.steering_input.normalized_angle_rate = phase["normalized_angle_rate"]
            else:
                raise ValueError("Received unexpected or no steering_input.")

            self._phases.append(_Phase(self._total_duration, command))

    # Currently a bit of a one-off for when the caller already has a
    # state. As opposed to the others which are geared toward manual
    # text-based specification.
    def append_override_state(self, duration, state2d):
        if duration <= 0.0:
            raise ValueError("Invalid duration %f (must be positive)" % duration)
        self._total_duration += duration
        command = motion_model_pb2.Input()
        command.override_state.MergeFrom(proto_util.state2d_to_proto(state2d))
        self._phases.append(_Phase(self._total_duration, command))

    def is_empty(self):
        return len(self._phases) == 0

    def assert_nonempty(self):
        if not self._phases:
            raise ValueError("Empty motion command sequence.")

    def create_command_relative_time(self, dt):
        return self.create_command_absolute_time(self._time + _to_value(dt))

    # Note this works (fingers crossed) for tt being either a number
    # or a datetime. Under the hood, it is all getting converted to
    # seconds, represented as a floating point for historical
    # reasons. Using a datetime or timedelta could be better here,
    # especially if the create_command_relative_time() method is used,
    # as that can accumulate roundoff errors.
    def create_command_absolute_time(self, tt):
        self.assert_nonempty()
        self._time = math.fmod(_to_value(tt), self._total_duration)
        # Simple linear search suffice for now, we don't expect very
        # long precanned sequences.
        for phase in self._phases:
            if phase.end_time < self._time:
                continue
            return phase.command
        raise Exception("Bug? Fell off the end of the precanned motion command sequence.")


def _to_value(value):
    if isinstance(value, numbers.Number):
        return value
    # For now assume the only alternative is what we use for sim_time
    # method arg plumbing (as opposed to sim_time in sim data proto
    # which is just a float).
    return 3600 * value.hour + 60 * value.minute + value.second + 1e-6 * value.microsecond
