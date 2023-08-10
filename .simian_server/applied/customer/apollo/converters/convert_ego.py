"""
Convert between Apollo and Applied data formats for the ego vehicle.
"""
import math

# Apollo imports
from modules.canbus.proto import chassis_pb2
from modules.common.proto import vehicle_signal_pb2
from modules.localization.proto import localization_pb2

# Simian imports
from simian.public.transforms import spatial


def _copy_t(src, dst):
    """Helper to copy t prefix values"""
    dst.x = src.tx
    dst.y = src.ty
    dst.z = src.tz


def _copy_r(src, dst):
    """Helper to copy r prefix values"""
    dst.x = src.rx
    dst.y = src.ry
    dst.z = src.rz


def _clamp_positive(value):
    """Clamp helper for positive checks"""
    return min(1.0, max(0.0, value))


def _clamp_centered(value):
    """Clamp helper for centered values between -1.0 and 1.0"""
    return min(1.0, max(-1.0, value))


def _convert_axes(msg):
    """Rotates Simian to Apollo convention (-90 deg about Z axis)"""

    # Convert orientation
    orientation = msg.pose.orientation
    rotation_quat = spatial.Quaternion().from_roll_pitch_yaw(0, 0, -math.pi / 2)
    original_quat = spatial.Quaternion(
        orientation.qw, orientation.qx, orientation.qy, orientation.qz
    )
    new_orientation_quat = rotation_quat * original_quat
    orientation.qw = new_orientation_quat.w
    orientation.qx = new_orientation_quat.x
    orientation.qy = new_orientation_quat.y
    orientation.qz = new_orientation_quat.z

    # Convert velocities and accelerations
    linear_velocity = msg.pose.linear_velocity
    linear_velocity.x, linear_velocity.y = -linear_velocity.y, linear_velocity.x
    angular_velocity = msg.pose.angular_velocity
    angular_velocity.x, angular_velocity.y = -angular_velocity.y, angular_velocity.x
    linear_acceleration = msg.pose.linear_acceleration
    linear_acceleration.x, linear_acceleration.y = -linear_acceleration.y, linear_acceleration.x

    return msg


class AppliedToApollo:
    def __init__(
        self,
        max_acceleration=2.0,
        max_deceleration=6.0,
        max_steering_angle=0.51269,
        wheelbase=2.8448,
    ):
        self._max_acceleration = max_acceleration
        self._max_deceleration = max_deceleration
        self._max_steering_angle = max_steering_angle
        self._wheelbase = wheelbase

    def create_localization_message(self, sim_state):
        """Creates LocalizationEstimate message in Apollo format from simulator state.

        For the header, it only fills in timestamp_sec. In particular,
        sequence number and module name are left empty.
        """
        msg = localization_pb2.LocalizationEstimate()
        msg.header.timestamp_sec = sim_state.sim_time
        state = sim_state.ego.sections[0].state
        msg.pose.position.x = state.pose.px
        msg.pose.position.y = state.pose.py
        msg.pose.position.z = state.pose.pz
        msg.pose.orientation.qx = state.pose.qx
        msg.pose.orientation.qy = state.pose.qy
        msg.pose.orientation.qz = state.pose.qz
        msg.pose.orientation.qw = state.pose.qw
        _copy_t(state.velocity, msg.pose.linear_velocity)
        _copy_r(state.velocity, msg.pose.angular_velocity)
        _copy_r(state.velocity, msg.pose.angular_velocity_vrf)
        _copy_t(state.acceleration, msg.pose.linear_acceleration)
        _copy_t(state.acceleration, msg.pose.linear_acceleration_vrf)
        msg.pose.heading = math.copysign(2.0 * math.acos(state.pose.qw), state.pose.qz)

        # Apollo's axis convention is Y forward and X right. Simian convention is X forward with Y
        # left. So we need to rotate by 90 degrees counter-clockwise around Z.
        _convert_axes(msg)

        return msg

    def create_chassis_message(self, sim_state):
        """Create chassis message for ego in apollo format"""
        msg = chassis_pb2.Chassis()
        msg.header.timestamp_sec = sim_state.sim_time
        msg.error_code = chassis_pb2.Chassis.NO_ERROR
        msg.driving_mode = chassis_pb2.Chassis.COMPLETE_AUTO_DRIVE
        msg.gear_location = chassis_pb2.Chassis.GEAR_DRIVE
        state = sim_state.ego.sections[0].state
        msg.speed_mps = state.velocity.tx
        msg.throttle_percentage = 100.0 * _clamp_positive(
            state.acceleration.tx / self._max_acceleration
        )
        msg.brake_percentage = 100.0 * _clamp_positive(
            -state.acceleration.tx / self._max_deceleration
        )
        curvature = 0.0
        if abs(state.velocity.tx) > 1e-9:
            curvature = state.velocity.rz / state.velocity.tx
        steering_angle = math.atan(curvature * self._wheelbase)
        msg.steering_percentage = 100.0 * _clamp_centered(steering_angle / self._max_steering_angle)

        for message in sim_state.ego.messages:
            if message.name == "turn_signal":
                if message.value in ["TURN_NONE", "TURN_LEFT", "TURN_RIGHT"]:
                    msg.signal.turn_signal = vehicle_signal_pb2.VehicleSignal.TurnSignal.Value(
                        message.value
                    )

        return msg
