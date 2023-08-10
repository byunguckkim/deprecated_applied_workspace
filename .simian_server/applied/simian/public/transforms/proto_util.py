"""
PY2CPP Warning:
This module has been frozen as part of the migration to C++.
Please make changes to the C++ implementation instead.
"""

# Copyright (C) 2019 Applied Intuition, Inc. All rights reserved.
# This source code file is distributed under and subject to the LICENSE in license.txt
import math

from simian.public.osi3 import osi_common_pb2
from simian.public.proto import planar_pb2
from simian.public.proto import spatial_pb2
from simian.public.transforms import planar
from simian.public.transforms import proto_conversion_py_loader as proto_conversion_py
from simian.public.transforms import spatial
from simian.public.transforms import spatial_py_loader as spatial_py

# The below pybind does not work consistently. In some environments we get a bad alloc error.
# See https://github.com/AppliedIntuition/applied2/issues/110175
# proto_to_pose3d = proto_conversion_py.pose_proto_to_pose3d_return_nocheck


def _hasnan(quaternion: spatial.Quaternion) -> bool:
    """Checks if any quaternion component is nan"""
    return (
        math.isnan(quaternion.qw)
        or math.isnan(quaternion.qx)
        or math.isnan(quaternion.qy)
        or math.isnan(quaternion.qz)
    )


def check_pose_3d_has_nan(pose3d: spatial.Pose3d) -> bool:
    return (
        math.isnan(pose3d.qw)
        or math.isnan(pose3d.qx)
        or math.isnan(pose3d.qy)
        or math.isnan(pose3d.qz)
    )


def proto_to_vector2d(point2d_proto: planar_pb2.Point2d) -> planar.Vector2d:
    return planar.Vector2d(point2d_proto.x, point2d_proto.y)


def vector2d_to_proto(point2d: planar.Vector2d) -> planar_pb2.Point2d:
    return planar_pb2.Point2d(x=point2d.x, y=point2d.y)


def proto_to_pose2d(pose2d_proto: planar_pb2.Pose2d) -> planar.Pose2d:
    return planar.Pose2d(pose2d_proto.x, pose2d_proto.y, pose2d_proto.heading)


def pose2d_to_proto(pose2d: planar.Pose2d) -> planar_pb2.Pose2d:
    return planar_pb2.Pose2d(x=pose2d.x, y=pose2d.y, heading=pose2d.heading)


def proto_to_screw2d(screw2d_proto: planar_pb2.Screw2d) -> planar.Screw2d:
    return planar.Screw2d(screw2d_proto.tx, screw2d_proto.ty, screw2d_proto.rz)


def screw2d_to_proto(screw2d: planar.Screw2d) -> planar_pb2.Screw2d:
    return planar_pb2.Screw2d(tx=screw2d.tx, ty=screw2d.ty, rz=screw2d.rz)


#  planar_pb2.State2d holds velocity, yawrate and velocity_screw as well as acceleration and acceleration_screw.
#  Defining the screw takes priority over the other fields, i.e:
# - if velocity_screw is set, we ignore velocity and yawrate.
# - if acceleration_screw is set, we ignore acceleration.
def proto_to_state2d(state2d_proto: planar_pb2.State2d) -> planar.State2d:
    if state2d_proto.HasField("velocity_screw"):
        velocity = proto_to_screw2d(state2d_proto.velocity_screw)
    else:
        velocity = planar.Screw2d(state2d_proto.velocity, 0, state2d_proto.yawrate)

    if state2d_proto.HasField("acceleration_screw"):
        acceleration = proto_to_screw2d(state2d_proto.acceleration_screw)
    else:
        acceleration = planar.Screw2d(state2d_proto.acceleration, 0, 0)

    return planar.State2d(
        proto_to_pose2d(state2d_proto.pose),
        velocity,
        acceleration,
    )


# planar_pb2.State2d holds velocity, yawrate and velocity_screw as well as acceleration and acceleration_screw.
# We populate all fields in the output proto to ensure backwards compatibility.
def state2d_to_proto(state2d: planar.State2d) -> planar_pb2.State2d:
    return planar_pb2.State2d(
        acceleration=state2d.acceleration,
        velocity=state2d.velocity,
        yawrate=state2d.yawrate,
        pose=pose2d_to_proto(state2d.pose),
        velocity_screw=screw2d_to_proto(state2d.velocity_screw),
        acceleration_screw=screw2d_to_proto(state2d.acceleration_screw),
    )


def state2d_full_proto_to_motion_state(override_state2d_full_proto):
    # Motion state is "fuller" than state2d_full as it takes into account the z component
    # Calling this method fills up the missing information with 0,
    # specifically z, roll & pitch, and their first and second order derivatives.
    return spatial.MotionState(
        spatial.Pose3d().create_with_roll_pitch_yaw(
            override_state2d_full_proto.pose.x,
            override_state2d_full_proto.pose.y,
            0.0,  # z
            0.0,  # roll
            0.0,  # pitch
            override_state2d_full_proto.pose.heading,
        ),
        spatial.Screw(
            spatial.Vector3d(
                override_state2d_full_proto.velocity.tx,
                override_state2d_full_proto.velocity.ty,
                0.0,  # z
            ),
            spatial.Vector3d(
                0.0, 0.0, override_state2d_full_proto.velocity.rz
            ),  # roll, pitch set to 0.0
        ),
        spatial.Screw(
            spatial.Vector3d(
                override_state2d_full_proto.acceleration.tx,
                override_state2d_full_proto.acceleration.ty,
                0.0,  # z
            ),
            spatial.Vector3d(
                0.0, 0.0, override_state2d_full_proto.acceleration.rz  # roll, pitch set to 0.0
            ),
        ),
    )


def proto_to_vector3d(point3d_proto: spatial_pb2.Point) -> spatial.Vector3d:
    return spatial.Vector3d(point3d_proto.x, point3d_proto.y, point3d_proto.z)


def vector3d_to_proto(point3d: spatial.Vector3d) -> spatial_pb2.Point:
    return spatial_pb2.Point(x=point3d.x, y=point3d.y, z=point3d.z)


def proto_to_pose3d(pose_proto: spatial_pb2.Pose) -> spatial.Pose3d:
    return spatial.Pose3d(
        spatial.Vector3d(pose_proto.px, pose_proto.py, pose_proto.pz),
        spatial.Quaternion(pose_proto.qw, pose_proto.qx, pose_proto.qy, pose_proto.qz),
    )


def pose_spec_proto_to_pose3d(pose_spec_proto: spatial_pb2.PoseSpec) -> spatial.Pose3d:
    typestring = pose_spec_proto.WhichOneof("orientation")
    if typestring == "quaternion":
        if _hasnan(pose_spec_proto.quaternion):
            raise ValueError("All components of quaternion must be specified")
    pose3d = spatial_py.Pose3d()
    proto_conversion_py.pose_spec_proto_to_pose3d(pose_spec_proto, pose3d)
    return pose3d


def osi_base_stationary_proto_to_pose3d(
    osi_base_proto: osi_common_pb2.BaseStationary,
) -> spatial.Pose3d:
    return spatial.Pose3d(
        spatial.Vector3d(
            osi_base_proto.position.x, osi_base_proto.position.y, osi_base_proto.position.z
        ),
        spatial.Quaternion.from_roll_pitch_yaw(
            osi_base_proto.orientation.roll,
            osi_base_proto.orientation.pitch,
            osi_base_proto.orientation.yaw,
        ),
    )


def pose3d_to_proto(pose: spatial.Pose3d) -> spatial_pb2.Pose:
    return spatial_pb2.Pose(
        px=pose.px,
        py=pose.py,
        pz=pose.pz,
        qw=pose.qw,
        qx=pose.qx,
        qy=pose.qy,
        qz=pose.qz,
    )


def screw_to_proto(
    translational: spatial.Vector3d, rotational: spatial.Vector3d
) -> spatial_pb2.Screw:
    return spatial_pb2.Screw(
        tx=translational.x,
        ty=translational.y,
        tz=translational.z,
        rx=rotational.x,
        ry=rotational.y,
        rz=rotational.z,
    )


def proto_to_screw(screw_proto: spatial_pb2.Screw) -> spatial.Screw:
    return spatial.Screw(
        screw_proto_translation_to_vector3d(screw_proto),
        screw_proto_rotation_to_vector3d(screw_proto),
    )


# State2dFull and State2d hold the same information. For backwards compatibility reasons, we still need State2dFull.
# This method should be used when dealing with State2dFull to convert it to the preferred State2d.
def state2d_full_proto_to_state2d_proto(
    state2d_full_proto: planar_pb2.State2dFull,
) -> planar_pb2.State2d:
    """
    Converts a State2dFull proto to a State2d proto without loss of information.

    Args:
        state2d_full (planar_pb2.State2dFull): input State2dFull proto

    Returns:
        (planar_pb2.State2d): State2d proto with all fields specified.
    """
    return planar_pb2.State2d(
        acceleration=state2d_full_proto.acceleration.tx,
        velocity=state2d_full_proto.velocity.tx,
        yawrate=state2d_full_proto.velocity.rz,
        pose=state2d_full_proto.pose,
        velocity_screw=state2d_full_proto.velocity,
        acceleration_screw=state2d_full_proto.acceleration,
    )


# This converter works for simian_public.spatial.State and for
# simian_public.perception.PerceptionChannel.LocalizationSensor
# messages. And possibly others, because Python just looks up
# attributes by name.
def proto_to_motion_state(proto):
    return spatial.MotionState(
        proto_to_pose3d(proto.pose),
        proto_to_screw(proto.velocity),
        proto_to_screw(proto.acceleration),
    )


def motion_state_to_proto(motion_state):
    return spatial_pb2.State(
        pose=pose3d_to_proto(motion_state.pose),
        velocity=screw_to_proto(motion_state.velocity.translation, motion_state.velocity.rotation),
        acceleration=screw_to_proto(
            motion_state.acceleration.translation, motion_state.acceleration.rotation
        ),
    )


def screw_proto_translation_to_vector3d(screw_proto: spatial_pb2.Screw) -> spatial.Vector3d:
    return spatial.Vector3d(screw_proto.tx, screw_proto.ty, screw_proto.tz)


def screw_proto_rotation_to_vector3d(screw_proto: spatial_pb2.Screw) -> spatial.Vector3d:
    return spatial.Vector3d(screw_proto.rx, screw_proto.ry, screw_proto.rz)


def create_pose3d_proto_xyplane(ox, oy, heading):
    """Create a spatial.Pose3d proto given a planar pose's origin and heading."""
    pose = spatial_pb2.Pose()
    pose.px = ox
    pose.py = oy
    pose.pz = 0.0
    pose.qx = 0.0
    pose.qy = 0.0
    pose.qz = math.sin(heading / 2.0)
    pose.qw = math.cos(heading / 2.0)
    return pose


def create_pose3d_proto_identity():
    return create_pose3d_proto_xyplane(0.0, 0.0, 0.0)


def create_screw_proto_xyplane(speed, heading):
    velocity = spatial_pb2.Screw()
    velocity.tx = speed * math.cos(heading)
    velocity.ty = speed * math.sin(heading)
    return velocity


# See proto_util_py.cc for the C++ actor convention implementation.
# See legacy/proto_util_legacy.py for older versions.
def create_state3d_proto_tangent_ego_convention(
    pose3d: spatial.Pose3d, state2d: planar.State2d
) -> spatial_pb2.State:
    """The returned state3d contains velocities and accelerations observed
    in the world frame and expressed in the frame described by the
    provided pose3d (most often the first section of ego; that is,
    expressed in the vehicle frame).

    The returned state3d does not include a non-zero lateral velocity component and rotational acceleration.
    If those values are needed, use create_state3d_proto_from_state2d instead.
    """
    vt = spatial.Vector3d(state2d.velocity, 0.0, 0.0)
    vr = spatial.Vector3d(0.0, 0.0, state2d.yawrate)
    cp = state2d.yawrate * state2d.velocity  # centripetal acceleration
    at = spatial.Vector3d(state2d.acceleration, cp, 0.0)
    return spatial_pb2.State(
        pose=pose3d_to_proto(pose3d),
        velocity=screw_to_proto(vt, vr),
        acceleration=screw_to_proto(at, _ZERO_VECTOR_3D),
    )


def create_state3d_proto_from_pose3d_and_state2d(
    pose3d: spatial.Pose3d, state2d: planar.State2d
) -> spatial_pb2.State:
    """The returned state3d contains pose, velocities, and accelerations observed
    in the world frame and expressed in the frame described by the
    provided pose3d and state2d.
    Pose3d and State2d are assumed to be consistent in their frames.
    """
    vt = spatial.Vector3d(state2d.velocity_screw.tx, state2d.velocity_screw.ty, 0.0)
    vr = spatial.Vector3d(0.0, 0.0, state2d.velocity_screw.rz)
    at = spatial.Vector3d(state2d.acceleration_screw.tx, state2d.acceleration_screw.ty, 0.0)
    ar = spatial.Vector3d(0.0, 0.0, state2d.acceleration_screw.rz)
    return spatial_pb2.State(
        pose=pose3d_to_proto(pose3d),
        velocity=screw_to_proto(vt, vr),
        acceleration=screw_to_proto(at, ar),
    )


def pose3d_to_pose_spec_proto(pose3d: spatial.Pose3d) -> spatial_pb2.PoseSpec:
    pose_spec_proto = spatial_pb2.PoseSpec(px=pose3d.px, py=pose3d.py, pz=pose3d.pz)
    quat = spatial.Quaternion(
        pose3d.orientation.w, pose3d.orientation.x, pose3d.orientation.y, pose3d.orientation.z
    )
    rpy = quat.roll_pitch_yaw()
    pose_spec_proto.rpy.roll = rpy.x
    pose_spec_proto.rpy.pitch = rpy.y
    pose_spec_proto.rpy.yaw = rpy.z
    return pose_spec_proto


def state2d_to_state3d_proto(
    state2d: planar.State2d,
) -> spatial_pb2.State:
    state3d = spatial_pb2.State()
    state3d.pose.px = state2d.pose.x
    state3d.pose.py = state2d.pose.y
    state3d.pose.pz = 0.0
    state3d.pose.qx = 0.0
    state3d.pose.qy = 0.0
    state3d.pose.qz = math.sin(state2d.pose.heading / 2)
    state3d.pose.qw = math.cos(state2d.pose.heading / 2)
    state3d.velocity.tx = state2d.velocity_screw.tx
    state3d.velocity.ty = state2d.velocity_screw.ty
    state3d.velocity.tz = 0.0
    state3d.velocity.rx = 0.0
    state3d.velocity.ry = 0.0
    state3d.velocity.rz = state2d.velocity_screw.rz
    state3d.acceleration.tx = state2d.acceleration_screw.tx
    state3d.acceleration.ty = state2d.acceleration_screw.ty
    state3d.acceleration.tz = 0.0
    state3d.acceleration.rx = 0.0
    state3d.acceleration.ry = 0.0
    state3d.acceleration.rz = state2d.acceleration_screw.rz
    return state3d


def state3d_proto_to_state2d(state3d: spatial_pb2.State) -> planar.State2d:
    return planar.State2d(
        planar.Pose2d(
            state3d.pose.px, state3d.pose.py, spatial.pose_proto_to_heading(state3d.pose)
        ),
        planar.Screw2d(
            state3d.velocity.tx,
            state3d.velocity.ty,
            state3d.velocity.rz,
        ),
        planar.Screw2d(
            state3d.acceleration.tx,
            state3d.acceleration.ty,
            state3d.acceleration.rz,
        ),
    )


def pose3d_proto_to_pose2d_proto(pose3d_proto: spatial_pb2.Pose) -> planar_pb2.Pose2d:
    """ "
    Convenience function for dealing with the fact that Simian sends
    3D ego and actor poses, but expects 2D poses for set_ego_pose and
    set_actor_pose commands.

    Example usage:

    # Somewhere in your stack shim:
    def convert_and_return_listened_sim_inputs(self):
        sim_input = sim_data_pb2.SimulatorInput()
        sim_command = sim_input.sim_commands.add()
        sim_command.set_ego_pose.pose.MergeFrom(
            proto_util.pose3d_proto_to_pose2d_proto(self._prev_pose3d))
        sim_command.set_ego_pose.pose.x = something_else()
        fill_in_rest_of(sim_input)
        return sim_input
    """
    qw = pose3d_proto.qw
    qz = pose3d_proto.qz
    return planar_pb2.Pose2d(
        x=pose3d_proto.px, y=pose3d_proto.py, heading=math.copysign(2.0 * math.acos(qw), qz)
    )


_ZERO_VECTOR_3D = spatial.Vector3d(0.0, 0.0, 0.0)
