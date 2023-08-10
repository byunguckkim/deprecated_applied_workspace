# Copyright (C) 2019 Applied Intuition, Inc. All rights reserved.
# This source code file is distributed under and subject to the LICENSE in license.txt


from simian.public.transforms import spatial_py_loader as spatial_py

MotionState = spatial_py.MotionState  # MotionState is synonymous with State3d
Pose3d = spatial_py.Pose3d
Quaternion = spatial_py.Quaternion
Screw = spatial_py.Screw
Vector3d = spatial_py.Vector3d
SphericalCoordinates = spatial_py.SphericalCoordinates


def quaternion_to_heading(qw, qx, qy, qz):
    qq = spatial_py.Quaternion(qw, qx, qy, qz)
    return qq.to_heading()


def pose_proto_to_heading(pose_proto):
    return quaternion_to_heading(pose_proto.qw, pose_proto.qx, pose_proto.qy, pose_proto.qz)


def compute_distance(point1, point2):
    """Get distance between two points. Note that this works for poses as well"""
    return (
        (point1.x - point2.x) ** 2.0 + (point1.y - point2.y) ** 2.0 + (point1.z - point2.z) ** 2.0
    ) ** 0.5


def compute_distance_using_indices(point1, point2):
    """Get distance between two points. Note that this works for poses as well"""
    return (
        (point1[0] - point2[0]) ** 2.0
        + (point1[1] - point2[1]) ** 2.0
        + (point1[2] - point2[2]) ** 2.0
    ) ** 0.5


def compute_length(pts):
    if len(pts) < 2:
        return 0
    return sum(compute_distance(p1, p2) for p1, p2 in zip(pts, pts[1:]))


def compute_length_using_indicies(pts):
    if len(pts) < 2:
        return 0
    return sum(compute_distance_using_indices(p1, p2) for p1, p2 in zip(pts, pts[1:]))


def avg_points_using_indicies(point1, point2):
    return (
        (point1[0] + point2[0]) / 2.0,
        (point1[1] + point2[1]) / 2.0,
        (point1[2] + point2[2]) / 2.0,
    )


def make_identity_pose():
    return Pose3d.create_with_roll_pitch_yaw(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
