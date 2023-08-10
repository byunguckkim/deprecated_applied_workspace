# Copyright (C) 2019 Applied Intuition, Inc. All rights reserved.
# This source code file is distributed under and subject to the LICENSE in license.txt
import math

import numpy

from simian.public.transforms import planar_py_loader as planar_py
from simian.public.transforms import scalar_py_loader as scalar_py

Screw2d = planar_py.Screw2d
State2d = planar_py.State2d
Vector2d = planar_py.Vector2d
Pose2d = planar_py.Pose2d

# TODO(rolo) Follow the c++ layout, extract out a separate scalar.py.
mod2pi = scalar_py.mod2pi
mod2pi_positive = scalar_py.mod2pi_positive


class Sector:
    def __init__(self, yaw_start, yaw_length, distance_near, distance_far):
        self._distance_interval = scalar_py.Interval(distance_near, distance_far)
        self._angle_interval = scalar_py.AngleInterval(yaw_start, yaw_length)

    def contains_point(self, xx, yy):
        return self.contains_polar(*polar(xx, yy))

    def contains_polar(self, distance, angle):
        return self._distance_interval.contains(distance) and self._angle_interval.contains(angle)


def zero_state():
    return State2d(0.0, 0.0, 0.0, Pose2d.identity())


def mix_state(start_state, end_state, relative_progress):
    weight_start = 1.0 - relative_progress
    px = weight_start * start_state.pose.x + relative_progress * end_state.pose.x
    py = weight_start * start_state.pose.y + relative_progress * end_state.pose.y
    # We could use the sign of the yawrate to break the tie between
    # going CCW or CW, but that would be a can of worms if the signs
    # of the yawrate are not the same at start and end. So, we assume
    # that mix_state gets called only for relatively small heading
    # changes (i.e. <pi) and simply pick the direction that minimizes
    # the magnitude of the angle to traverse.
    ph = mod2pi(
        start_state.pose.heading
        + relative_progress * angle_difference(end_state.pose.heading, start_state.pose.heading)
    )
    yr = weight_start * start_state.yawrate + relative_progress * end_state.yawrate
    acc = weight_start * start_state.acceleration + relative_progress * end_state.acceleration
    vel = weight_start * start_state.velocity + relative_progress * end_state.velocity
    return State2d(acc, vel, yr, Pose2d(px, py, ph))


# Reverse State2d heading, adjust acceleration and velocity appropriately.
def reverse_state(curr_state):
    return State2d(
        -curr_state.acceleration,
        -curr_state.velocity,
        curr_state.yawrate,
        Pose2d(curr_state.pose.x, curr_state.pose.y, mod2pi(curr_state.pose.heading + math.pi)),
    )


def polar(x, y):
    """Returns (distance, angle) with angle in (-pi, pi].

    The angle is zero at the origin.
    """
    return math.hypot(x, y), math.atan2(y, x)


def angle_difference(lhs, rhs):
    """
    Input can be more than pi or less than -pi.
    Returns signed angle difference in [-pi, pi].
    Angle difference is positive in counter clockwise from 'rhs'.
    """
    return mod2pi(lhs - rhs)


def angles_in_same_halfspace(lhs, rhs):
    return abs(angle_difference(lhs, rhs)) < math.pi / 2.0


def project_point_onto_line_segment(line_start_xy, line_end_xy, point_xy):
    """Returns (ratio, closest_point, distance) where 0<=ratio<=1 says where
    on the line the closest_point lies.
    """
    line_start = numpy.array([line_start_xy.x, line_start_xy.y])
    line_end = numpy.array([line_end_xy.x, line_end_xy.y])
    point = numpy.array([point_xy.x, point_xy.y])

    uu = line_end - line_start
    ulen = numpy.linalg.norm(uu)
    if ulen < 1e-6:
        return (1.0, line_end, numpy.linalg.norm(point - line_end))
    uu = uu / ulen
    pp = point - line_start
    ll = numpy.dot(uu, pp)
    if ll < 0.0:
        return (0.0, line_start, numpy.linalg.norm(point - line_start))
    if ll <= ulen:
        projected_point = line_start + ll * uu
        return (ll / ulen, projected_point, numpy.linalg.norm(point - projected_point))
    return (1.0, line_end, numpy.linalg.norm(point - line_end))


def compute_distance(point1, point2):
    """Get distance between two points. Note that this works for poses as well"""
    return ((point1.x - point2.x) ** 2.0 + (point1.y - point2.y) ** 2.0) ** 0.5


def compute_distance_squared(point1, point2):
    """Get squared distance between two points. Note that this works for poses as well"""
    return (point1.x - point2.x) ** 2.0 + (point1.y - point2.y) ** 2.0


def compute_line_angle(from_point, to_point):
    """Get angle of line segment between points. Note that this works for poses as well"""
    return math.atan2(to_point.y - from_point.y, to_point.x - from_point.x)
