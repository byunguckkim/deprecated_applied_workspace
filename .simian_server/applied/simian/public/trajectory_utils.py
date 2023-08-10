# Copyright (C) 2018 Applied Intuition, Inc. All rights reserved.
# This source code file is distributed under and subject to the LICENSE in license.txt
import math
import sys

import numpy

from simian.public.transforms import planar

Pose2d = planar.Pose2d
State2d = planar.State2d


def brake_hard(state2d, max_deceleration, tick_duration):
    assert tick_duration >= 0.0
    if tick_duration == 0.0:
        return state2d
    acceleration = max(-max_deceleration, min(max_deceleration, -state2d.velocity / tick_duration))
    velocity = state2d.velocity + acceleration * tick_duration
    curvature = 0.0
    if abs(state2d.velocity) > 1e-3:
        curvature = state2d.yawrate / state2d.velocity
    yawrate = velocity * curvature
    distance = velocity * tick_duration
    heading = state2d.pose.heading + yawrate * tick_duration
    pose = Pose2d(
        state2d.pose.x + distance * math.cos(heading),
        state2d.pose.y + distance * math.sin(heading),
        heading,
    )
    return State2d(acceleration, velocity, yawrate, pose)


def match_point_to_trajectory(point, trajectory):
    """Returns (continuous_index, point-on-trajectory, distance).
    Assumes point is numpy.array.
    """
    if len(trajectory) == 0:
        return (None, None, None)
    # Find closest trj point, then see if interpolating between that
    # and its predecessor/successor gives a better match. Prefer
    # successor if it is interpolated or maxed out. Otherwise
    # predecessor, and fall back on snap point.
    snap_index = None
    snap_d2 = None
    for ii in reversed(range(len(trajectory))):
        pt = trajectory[ii].state.pose
        d2 = (point.x - pt.x) ** 2 + (point.y - pt.y) ** 2
        if snap_d2 is None or d2 < snap_d2:
            snap_index = ii
            snap_d2 = d2
    pred = (None, None, sys.float_info.max)
    if snap_index > 0:  # type: ignore[operator] # ignore baseline; see #61427
        pred = planar.project_point_onto_line_segment(
            trajectory[snap_index - 1].state.pose, trajectory[snap_index].state.pose, point  # type: ignore[operator] # ignore baseline; see #61427
        )
    succ = (None, None, sys.float_info.max)
    if snap_index < len(trajectory) - 1:  # type: ignore[operator] # ignore baseline; see #61427
        succ = planar.project_point_onto_line_segment(
            trajectory[snap_index].state.pose, trajectory[snap_index + 1].state.pose, point  # type: ignore[operator] # ignore baseline; see #61427
        )
    if succ[2] < pred[2]:
        if succ[0] is None:
            pot = numpy.array(
                [trajectory[snap_index].state.pose.x, trajectory[snap_index].state.pose.y]
            )
            return (snap_index, pot, numpy.linalg.norm(pot - numpy.array([point.x, point.y])))
        return (snap_index + succ[0], succ[1], succ[2])
    if pred[0] is None:
        pot = numpy.array(
            [trajectory[snap_index].state.pose.x, trajectory[snap_index].state.pose.y]
        )
        return (snap_index, pot, numpy.linalg.norm(pot - numpy.array([point.x, point.y])))
    return (snap_index + pred[0] - 1, pred[1], pred[2])


def _pred_of_continuous_index(continuous_index):
    pred = int(continuous_index)
    return pred


def interpolate_trajectory_index(continuous_index, trajectory):
    assert len(trajectory) > 0
    pred = _pred_of_continuous_index(continuous_index)
    if pred < 0:
        return trajectory[0].time
    if pred >= len(trajectory) - 1:
        return trajectory[-1].time
    succ = pred + 1
    d_pred = max(0.0, continuous_index - pred)
    d_succ = 1.0 - d_pred
    return d_succ * trajectory[pred].time + d_pred * trajectory[succ].time


def _trajectory_sample_to_state(sample):
    state = sample.state
    pose = state.pose
    return State2d(
        state.acceleration, state.velocity, state.yawrate, Pose2d(pose.x, pose.y, pose.heading)
    )


def interpolate_trajectory_time(desired_time, trajectory):
    assert len(trajectory) > 0
    if trajectory[0].time > desired_time:
        return None
    if trajectory[-1].time <= desired_time:
        return None
    # Binary search first time that is beyond desired_time
    left = 0
    right = len(trajectory) - 1
    assert left <= right, "Empty trajectory"
    while left != right:
        middle = (left + right) // 2
        if trajectory[middle].time <= desired_time:
            left = middle + 1
        else:
            right = middle
    # Here, left==right are the first index with time>desired_time
    if left == 0:
        return None  # Should never happen
    left -= 1  # left is the predecessor, right is the successor
    assert trajectory[left].time <= desired_time
    assert trajectory[right].time > desired_time
    dt_left = desired_time - trajectory[left].time
    dt_segment = trajectory[right].time - trajectory[left].time
    return planar.mix_state(
        _trajectory_sample_to_state(trajectory[left]),
        _trajectory_sample_to_state(trajectory[right]),
        dt_left / dt_segment,
    )


def stitch_trajectory(old_trajectory, new_trajectory):
    assert new_trajectory is not None
    assert len(new_trajectory) > 0
    if old_trajectory is None:
        return None

    # Find out where to cut the old trajectory and fix timing of new
    # trajectory to match that.
    handover_index, _handover_point, _handover_distance = match_point_to_trajectory(
        new_trajectory[0].state.pose, old_trajectory
    )
    assert handover_index is not None
    handover_time = interpolate_trajectory_index(handover_index, new_trajectory)
    time_correction = handover_time - new_trajectory[0].time
    for trj in new_trajectory:
        trj.time += time_correction

    # Chomp off old part of sane_trajectory, append time-corrected new
    # trajectory.
    chomp_index = max(0, _pred_of_continuous_index(handover_index))
    return old_trajectory[0:chomp_index] + new_trajectory


def stitch_or_switch(pose, old_trajectory, new_trajectory, max_jump):
    """Returns (trajectory, match_time) on success.

    The returned trajectory is either the new one, or the new stitched
    onto the old. This is controlled by the max_jump parameter: if the
    state is closer than max_jump to the new trajectory, we use
    that. Otherwise, we try to stitch, but if the old trajectory is
    None or new trajectory is also more than max_jump away from the
    old, we give up and return (None, None). Likewise, if the new
    trajectory is None or empty, we return (None, None).

    The returned match_time is the time of the trajectory
    point closest to the given pose.

    Asserts that the timestamps of the new trajectory are strictly
    increasing.
    """
    if new_trajectory is None or len(new_trajectory) == 0:
        return (None, None)
    for ii in range(1, len(new_trajectory)):
        if new_trajectory[ii].time <= new_trajectory[ii - 1].time:
            dbgmsg = "  num_samples %d\n  samples around index %d:" % (len(new_trajectory), ii)
            for jj in range(max(0, ii - 2), min(len(new_trajectory), ii + 3)):
                sample = new_trajectory[jj]
                dbgmsg = "%s\n    i: %d  t: %f  x: %f  y: %f  heading: %f" % (
                    dbgmsg,
                    jj,
                    sample.time,
                    sample.state.pose.x,
                    sample.state.pose.y,
                    sample.state.pose.heading,
                )
            raise ValueError(
                "Invalid input trajectory: time[%d]=%f should be < time[%d]=%f.\n%s"
                % (ii - 1, new_trajectory[ii - 1].time, ii, new_trajectory[ii].time, dbgmsg)
            )

    match_index, _match_point, match_distance = match_point_to_trajectory(pose, new_trajectory)
    assert match_index is not None

    if match_distance <= max_jump:
        return (new_trajectory, interpolate_trajectory_index(match_index, new_trajectory))

    sane_trajectory = stitch_trajectory(old_trajectory, new_trajectory)
    if sane_trajectory is None:
        return (None, None)
    match_index, _match_point, match_distance = match_point_to_trajectory(pose, sane_trajectory)
    if match_distance > max_jump:
        return (None, None)
    return (sane_trajectory, interpolate_trajectory_index(match_index, sane_trajectory))


class StitchingCursor:
    def __init__(self, max_deceleration, max_jump):
        assert max_deceleration > 0
        assert max_jump > 0
        self.max_deceleration = max_deceleration
        self.max_jump = max_jump
        self.trajectory = None

    def compute_next_state(self, state2d, incoming_trajectory, lookahead_duration):
        self.trajectory, match_time = stitch_or_switch(
            state2d.pose, self.trajectory, incoming_trajectory, self.max_jump
        )
        if self.trajectory is None:
            return brake_hard(state2d, self.max_deceleration, lookahead_duration)

        wanted_time = match_time + lookahead_duration
        interpolated_state = interpolate_trajectory_time(wanted_time, self.trajectory)
        if interpolated_state is None:
            self.trajectory = None
            return brake_hard(state2d, self.max_deceleration, lookahead_duration)
        return interpolated_state
