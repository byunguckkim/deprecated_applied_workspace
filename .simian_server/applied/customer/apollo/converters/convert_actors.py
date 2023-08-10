"""
Convert between Apollo and Applied obstacle message formats.
"""
# Apollo include
from modules.perception.proto import perception_obstacle_pb2

from simian.public.proto import actor_pb2

# Simian actor msg
from simian.public.transforms import spatial

_OBSTACLE_TYPE_TO_LEGACY = {
    actor_pb2.Actor.UNKNOWN: perception_obstacle_pb2.PerceptionObstacle.UNKNOWN,
    actor_pb2.Actor.UNKNOWN_MOVABLE: perception_obstacle_pb2.PerceptionObstacle.UNKNOWN_MOVABLE,
    actor_pb2.Actor.UNKNOWN_UNMOVABLE: perception_obstacle_pb2.PerceptionObstacle.UNKNOWN_UNMOVABLE,
    actor_pb2.Actor.PEDESTRIAN: perception_obstacle_pb2.PerceptionObstacle.PEDESTRIAN,
    actor_pb2.Actor.VEHICLE: perception_obstacle_pb2.PerceptionObstacle.VEHICLE,
    actor_pb2.Actor.BICYCLE: perception_obstacle_pb2.PerceptionObstacle.BICYCLE,
    actor_pb2.Actor.TRUCK: perception_obstacle_pb2.PerceptionObstacle.UNKNOWN_MOVABLE,
    actor_pb2.Actor.MOTORCYCLE: perception_obstacle_pb2.PerceptionObstacle.UNKNOWN_MOVABLE,
    actor_pb2.Actor.GOLF_CART: perception_obstacle_pb2.PerceptionObstacle.VEHICLE,
    actor_pb2.Actor.ANIMAL: perception_obstacle_pb2.PerceptionObstacle.UNKNOWN_MOVABLE,
    actor_pb2.Actor.VEHICLE_ON_SHOULDER: perception_obstacle_pb2.PerceptionObstacle.VEHICLE,
}


def _copy_t(src, dst):
    """Helper to copy t prefix values"""
    dst.x = src.tx
    dst.y = src.ty
    dst.z = src.tz


def _set_perception_obstacle_info(actor, out_msg):
    """Set the fields on a perception_obstacle proto"""
    out_msg.id = actor.id
    out_msg.type = _OBSTACLE_TYPE_TO_LEGACY.get(
        actor.obstacle_type, perception_obstacle_pb2.PerceptionObstacle.UNKNOWN
    )

    out_msg.position.x = actor.pose.px
    out_msg.position.y = actor.pose.py
    out_msg.position.z = actor.pose.pz
    out_msg.theta = spatial.pose_proto_to_heading(actor.pose)

    _copy_t(actor.velocity, out_msg.velocity)

    out_msg.length = actor.legacy.length
    out_msg.width = actor.legacy.width
    out_msg.height = actor.legacy.height
    for point_in in actor.legacy.polygon_points:
        point_out = out_msg.polygon_point.add()
        point_out.x = point_in.x
        point_out.y = point_in.y
        point_out.z = point_in.z


def create_perception_obstacles_msg(sim_time, actors):
    """Creates Apollo PerceptionObstacles message at given time with actor list

    For the header, it only fills in timestamp_sec. In particular,
    sequence number and module name are left empty.
    """
    perception_obstacles_msg = perception_obstacle_pb2.PerceptionObstacles()
    perception_obstacles_msg.header.timestamp_sec = sim_time
    for actor in actors:
        out = perception_obstacles_msg.perception_obstacle.add()
        _set_perception_obstacle_info(actor, out)

    return perception_obstacles_msg
