"""
Convert between Applied Pose and ROS Transform.
"""
# Simian imports
# Ros imports
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Transform

from simian.public.proto import spatial_pb2


def applied_pose_to_ros_transform(pose):
    ros_transform = Transform()
    ros_transform.translation.x = pose.px
    ros_transform.translation.y = pose.py
    ros_transform.translation.z = pose.pz
    ros_transform.rotation.w = pose.qw
    ros_transform.rotation.x = pose.qx
    ros_transform.rotation.y = pose.qy
    ros_transform.rotation.z = pose.qz
    return ros_transform


def applied_pose_to_ros_pose(pose):
    ros_pose = Pose()
    ros_pose.position.x = pose.px
    ros_pose.position.y = pose.py
    ros_pose.position.z = pose.pz
    ros_pose.orientation.w = pose.qw
    ros_pose.orientation.x = pose.qx
    ros_pose.orientation.y = pose.qy
    ros_pose.orientation.z = pose.qz
    return ros_pose


def ros_transform_to_applied_pose(ros_transform):
    pose = spatial_pb2.Pose()
    pose.px = ros_transform.translation.x
    pose.py = ros_transform.translation.y
    pose.pz = ros_transform.translation.z
    pose.qw = ros_transform.rotation.w
    pose.px = ros_transform.rotation.x
    pose.qy = ros_transform.rotation.y
    pose.qz = ros_transform.rotation.z
    return pose
