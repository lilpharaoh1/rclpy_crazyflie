import math
import sys

from geometry_msgs.msg import TransformStamped

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

def euler_to_quaternion(r):
    (yaw, pitch, roll) = (r[0], r[1], r[2])
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]


class PoseTransform(Node):
    """
    Broadcast transforms that never change.

    This example publishes transforms from `world` to a static turtle frame.
    The transforms are only published once at startup, and are constant for all
    time.
    """

    def __init__(self, name, initial_pose):
        super().__init__('pose_tf_node_' + name)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # Publish static transforms once at startup
        self.make_transforms(name, initial_pose)

    def make_transforms(self, name, initial_pose):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = '/local/' + name
        t.child_frame_id = '/world/' + name

        t.transform.translation.x = -float(initial_pose[0])
        t.transform.translation.y = -float(initial_pose[1])
        t.transform.translation.z = -float(initial_pose[2])
        qx, qy, qz, qw = euler_to_quaternion(
            (float(initial_pose[3]), float(0.0), float(0.0)))
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_static_broadcaster.sendTransform(t)


def main():
    logger = rclpy.logging.get_logger('pose_tf_logger')
    name = sys.argv[1]
    initial_pose = sys.argv[2:6] # [x, y, z, yaw]
    logger.info("initial_pose : " + str(sys.argv[2:6]))
    # pass parameters and initialize node
    rclpy.init()
    node = PoseTransform(name, initial_pose)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()