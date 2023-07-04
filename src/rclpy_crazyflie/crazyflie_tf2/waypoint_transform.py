import math
import sys

from geometry_msgs.msg import TransformStamped, PointStamped

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class WaypointTransform(Node):
    """
    Broadcast transforms that never change.

    This example publishes transforms from `world` to a static turtle frame.
    The transforms are only published once at startup, and are constant for all
    time.
    """

    def __init__(self, name, initial_pose):
        super().__init__('waypoint_tf_node_' + name)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self._name = name
        self._initial_pose = initial_pose

        self.global_waypoint_sub = self.create_subscription(PointStamped, name + '/control/position/global', self.global_waypoint_cb, 10)
        self.local_waypoint_pub = self.create_publisher(PointStamped, name + '/control/position/local', 1)

    def global_waypoint_cb(self, data):
        msg = PointStamped()
        msg.header.stamp = data.header.stamp
        msg.header.frame_id = 'local/' + self._name

        msg.point.x = data.point.x - float(self._initial_pose[0])
        msg.point.y = data.point.y - float(self._initial_pose[1])
        msg.point.z = data.point.z - float(self._initial_pose[2])

        self.local_waypoint_pub.publish(msg)

def main():
    logger = rclpy.logging.get_logger('waypoint_tf_logger')
    name = sys.argv[1]
    initial_pose = sys.argv[2:6] # [x, y, z, yaw]
    # pass parameters and initialize node
    rclpy.init()
    node = WaypointTransform(name, initial_pose)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()