import math
import sys

from geometry_msgs.msg import TransformStamped, PoseStamped

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

def euler_from_quaternion(q):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        x, y, z, w = q.x, q.y, q.z, q.w
        
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians


class PoseTransform(Node):
    """
    Broadcast transforms that never change.

    This example publishes transforms from `world` to a static turtle frame.
    The transforms are only published once at startup, and are constant for all
    time.
    """

    def __init__(self, name, initial_pose):
        super().__init__('pose_tf_node_' + name)
        self._name = name
        self._initial_pose = initial_pose

        self.local_pose_sub = self.create_subscription(PoseStamped, name + '/pose/local', self.local_pose_cb, 10)
        self.global_pose_pub = self.create_publisher(PoseStamped, name + '/pose/global', 1)

    def local_pose_cb(self, data):
        msg = PoseStamped()
        msg.header.stamp = data.header.stamp
        msg.header.frame_id = 'world'

        msg.pose.position.x = data.pose.position.x - float(self._initial_pose[0])
        msg.pose.position.y = data.pose.position.y - float(self._initial_pose[1])
        msg.pose.position.z = data.pose.position.z - float(self._initial_pose[2])

        yaw, pitch, roll = euler_from_quaternion(data.pose.orientation)
        yaw += float(self._initial_pose[3])
        q = euler_to_quaternion([yaw, pitch, roll])

        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        self.global_pose_pub.publish(msg)


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