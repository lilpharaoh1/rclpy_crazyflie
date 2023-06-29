from geometry_msgs.msg import TransformStamped

import numpy as np

import rclpy
from rclpy.node import Node

from cf_msgs.srv import PoseTransform


class PoseTransforrmer(Node):
    def __init__(self):
        super().__init__('pose_tranformer')
        self.declare_parameter('name', 'E7E7E7E7E7')
        self.declare_parameter('transform_xyzyaw', [0.0, 0.0, 0.0, 0.0])
        self._name = self.get_parameter('name').get_parameter_value().string_value
        self.dx, self.dy, self.dz, self.dyaw = self.get_parameter('transform_xyzyaw').get_parameter_value().double_array_value

        self.get_logger().info(str(self._name))

        self._se_srv = self.create_service(
            PoseTransform, 
            self._name + '/se_transform',
            self._se_cb
        )

    def _se_cb(self, req, response):
        response.x = req.x + self.dx
        response.y = req.y + self.dy
        response.z = req.z + self.dz
        response.yaw = req.yaw + self.dyaw
        return response

def main():
    # pass parameters and initialize node
    rclpy.init()
    node = PoseTransforrmer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()