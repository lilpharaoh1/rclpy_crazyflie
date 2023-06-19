
import numpy as np
import pickle
import time

from cflib.crazyflie import Crazyflie
from cflib.positioning.motion_commander import MotionCommander

import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt16
from geometry_msgs.msg import Vector3

# from rospy_crazyflie.msg import *
from cf_msgs.srv import SendHoverSetpoint
from motion_commander import *

class CrazyflieControl(Node):

    def __init__(self, name, crazyflie):
        super().__init__(name + '_control')
        # Instantiate motion commander
        self._cf = crazyflie
        self._name = name
        self._mc = MotionCommander(self._cf)

        self._send_hover_setpoint_srv = self.create_service(
            SendHoverSetpoint,
            self._name + '/send_hover_setpoint',
            self._send_hover_setpoint_cb
        )

    def _send_hover_setpoint_cb(self, req):
        self.get_logger().info("SendHoverSetpoint called...")
        vx = req.vx
        vy = req.vy
        z = req.z
        yaw_rate = req.yaw_rate
        self._cf.commander.send_hover_setpoint(vx, vy, yaw_rate, z)
        return []
    
def main(args=None):
    rclpy.init(args=args)
    control_node = CrazyflieControl()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

