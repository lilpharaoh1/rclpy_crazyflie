
import numpy as np
import pickle
import time

from cflib.crazyflie import Crazyflie
from cflib.positioning.motion_commander import MotionCommander

import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt16
from geometry_msgs.msg import Vector3

from cf_msgs.srv import SendHoverSetpoint, ResetPositionEstimator, SetParam, VelocityControl, TakeOff, Land
from motion_commander.velocity_primitives import *
# from motion_commander.position_primatives import *

class CrazyflieControl(Node):
    DEFAULT_HEIGHT = 0.3
    
    def __init__(self, name: str, crazyflie : Crazyflie):
        super().__init__(name + '_control')
        # Instantiate motion commander
        self._cf = crazyflie
        self._name = name
        self._mc = MotionCommander(self._cf)
        self._is_flying = False

        self._reset_position_estimator_srv = self.create_service(
            ResetPositionEstimator,
            self._name + '/reset_position_estimator',
            self._reset_position_estimator_cb
        )

        self._send_hover_setpoint_srv = self.create_service(
            SendHoverSetpoint,
            self._name + '/send_hover_setpoint',
            self._send_hover_setpoint_cb
        )

        self._set_param_srv = self.create_service(
            SetParam,
            self._name + '/set_param',
            self._set_param_cb
        )

        self._velocity_control_srv = self.create_service(
            VelocityControl,
            self._name + '/velocity_control',
            self._velocity_control_cb
        )

        self._take_off_srv = self.create_service(
            TakeOff,
            self._name + '/take_off',
            self._take_off_cb
        )

        self.land_srv = self.create_service(
            Land,
            self._name + '/land',
            self._land_cb
        )
        
    def _reset_position_estimator_cb(self, req, response):
        response.response = True
        return response

    def _send_hover_setpoint_cb(self, req, response):
        vx = req.vx
        vy = req.vy
        z = req.z
        yaw_rate = req.yaw_rate
        self._cf.commander.send_hover_setpoint(vx, vy, yaw_rate, z)
        response.response = True
        return response
    
    def _set_param_cb(self, req, response):
        self._cf.param.set_value(req.param, req.value)
        self.get_logger().info("set %s to %s" % (req.param, req.value))
        response.response = True
        return response

    def _velocity_control_cb(self, req, response):
        if not self._mc._is_flying:
            self._takeoff()
        try:
            self._mc._set_vel_setpoint(req.vx, req.vy, req.vz, req.yaw_rate)
        except Exception as e:
            self.get_logger().info('error occured in velocity control')
            print(str(e))
            response.response = False
            return False
        response.response = True
        return response
    
    def _take_off_cb(self, req, response):
        height = req.height
        self._takeoff(height=height)
        response.response = True
        return response
    
    def _land_cb(self, req, response):
        self._land()
        response.response = True
        return response
    
    def _takeoff(self, height=DEFAULT_HEIGHT):
        try:
            self._mc.take_off(height = height)
            time.sleep(5)
        except BaseException as e:
            print(e)
            return

    def _land(self):
        try:
            self._mc.land()
        except BaseException as e:
            print(e)
            return
    
def main(args=None):
    rclpy.init(args=args)
    control_node = CrazyflieControl()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

