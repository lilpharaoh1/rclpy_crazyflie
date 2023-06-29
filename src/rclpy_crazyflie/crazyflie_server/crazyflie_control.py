
import numpy as np
import pickle
import time

from cflib.crazyflie import Crazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.positioning.position_hl_commander import PositionHlCommander

import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt16
from geometry_msgs.msg import Vector3

from cf_msgs.srv import SendHoverSetpoint, ResetPositionEstimator, SetParam, VelocityControl, PositionControl, TakeOff, Land, PoseTransform
from motion_commander.velocity_primitives import *
# from motion_commander.position_primatives import *

class CrazyflieControl(Node):
    DEFAULT_HEIGHT = 0.3
    DEFAULT_VELOCITY = 0.5
    SERVICE_MESSAGES = False
    
    def __init__(self, name: str, crazyflie : Crazyflie):
        super().__init__(name + '_control')
        # Instantiate motion commander
        self._cf = crazyflie
        self._name = name
        self._mc = MotionCommander(self._cf, default_height=self.DEFAULT_HEIGHT)
        self._pc = PositionHlCommander(self._cf, default_height=self.DEFAULT_HEIGHT)
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

        self._position_control_srv = self.create_service(
            PositionControl,
            self._name + '/position_control',
            self._position_control_cb
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

        self._se_tf_client = self.create_client(PoseTransform, self._name + '/se_transform')

        while not self._se_tf_client.wait_for_service(timeout_sec=1.0):
            if self.SERVICE_MESSAGES:
                self.get_logger().info('reset_position_estimator service not available, waiting again...')
        self._se_tf_req = PoseTransform.Request()
        
    def _reset_position_estimator_cb(self, req, response):
        response.response = True
        return response

    def _send_hover_setpoint_cb(self, req, response):
        if not self._mc._is_flying:
            self._takeoff()
        vx = req.vx
        vy = req.vy
        z_req = self._se_tf_handler(z=req.z)
        z = z_req.z
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
        try:
            self._mc._set_vel_setpoint(req.vx, req.vy, req.vz, req.yaw_rate)
        except Exception as e:
            self.get_logger().info('error occured in velocity control')
            print(str(e))
            response.response = False
            return False
        response.response = True
        return response
    
    def _position_control_cb(self, req, response):
        try:
            req = self._se_tf_handler(x=float(req.x), y=float(req.y), z=float(req.z))
            if req is not None:
                self.get_logger().info(str(req))
                self._pc.go_to(req.x, req.y, req.z, velocity=self.DEFAULT_VELOCITY) # TODO: FIX THIS!!!
        except Exception as e:
            self.get_logger().info('error occured in position control')
            print(str(e))
            response.response = False
            return False
        response.response = True
        return response
    
    def _take_off_cb(self, req, response):
        req = self._se_tf_handler(z=float(req.height))
        height = req.z
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
            self._pc.take_off(height = height)
        except BaseException as e:
            print(e)
            return

    def _land(self):
        try:
            self._mc.land()
            self._pc.land()
        except BaseException as e:
            print(e)
            return
        
    def _se_tf_handler(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        self._se_tf_req.x = float(x)
        self._se_tf_req.y = float(y)
        self._se_tf_req.z = float(z) 
        self._se_tf_req.yaw = float(yaw)
        self.future = self._se_tf_client.call_async(self._se_tf_req)
        self.get_logger().info('futrure done now...')
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info('futrure done now...')
        return self.future.result()
    
def main(args=None):
    rclpy.init(args=args)
    control_node = CrazyflieControl()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

