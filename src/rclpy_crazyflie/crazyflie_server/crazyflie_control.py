
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
from cf_msgs.srv import SendHoverSetpoint, ResetPositionEstimator, SetParam, VelocityControl
from motion_commander.velocity_primitives import *
from motion_commander.position_primatives import *

DEFAULT_HEIGHT = 0.5

class CrazyflieControl(Node):
    def __init__(self, name, crazyflie):
        super().__init__(name + '_control')
        # Instantiate motion commander
        self._cf = crazyflie
        self._name = name
        self._mc = MotionCommander(self._cf)

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
        
    def _reset_position_estimator_cb(self, req):
        pass

    def _send_hover_setpoint_cb(self, req):
        self.get_logger().info("SendHoverSetpoint called...")
        vx = req.vx
        vy = req.vy
        z = req.z
        yaw_rate = req.yaw_rate
        self._cf.commander.send_hover_setpoint(vx, vy, yaw_rate, z)
        return []
    
    def _set_param_cb(self, req):
        self._cf.param.set_value(req.param, req.value)
        self.get_logger().info("set %s to %s" % (req.param, req.value))
        return []

    def _velocity_control_cb(self, req):
        try:
            obj = pickle.loads(req.pickle)
            print(self._mc)
            if isinstance(obj, SetVelSetpoint):
                self._mc._set_vel_setpoint(obj.vx, obj.vy, obj.vz, obj.rate_yaw)
            elif isinstance(obj, StartBack):
                self._mc.start_back(velocity = obj.velocity)
            elif isinstance(obj, StartCircleLeft):
                self._mc.start_circle_left(obj.radius_m, velocity = obj.velocity)
            elif isinstance(obj, StartCircleRight):
                self._mc.start_turn_right(obj.radius_m, velocity = obj.velocity)
            elif isinstance(obj, StartDown):
                self._mc.start_down(velocity = obj.velocity)
            elif isinstance(obj, StartForward):
                self._mc.start_forward(velocity = obj.velocity)
            elif isinstance(obj, StartLeft):
                self._mc.start_left(velocity = obj.velocity)
            elif isinstance(obj, StartLinearMotion):
                self._mc.start_linear_motion(obj.vx, obj.vy, obj.vz)
            elif isinstance(obj, StartRight):
                self._mc.start_right(velocity = obj.velocity)
            elif isinstance(obj, StartTurnLeft):
                self._mc.start_turn_left(rate = obj.rate)
            elif isinstance(obj, StartTurnRight):
                self._mc.start_turn_right(rate = obj.rate)
            elif isinstance(obj, StartUp):
                self._mc.start_up(velocity = obj.velocity)
            elif isinstance(obj, Stop):
                self._mc.stop()
            else:
                return 'Object is not a valid velocity command'
        except Exception as e:
            print(str(e))
            raise e
        return 'ok'
    
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

