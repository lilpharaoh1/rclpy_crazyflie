import sys
import pickle

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from cf_msgs.msg import FlyStamped
from cf_msgs.srv import SendHoverSetpoint, ResetPositionEstimator, SetParam, VelocityControl, TakeOff, Land
from motion_commander.velocity_primitives import *
from .connection_callback_group import ConnectionCallbackGroup


class CrazyflieClient(Node):

    def __init__(self):
        super().__init__('client_node')
        self.declare_parameter('uri', '')
        uri = self.get_parameter('uri').get_parameter_value().string_value
        self._name = uri.split('/')[-1]

        self.send_hover_setpoint_client = self.create_client(SendHoverSetpoint, self._name + '/send_hover_setpoint')
        self.reset_position_estimator_client = self.create_client(ResetPositionEstimator, self._name + '/reset_position_estimator')
        self.set_param_client = self.create_client(SetParam, self._name + '/set_param')
        self.velocity_control_client = self.create_client(VelocityControl, self._name + '/velocity_control')
        self.take_off_client = self.create_client(TakeOff, self._name + '/take_off')
        self.land_client = self.create_client(Land, self._name + '/land')

        self.connection_callback_group = ConnectionCallbackGroup(self._name)
        self.connection_sub = self.create_subscription(Bool, self._name + '/connection', self.connection_callback_group, 10)
        self.velocity_control_sub = self.create_subscription(FlyStamped, self._name + '/control/fly', self.fly_cb, 10, callback_group=self.connection_callback_group)
        self.take_off_sub = self.create_subscription(Float32, self._name + '/control/take_off', self.take_off_cb, 10, callback_group=self.connection_callback_group)
        self.land_sub = self.create_subscription(Float32, self._name + '/control/land', self.land_cb, 10, callback_group=self.connection_callback_group)

        while not self.send_hover_setpoint_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('send_hover_setpoint service not available, waiting again...')
        self.send_hover_setpoint_req = SendHoverSetpoint.Request()

        while not self.reset_position_estimator_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('reset_position_estimator service not available, waiting again...')
        self.reset_position_estimator_req = ResetPositionEstimator.Request()

        while not self.set_param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_param service not available, waiting again...')
        self.set_param_req = SetParam.Request()

        while not self.velocity_control_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('velocity_control service not available, waiting again...')
        self.velocity_control_req = VelocityControl.Request()

        while not self.take_off_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('take_off service not available, waiting again...')
        self.take_off_req = TakeOff.Request()

        while not self.land_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('land service not available, waiting again...')
        self.land_req = Land.Request()

    def fly_cb(self, data):
        vx = data.fly.linear_velocity.x
        vy = data.fly.linear_velocity.y
        vz = data.fly.linear_velocity.z
        yaw_rate = data.fly.yaw_rate
        _ = self.velocity_control_handler(vx, vy, vz, yaw_rate)

    def take_off_cb(self, data):
        height = data.data
        _ = self.take_off_handler(height)

    def land_cb(self, data):
        _ = self.land_handler()
    
    def send_hover_setpoint_handler(self, vx : float, vy : float, yaw_rate : float, z : float):
        self.send_hover_setpoint_req.vx = vx
        self.send_hover_setpoint_req.vy = vy
        self.send_hover_setpoint_req.yaw_rate = yaw_rate
        self.send_hover_setpoint_req.z = z
        self.future = self.send_hover_setpoint_client.call_async(self.send_hover_setpoint_req)
        return self.future.result()

    def reset_position_estimator_handler(self):
        self.future = self.reset_position_estimator_client.call_async(self.reset_position_estimator_req)
        return self.future.result()

    def set_param_handler(self, param : str, value : str):
        self.set_param_req.param = param
        self.set_param_req.value = value
        self.future = self.set_param_client.call_async(self.set_param_req)
        return self.future.result()
    
    def velocity_control_handler(self, vx : float, vy : float, vz : float, yaw_rate : float):
        self.velocity_control_req.vx = vx
        self.velocity_control_req.vy = vy
        self.velocity_control_req.vz = vz
        self.velocity_control_req.yaw_rate = yaw_rate
        self.future = self.velocity_control_client.call_async(self.velocity_control_req)
        return self.future.result()
    
    def take_off_handler(self, height : float):
        self.take_off_req.height = height
        self.future = self.take_off_client.call_async(self.take_off_req)
        return self.future.result()
    
    def land_handler(self):
        self.future = self.land_client.call_async(self.land_req)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)
    client_node = CrazyflieClient()
    rclpy.spin(client_node)
    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()