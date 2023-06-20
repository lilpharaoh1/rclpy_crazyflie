import sys

import rclpy
from rclpy.node import Node
from cf_msgs.srv import SendHoverSetpoint, ResetPositionEstimator, SetParam, VelocityControl


class CrazyflieClient(Node):

    def __init__(self):
        super().__init__('client_node')
        self._name = '/E7E7E7E7E7'

        self.send_hover_setpoint_client = self.create_client(SendHoverSetpoint, self._name + '/send_hover_setpoint')
        self.reset_position_estimator_client = self.create_client(ResetPositionEstimator, self._name + '/reset_position_estimator')
        self.set_param_client = self.create_client(SetParam, self._name + '/set_param')
        self.velocity_control_client = self.create_client(VelocityControl, self._name + '/velocity_control')

        self.set_param_timer = self.create_timer(2, self.timer_cb)

        while not self.set_param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set param service not available, waiting again...')
        self.set_param_req = SetParam.Request()

    def timer_cb(self):
        pass

    def enable_motors(self):
        self.set_param_req.param = 'motorPowerSet.enable'
        self.set_param_req.value = '1'
        self.future = self.set_param_client.call_async(self.set_param_req)
        return self.future.result()

    def set_motor_speed(self, speed=1000):
        self.set_param_req.param = 'motorPowerSet.m1'
        self.set_param_req.value = str(speed)
        self.future = self.set_param_client.call_async(self.set_param_req)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)
    client_node = CrazyflieClient()
    rclpy.spin(client_node)
    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()