import rclpy
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
import numpy as np
import time
import threading

from std_msgs.msg import String, Float32
from cf_msgs.msg import HoverStamped, FlyStamped

class RandomMovementCallBackGroup(CallbackGroup):
    def __init__(self):
        super().__init__()
        self.not_done = True

    def can_execute(self, entity) -> bool:
        return self.not_done
    
    def beginning_execution(self, entity) -> bool:
        return True
    
    def ending_execution(self, entity) -> None:
        pass

class RandomMovement(Node):

    HOVER_PERIOD = 10

    def __init__(self):
        super().__init__('rm_cf_publisher')
        self.declare_parameter('name', '')
        self._name = self.get_parameter('name').get_parameter_value().string_value

        self.cb_group = RandomMovementCallBackGroup()

        self.hover_pub = self.create_publisher(HoverStamped, self._name + '/control/hover', 10)
        self.take_off_pub = self.create_publisher(Float32, self._name + '/control/take_off', 10)
        self.land_pub = self.create_publisher(Float32, self._name+ '/control/land', 10)

        self.hover_timer = self.create_timer(0.05, self.hover_cb, callback_group=self.cb_group)
        
        threading.Thread(target=self.take_off_control).start()

    def hover_cb(self):
        msg = HoverStamped()
        msg.stamp.stamp = self.get_clock().now().to_msg()
        msg.hover.vx = 0.0#0.2*np.sin(6 * np.deg2rad(int(time.time()) % 60))
        msg.hover.vy = 0.0#0.2*np.cos(6 * np.deg2rad(int(time.time()) % 60))
        msg.hover.yaw_rate = 0.0
        msg.hover.height = 0.5
        self.hover_pub.publish(msg)

    def take_off_control(self):
        time.sleep(60)
        self.cb_group.not_done = False
        self.get_logger().info('Demo done...')

def main(args=None):
    rclpy.init(args=args)
    rm_cf_node = RandomMovement()
    rclpy.spin(rm_cf_node)
    rm_cf_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()