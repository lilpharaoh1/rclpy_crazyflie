import rclpy
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
import numpy as np
import time
import threading
import sys

from std_msgs.msg import String, Float32
from geometry_msgs.msg import PointStamped

class SwarmTakeoffCallBackGroup(CallbackGroup):
    def __init__(self):
        super().__init__()
        self.not_done = True

    def can_execute(self, entity) -> bool:
        return self.not_done
    
    def beginning_execution(self, entity) -> bool:
        return True
    
    def ending_execution(self, entity) -> None:
        pass

class SwarmTakeoff(Node):
    def __init__(self, name):
        super().__init__('pc_cf_publisher')
        self._name = name
        self.takeoff_pub = self.create_publisher(Float32, self._name + '/control/take_off', 1)
        self.land_pub = self.create_publisher(Float32, self._name + '/control/land', 1)
        self.sequence()
    
    def sequence(self):
        takeoff_msg = Float32(data=0.5)
        self.takeoff_pub.publish(takeoff_msg)

        time.sleep(10)

        land_msg = Float32(data=0.0)
        self.land_pub.publish(land_msg)

        time.sleep(5)

        self.get_logger().info('Demo done...')

def main(args=None):
    rclpy.init(args=args)
    name = sys.argv[1]
    swarm_takeoff = SwarmTakeoff(name)
    rclpy.spin(swarm_takeoff)
    swarm_takeoff.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()