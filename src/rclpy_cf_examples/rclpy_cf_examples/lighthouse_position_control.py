import rclpy
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
import numpy as np
import time
import threading
import sys

from std_msgs.msg import String, Float32
from geometry_msgs.msg import PointStamped, PoseStamped
from cf_msgs.msg import HoverStamped, FlyStamped, PositionStamped, StateEstimate

waypoints = np.array([[1.0,  0.0,  1.0],
                      [1.0,  0.5,  1.0],
                      [-1.0, 0.5,  0.4],
                      [1.0,  -0.8,  0.4],
                      [0.0,  0.0,  1.3],
                      [-0.5, 0.5,  1.3],
                      [0.5, 1.0, 0.6],
                      [1.0,  0.0,  1.0],
                      [1.0,  0.5,  1.0],
                      [-1.0, 0.5,  0.4],
                      [1.0,  -0.8,  0.4],
                      [0.0,  0.0,  1.3],
                      [-0.5, 0.5,  1.3],
                      [0.5, 1.0, 0.6]])

class PositionControlCallBackGroup(CallbackGroup):
    def __init__(self):
        super().__init__()
        self.not_done = True

    def can_execute(self, entity) -> bool:
        return self.not_done
    
    def beginning_execution(self, entity) -> bool:
        return True
    
    def ending_execution(self, entity) -> None:
        pass

class PositionControl(Node):

    HOVER_PERIOD = 10
    POINT_TOLERANCE = 0.2

    def __init__(self, name):
        super().__init__('pc_cf_publisher')
        self._name = name
        self.pose = np.zeros((6, 1))
        self.waypoints = waypoints
        self.obj = None if not len(self.waypoints) else self.waypoints[0].reshape((3,1))

        self.cb_group = PositionControlCallBackGroup()

        self.position_pub = self.create_publisher(PointStamped, self._name + '/control/position/global', 10)
        self.pose_sub = self.create_subscription(PoseStamped, self._name + '/pose/global', self.pose_cb, 10)
        
    def waypoint_select(self, at):
        if not at:
            return self.obj
        time.sleep(0.5)
        self.waypoints = np.delete(self.waypoints, 0, axis=0)
        self.obj = None if not len(self.waypoints) else self.waypoints[0].reshape((3,1))

    def at(self, point):
        if np.allclose(point, self.pose[0:3], atol=self.POINT_TOLERANCE):
            return True
        return False

    def pose_cb(self, data):
        self.pose[0, 0] = data.pose.position.x
        self.pose[1, 0] = data.pose.position.y
        self.pose[2, 0] = data.pose.position.z

        self.waypoint_select(self.at(self.obj))
        self.get_logger().info('Next Waypoint : ' + str(self.obj))
        if not isinstance(self.obj, np.ndarray):
            self.cb_group.not_done = False
            time.sleep(5)
            self.destroy_node()
            while(True):
                time.sleep(1)
        else:
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'world'
            msg.point.x = self.obj[0, 0]
            msg.point.y = self.obj[1, 0]
            msg.point.z = self.obj[2, 0]
            self.position_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    name = sys.argv[1]
    pc_cf_node = PositionControl(name)
    rclpy.spin(pc_cf_node)
    pc_cf_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()