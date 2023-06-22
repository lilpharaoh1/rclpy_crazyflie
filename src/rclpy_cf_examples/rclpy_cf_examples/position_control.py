import rclpy
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
import numpy as np
import time
import threading

from std_msgs.msg import String, Float32
from geometry_msgs.msg import PointStamped
from cf_msgs.msg import HoverStamped, FlyStamped, PositionStamped, KalmanPositionEst

waypoints = np.array([[0.0,  0.0,  1.0],
                      [0.7,  0.0,  1.0],
                      [0.7,  0.5,  1.0],
                      [0.0, 0.5, 1.0],
                      [0.0, 0.5, 0.55],
                      [0.0, 0.0, 0.55],
                      [0.0, 0.0, 1.3],
                      [0.1, 0.0, 1.3],
                      [0.1, -0.5, 1.3],
                      [-0.3, -0.5, 0.7]])

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

    def __init__(self):
        super().__init__('pc_cf_publisher')
        self.declare_parameter('name', '')
        self._name = self.get_parameter('name').get_parameter_value().string_value
        self.pose = np.zeros((6, 1))
        self.waypoints = waypoints
        self.obj = None if not len(self.waypoints) else self.waypoints[0].reshape((3,1))

        self.cb_group = PositionControlCallBackGroup()

        self.position_pub = self.create_publisher(PositionStamped, self._name + '/control/position', 10)
        self.land_pub = self.create_publisher(Float32, self._name + '/control/land', 10)
        self.waypoint_pub = self.create_publisher(PointStamped, self._name + '/rviz/waypoint', 10)
        self.pose_pub = self.create_publisher(PointStamped, self._name + '/rviz/pose', 10)
        self.pose_sub = self.create_subscription(KalmanPositionEst, self._name + '/logging/KalmanPositionEst', self.pose_cb, 10)

        self.rviz_timer = self.create_timer(0.05, self.rviz_cb)#, callback_group=self.cb_group)
        
    def waypoint_select(self, at):
        if not at:
            return self.obj
        time.sleep(2)
        self.waypoints = np.delete(self.waypoints, 0, axis=0)
        self.obj = None if not len(self.waypoints) else self.waypoints[0].reshape((3,1))

    def at(self, point):
        if np.allclose(point, self.pose[0:3], atol=self.POINT_TOLERANCE):
            return True
        return False

    def pose_cb(self, data):
        self.pose[0, 0] = data.state_x
        self.pose[1, 0] = data.state_y
        self.pose[2, 0] = data.state_z

        self.waypoint_select(self.at(self.obj))
        self.get_logger().info('Next Waypoint : ' + str(self.obj))
        if not isinstance(self.obj, np.ndarray):
            self.land()
            time.sleep(5)
            self.destroy_node()
            while(True):
                time.sleep(1)
        else:
            msg = PositionStamped()
            msg.stamp.stamp = self.get_clock().now().to_msg()
            msg.position.x = self.obj[0, 0]
            msg.position.y = self.obj[1, 0]
            msg.position.z = self.obj[2, 0]
            self.position_pub.publish(msg)


    def rviz_cb(self):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.point.x = self.obj[0, 0]
        msg.point.y = self.obj[1, 0]
        msg.point.z = self.obj[2, 0]
        self.waypoint_pub.publish(msg)

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.point.x = self.pose[0, 0]
        msg.point.y = self.pose[1, 0]
        msg.point.z = self.pose[2, 0]
        self.pose_pub.publish(msg)

    def land(self):
        land_msg = Float32(data=0.0)
        self.land_pub.publish(land_msg)
        self.cb_group.not_done = False
        self.get_logger().info('Demo done...')

def main(args=None):
    rclpy.init(args=args)
    pc_cf_node = PositionControl()
    rclpy.spin(pc_cf_node)
    pc_cf_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()