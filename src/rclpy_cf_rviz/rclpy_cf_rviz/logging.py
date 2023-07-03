import rclpy
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
import numpy as np
import time
import threading

from std_msgs.msg import String, Float32
from geometry_msgs.msg import PointStamped, PoseStamped
from cf_msgs.msg import FlyStamped, PositionStamped, StateEstimate, Stabilizer

def euler_to_quaternion(yaw, pitch, roll):
    yaw, pitch, roll = np.deg2rad(yaw), np.deg2rad(pitch), np.deg2rad(roll)
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return qx, qy, qz, qw


class PvizLoggerCallBackGroup(CallbackGroup):
    def __init__(self):
        super().__init__()
        self.not_done = True

    def can_execute(self, entity) -> bool:
        return self.not_done
    
    def beginning_execution(self, entity) -> bool:
        return True
    
    def ending_execution(self, entity) -> None:
        pass

class RvizLogger(Node):

    def __init__(self):
        super().__init__('rviz_logging_node')
        self.declare_parameter('name', '')
        self.declare_parameter('log_kpe', False)
        self.declare_parameter('log_sta', False)
        self.declare_parameter('pose', False)
        self.declare_parameter('waypoints', False)
        self.declare_parameter('velocities', False)


        self._name = self.get_parameter('name').get_parameter_value().string_value
        self.pose = [0.0, 0.0, 0.0]
        self.ypr = None
   
        if self.get_parameter('pose').get_parameter_value().bool_value:
            if self.get_parameter('log_kpe').get_parameter_value().bool_value:
                self.se_sub = self.create_subscription(StateEstimate, self._name + '/logging/StateEstimate', self.se_cb, 10)
                self.ypr = [0.0, 0.0, 0.0]
                self.sta_sub = self.create_subscription(Stabilizer, self._name + 'logging/Stabilizer', self.sta_cb, 10)
                self.pose_pose_pub = self.create_publisher(PoseStamped, self._name + '/rviz/pose/pose', 10)
                self.pose_point_pub = self.create_publisher(PointStamped, self._name + '/rviz/pose/point', 10)
                
                if self.get_parameter('velocities').get_parameter_value().bool_value:
                    self.velocity_sub = self.create_subscription(FlyStamped, self._name + '/control/fly', self.velocity_cb, 10)
                    self.velocity_pub = self.create_publisher(PoseStamped, self._name + '/rviz/velocities', 10)
            else:
                self.get_logger().info("Cannot display pose or velocities, please enable log_kpe...")
        
        if self.get_parameter('waypoints').get_parameter_value().bool_value:
            self.waypoints_sub = self.create_subscription(PointStamped, self._name + '/control/position', self.waypoints_cb, 10)
            self.waypoints_pub = self.create_publisher(PointStamped, self._name + '/rviz/waypoints', 10)

    def sta_cb(self, data):
        self.ypr[0] = data.yaw
        self.ypr[1] = data.pitch
        self.ypr[2] = data.roll

    def se_cb(self, data):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.point.x = data.x
        msg.point.y = data.y
        msg.point.z = data.z
        self.pose_point_pub.publish(msg)
        
        qx, qy, qz, qw = euler_to_quaternion(*self.ypr)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = data.x
        msg.pose.position.y = data.y
        msg.pose.position.z = data.z
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.pose_pose_pub.publish(msg)

        self.pose = [data.x, data.y, data.z]

    def velocity_sub(self, data):
        # qx, qy, qz, qw = euler_to_quaternion()
        pass # TODO : Do this later I'm tired...
    
    def waypoints_cb(self, data):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.point.x = data.position.x
        msg.point.y = data.position.y
        msg.point.z = data.position.z
        self.waypoints_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rviz_logging_node = RvizLogger()
    rclpy.spin(rviz_logging_node)
    rviz_logging_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()