import rclpy
from rclpy.node import Node
from cf_msgs.srv import PoseTransform

class PoseTFHandler(Node):
    SERVICE_MESSAGES = False
    def __init__(self, name):
        super().__init__('pose_tf_handler')
        self._se_tf_client = self.create_client(PoseTransform, name + '/se_transform')

        while not self._se_tf_client.wait_for_service(timeout_sec=1.0):
            if self.SERVICE_MESSAGES:
                self.get_logger().info('reset_position_estimator service not available, waiting again...')
        self._se_tf_req = PoseTransform.Request()

    def waypoint(self, x, y, z, yaw):
        self._se_tf_req.x = float(x)
        self._se_tf_req.y = float(y)
        self._se_tf_req.z = float(z) 
        self._se_tf_req.yaw = float(yaw)

        self.future = self._se_tf_client.call_async(self._se_tf_req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def tf_handler(name, point, response, waypoint=True):
    rclpy.init()
    pose_tf_handler = PoseTFHandler()
    response.append(pose_tf_handler.waypoint(*point) if waypoint else pose_tf_handler.pose(*point))
    pose_tf_handler.destroy_node()
    rclpy.shutdown()
