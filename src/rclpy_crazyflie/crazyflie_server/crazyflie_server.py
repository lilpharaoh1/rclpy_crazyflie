from curses.ascii import CR
import rclpy
from rclpy.node import Node
from cflib.crazyflie import Crazyflie
import cflib.crtp
from .crazyflie_control import CrazyflieControl
from .crazyflie_log import CrazyflieLog
import threading
import time

from std_msgs.msg import String, Bool

class CrazyflieServer(Node):
    CONNECTION_PUB_TIME = 0.05 # s
    CONNECTION_EST_TIME = 30 # s
    RECONNECT = True

    def __init__(self):
        super().__init__('server_node')
        cflib.crtp.init_drivers(enable_debug_driver=False)
        self.declare_parameter('uris', ['radio://0/80/2M/E7E7E7E7E7'])
        self.declare_parameter('lighthouse', False)
        self.declare_parameter('log_rpy_rate', False)
        self.declare_parameter('log_rpyt', False)
        self.declare_parameter('log_se', False)
        self.declare_parameter('log_kpe', False)
        self.declare_parameter('log_pc', False)
        self.declare_parameter('log_mp', False)
        self.declare_parameter('log_sta', False)

        self._uris = self.get_parameter('uris').get_parameter_value().string_array_value
        self._lighthouse = self.get_parameter('lighthouse').get_parameter_value().bool_value
        self._connection_pubs = {}
        self._crazyflies = {}
        self._crazyflie_logs = {}
        self._controllers = {}

        for uri in self._uris:
            name = uri.split('/')[-1]
            cf = Crazyflie()
            cf.connected.add_callback(self._connected)
            cf.disconnected.add_callback(self._disconnected)
            cf.connection_failed.add_callback(self._connection_failed)
            cf.connection_lost.add_callback(self._connection_lost)
            cf.open_link(uri)
            self._crazyflies[uri] = [name, cf, 0]
            self._connection_pubs[uri] = self.create_publisher(Bool, name + '/connection', 10)
        
        self._connection_timer = self.create_timer(self.CONNECTION_PUB_TIME, self._connection_cb)

    def _connected(self, link_uri):
        self.get_logger().info('Connected to %s.' % (link_uri))
        if link_uri not in self._crazyflie_logs:
            logs = self._unpack_log_params()
            name, uri, _ = self._crazyflies[link_uri]
            log = CrazyflieLog(name, uri, c_rpy_rate=logs['c_rpy_rate'], c_rpyt=logs['c_rpyt'], se=logs['se'], \
                               kpe=logs['kpe'], pc=logs['pc'], mp=logs['mp'], sta=logs['sta'], lighthouse=self._lighthouse)
            self._crazyflie_logs[link_uri] = log
            self._node_spinner(self._crazyflie_logs[link_uri])
        if link_uri not in self._controllers:
            name, uri, _ = self._crazyflies[link_uri]
            controller = CrazyflieControl(name, uri)
            self._controllers[link_uri] = controller
            self._node_spinner(self._controllers[link_uri])

        self._crazyflies[link_uri][2] = 1

    def _connection_failed(self, link_uri, msg):
        """Callback when initial connection fails (i.e. no Crazyflie
        at the specified address)"""
        self.get_logger().info('Connection to %s failed: %s' % (link_uri, msg))
        self._crazyflies[link_uri][2] = 0
        if self.RECONNECT:
            time.sleep(self.CONNECTION_EST_TIME)
            self.get_logger().info(f"Attempting to reconnect with {link_uri}...")
            self._crazyflies[link_uri][1].open_link(link_uri)

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e.
        Crazyflie moves out of range)"""
        pass


    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        self.get_logger().info('Disconnected from %s' % link_uri)
        self._crazyflies[link_uri][2] = 0
        logs = self._crazyflie_logs.pop(link_uri, None)
        control = self._controllers.pop(link_uri, None)
        if logs is not None:
            logs.destroy_node()
        if control is not None:
            control.destroy_node()
        if self.RECONNECT:
            time.sleep(self.CONNECTION_EST_TIME)
            self.get_logger().info(f"Attempting to reconnect with {link_uri}...")
            self._crazyflies[link_uri][1].open_link(link_uri)

    def _node_spinner(self, node):
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()

    def _unpack_log_params(self):
        c_rpy_rate = self.get_parameter('log_rpy_rate').get_parameter_value().bool_value
        c_rpyt = self.get_parameter('log_rpyt').get_parameter_value().bool_value
        se = self.get_parameter('log_se').get_parameter_value().bool_value
        kpe = self.get_parameter('log_kpe').get_parameter_value().bool_value
        pc = self.get_parameter('log_pc').get_parameter_value().bool_value
        mp = self.get_parameter('log_mp').get_parameter_value().bool_value
        sta = self.get_parameter('log_sta').get_parameter_value().bool_value
        
        logs = {
            'c_rpy_rate': c_rpy_rate,
            'c_rpyt': c_rpyt,
            'se': se,
            'kpe': kpe,
            'pc': pc,
            'mp': mp,
            'sta': sta
        }

        return logs

    def _connection_cb(self):
        for uri in self._uris:
            if self._crazyflies[uri][2]:
                self._connection_pubs[uri].publish(Bool(data=True))
            else:
                self._connection_pubs[uri].publish(Bool(data=False))


def main(args=None):
    rclpy.init(args=args)
    server_node = CrazyflieServer()
    rclpy.spin(server_node)
    server_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()