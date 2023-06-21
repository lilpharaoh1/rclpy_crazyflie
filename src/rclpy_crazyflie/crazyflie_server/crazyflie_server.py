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
    CONNECTION_TIME = 3 # s
    RECONNECT = False
    RECONNECT_ATTEMPTS = 5 # Try to reconnect this many times
    RECONNECT_TIMER = 5

    def __init__(self):
        super().__init__('server_node')
        cflib.crtp.init_drivers(enable_debug_driver=False)
        self.declare_parameter('uris', ['uri_empty'])
        self.declare_parameter('log_rpy_rate', False)
        self.declare_parameter('log_rpyt', False)
        self.declare_parameter('log_kpe', False)
        self.declare_parameter('log_pc', False)
        self.declare_parameter('log_mp', False)
        self.declare_parameter('log_sta', False)

        self._uris = self.get_parameter('uris').get_parameter_value().string_array_value
        self._connection_pubs = {}
        self._crazyflies = {}
        self._crazyflie_logs = {}
        self._controllers = {}

        for uri in self._uris:
            name = uri.split('/')[-1]
            self.get_logger().info(str(name))
            cf = Crazyflie()
            cf.connected.add_callback(self._connected)
            cf.disconnected.add_callback(self._disconnected)
            cf.connection_failed.add_callback(self._connection_failed)
            cf.connection_lost.add_callback(self._connection_lost)
            cf.open_link(uri)
            self._crazyflies[uri] = [name, cf, 0]
            self._connection_pubs[uri] = self.create_publisher(Bool, name + '/connection', 10)
        

        self._connection_timer = self.create_timer(self.CONNECTION_TIME, self._connection_cb)

    def _connected(self, link_uri):
        print('Connected to %s.' % (link_uri))
        if link_uri not in self._crazyflie_logs:
            logs = self._unpack_log_params()
            name, uri, _ = self._crazyflies[link_uri]
            log = CrazyflieLog(name, uri, c_rpy_rate=logs['c_rpy_rate'], c_rpyt=logs['c_rpyt'], \
                               kpe=logs['kpe'], pc=logs['pc'], mp=logs['mp'], sta=logs['sta'])
            self._crazyflie_logs[link_uri] = log
            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(self._crazyflie_logs[link_uri])
            thread = threading.Thread(target=executor.spin, daemon=True)
            thread.start()
        if link_uri not in self._controllers:
            name, uri, _ = self._crazyflies[link_uri]
            controller = CrazyflieControl(name, uri)
            self._controllers[link_uri] = controller
            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(self._controllers[link_uri])
            thread = threading.Thread(target=executor.spin, daemon=True)
            thread.start()
        self._crazyflies[link_uri][2] = 1

    def _connection_failed(self, link_uri, msg):
        """Callback when initial connection fails (i.e. no Crazyflie
        at the specified address)"""
        self.get_logger().info('Connection to %s failed: %s' % (link_uri, msg))
        self._crazyflies[link_uri][2] = 0
        if self.RECONNECT:
            self.get_logger().info('Attempting reconnect with %s' % link_uri)
            self._crazyflies[link_uri][1].open_link(link_uri)
            # for attempt in range(self.RECONNECT_ATTEMPTS):
            #     try:
            #         self._crazyflies[link_uri][1].open_link(link_uri)
            #         break
            #     except:
            #         time.sleep(self.RECONNECT_TIMER)
            #         continue


    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e.
        Crazyflie moves out of range)"""
        self.get_logger().info('Connection to %s lost : %s' % (link_uri, msg))
        self._crazyflies[link_uri][2] = 0
        if self.RECONNECT:
            self.get_logger().info('Attempting reconnect with %s' % link_uri)
            self._crazyflies[link_uri][1].open_link(link_uri)
            # for attempt in range(self.RECONNECT_ATTEMPTS):
            #     try:
            #         self._crazyflies[link_uri][1].open_link(link_uri)
            #         break
            #     except:
            #         time.sleep(self.RECONNECT_TIMER)
            #         continue


    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        self.get_logger().info('Disconnected from %s' % link_uri)
        self._crazyflies[link_uri][2] = 0
        if self.RECONNECT:
            self.get_logger().info('Attempting reconnect with %s' % link_uri)
            self._crazyflies[link_uri][1].open_link(link_uri)
            # for attempt in range(1, self.RECONNECT_ATTEMPTS+1):
                # try:
                #     self.get_logger().info('Reconnection Attempt %i with %s' % (attempt, link_uri))
                #     self._crazyflies[link_uri][1].open_link(link_uri)
                #     self.get_logger().info('Reconnection succeed with %s' % link_uri)
                #     break
                # except:
                #     time.sleep(self.RECONNECT_TIMER)
                #     continue

    def _unpack_log_params(self):
        c_rpy_rate = self.get_parameter('log_rpy_rate').get_parameter_value().bool_value
        c_rpyt = self.get_parameter('log_rpyt').get_parameter_value().bool_value
        kpe = self.get_parameter('log_kpe').get_parameter_value().bool_value
        pc = self.get_parameter('log_pc').get_parameter_value().bool_value
        mp = self.get_parameter('log_mp').get_parameter_value().bool_value
        sta = self.get_parameter('log_sta').get_parameter_value().bool_value
        
        logs = {
            'c_rpy_rate': c_rpy_rate,
            'c_rpyt': c_rpyt,
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