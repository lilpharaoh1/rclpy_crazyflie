"""
Copyright (c) 2018, Joseph Sullivan
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the <project name> project.
"""

from geometry_msgs.msg import Vector3
from std_msgs.msg import UInt16
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig as cfLogConfig

import rclpy
from rclpy.node import Node
from cf_msgs.msg import ControllerRPYRate, ControllerRPYT, StateEstimate, KalmanPositionEst, MotorPower, PosCtl, Stabilizer
import json

class CrazyflieLog(Node):
    """
    This object manages the creation of vehicle telemetry logging configurations,
    and publishing log data to ros topics.
    """

    def __init__(self, name : str, crazyflie : Crazyflie, c_rpy_rate=False, c_rpyt=False, \
                se=False, kpe=False, mp=False, pc=False, sta=False, period_ms=100):
        super().__init__(name + '_log')
        self._name = name
        self._cf = crazyflie
        self._logs = {
            'c_rpy_rate' : c_rpy_rate,
            'c_rpyt' : c_rpyt,
            'se': se,
            'kpe' : kpe,
            'mp' : mp,
            'pc' : pc,
            'sta' : sta
        }

        # Set up log configs for standard log groups
        self._controller_rpy_rate_pub = self.create_publisher(
            ControllerRPYRate,
            self._name + '/logging/ControllerRPYRate',
            10
        )
        self._controller_rpy_rate_config = cfLogConfig(name='ControllerRPYRate', period_in_ms=100)
        self._controller_rpy_rate_config.data_received_cb.add_callback(self._controller_rpy_rate_cb)
        self._controller_rpy_rate_config.add_variable('controller.rollRate', 'float')
        self._controller_rpy_rate_config.add_variable('controller.pitchRate', 'float')
        self._controller_rpy_rate_config.add_variable('controller.yawRate', 'float')
        self._cf.log.add_config(self._controller_rpy_rate_config)


        self._controller_rpyt_pub = self.create_publisher(
            ControllerRPYT,
            self._name + '/logging/ControllerRPYT',
            10
        )
        self._controller_rpyt_config = cfLogConfig(name='ControllerRPYT', period_in_ms=100)
        self._controller_rpyt_config.data_received_cb.add_callback(self._controller_rpyt_cb)
        self._controller_rpyt_config.add_variable('controller.actuatorThrust', 'float')
        self._controller_rpyt_config.add_variable('controller.roll', 'float')
        self._controller_rpyt_config.add_variable('controller.pitch', 'float')
        self._controller_rpyt_config.add_variable('controller.yaw', 'float')
        self._cf.log.add_config(self._controller_rpyt_config)

        self._state_estimate_pub = self.create_publisher(
            StateEstimate,
            self._name + '/logging/StateEstimate',
            10
        )
        self._state_estimate_config = cfLogConfig(name='StateEstimate', period_in_ms=100)
        self._state_estimate_config.data_received_cb.add_callback(self._state_estimate_cb)
        self._state_estimate_config.add_variable('stateEstimate.x', 'float')
        self._state_estimate_config.add_variable('stateEstimate.y', 'float')
        self._state_estimate_config.add_variable('stateEstimate.z', 'float')
        # self._state_estimate_config.add_variable('stateEstimate.vx', 'float')
        # self._state_estimate_config.add_variable('stateEstimate.vy', 'float')
        # self._state_estimate_config.add_variable('stateEstimate.vz', 'float')
        # self._state_estimate_config.add_variable('stateEstimate.ax', 'float')
        # self._state_estimate_config.add_variable('stateEstimate.ay', 'float')
        # self._state_estimate_config.add_variable('stateEstimate.az', 'float')
        # self._state_estimate_config.add_variable('stateEstimate.yaw', 'float')
        # self._state_estimate_config.add_variable('stateEstimate.pitch', 'float')
        # self._state_estimate_config.add_variable('stateEstimate.roll', 'float')
        # self._state_estimate_config.add_variable('stateEstimate.qx', 'float')
        # self._state_estimate_config.add_variable('stateEstimate.qy', 'float')
        # self._state_estimate_config.add_variable('stateEstimate.qz', 'float')
        # self._state_estimate_config.add_variable('stateEstimate.qw', 'float')
        self._cf.log.add_config(self._state_estimate_config)

        self._kalman_position_pub = self.create_publisher(
            KalmanPositionEst,
            self._name + '/logging/KalmanPositionEst',
            10
        )
        self._kalman_position_config = cfLogConfig(name='KalmanPositionEst', period_in_ms=100)
        self._kalman_position_config.data_received_cb.add_callback(self._kalman_position_cb)
        self._kalman_position_config.add_variable('kalman.stateX', 'float')
        self._kalman_position_config.add_variable('kalman.stateY', 'float')
        self._kalman_position_config.add_variable('kalman.stateZ', 'float')
        self._cf.log.add_config(self._kalman_position_config)

        self._motor_power_pub = self.create_publisher(
            MotorPower,
            self._name + '/logging/MotorPower',
            10
        )
        self._motor_power_config = cfLogConfig(name='MotorPower', period_in_ms=100)
        self._motor_power_config.data_received_cb.add_callback(self._motor_power_cb)
        self._motor_power_config.add_variable('motor.m4', 'int32_t')
        self._motor_power_config.add_variable('motor.m1', 'int32_t')
        self._motor_power_config.add_variable('motor.m2', 'int32_t')
        self._motor_power_config.add_variable('motor.m3', 'int32_t')
        self._cf.log.add_config(self._motor_power_config)

        self._posCtl_pub = self.create_publisher(
            PosCtl,
            self._name + '/logging/posCtl',
            10
        )
        self._posCtl_config = cfLogConfig(name='posCtl', period_in_ms=100)
        self._posCtl_config.data_received_cb.add_callback(self._pos_ctl_cb)
        self._posCtl_config.add_variable('posCtl.targetVX', 'float')
        self._posCtl_config.add_variable('posCtl.targetVY', 'float')
        self._posCtl_config.add_variable('posCtl.targetVZ', 'float')
        self._posCtl_config.add_variable('posCtl.targetX', 'float')
        self._posCtl_config.add_variable('posCtl.targetY', 'float')
        self._posCtl_config.add_variable('posCtl.targetZ', 'float')
        self._cf.log.add_config(self._posCtl_config)

        self._stabilizer_pub = self.create_publisher(
            Stabilizer,
            self._name + '/logging/Stabilizer',
            10
        )
        self._stabilizer_config = cfLogConfig(name='Stabilizer', period_in_ms=100)
        self._stabilizer_config.data_received_cb.add_callback(self._stabilizer_cb)
        self._stabilizer_config.add_variable('stabilizer.roll', 'float')
        self._stabilizer_config.add_variable('stabilizer.pitch', 'float')
        self._stabilizer_config.add_variable('stabilizer.yaw', 'float')
        self._stabilizer_config.add_variable('stabilizer.thrust', 'uint16_t')
        self._cf.log.add_config(self._stabilizer_config)

        self.init_logs(period_ms)

    def init_logs(self, period_ms):
        if self._logs['c_rpy_rate'] : 
            self.log_controller_rpy_rate(period_in_ms=period_ms)
        if self._logs['c_rpyt'] :
            self.log_controller_rpyt(period_in_ms=period_ms)
        if self._logs['se']:
            self.log_state_estimate(period_in_ms=period_ms)
        if self._logs['kpe']:
            self.log_kalman_position_est(period_in_ms=period_ms)
        if self._logs['pc']:
            self.log_pos_ctl(period_in_ms=period_ms)
        if self._logs['mp']:
            self.log_motor_power(period_in_ms=period_ms)
        if self._logs['sta']:
            self.log_stabilizer(period_in_ms=period_ms)

    def _log_error_cb(self, logconf, msg):
        """
        Error handler from CrazyflieLibPython for all custom / ad hoc log configurations
        """
        # TODO:
        pass

    def stop_logs(self):
        """
        Stops log configurations,
        """
        for log_name in self._logs.keys():
            print("stopping log %s" %log_name)
            log = self._logs.pop(log_name)
            log['config'].stop()
            log['config'].delete()
            log['publisher'].unregister()
            del log

        self.stop_log_controller_rpy_rate()
        self.stop_log_controller_rpyt()
        self.stop_log_kalman_position_est()
        self.stop_log_motor_power()
        self.stop_log_pos_ctl()
        self.stop_log_stabilizer()

    def log_controller_rpy_rate(self, period_in_ms=100):
        """ Starts ControllerRPYRate log config"""
        self._controller_rpy_rate_config.period_in_ms = period_in_ms
        self._controller_rpy_rate_config.start()

    def log_controller_rpyt(self, period_in_ms=100):
        """ Starts ControllerRPYT log config """
        self._controller_rpyt_config.period_in_ms = period_in_ms
        self._controller_rpyt_config.start()

    def log_state_estimate(self, period_in_ms=100):
        """ Starts StateEstimate log config """
        self._state_estimate_config.period_in_ms = period_in_ms
        self._state_estimate_config.start()

    def log_kalman_position_est(self, period_in_ms=100):
        """ Starts KalmanPositionEst log config """
        self._kalman_position_config.period_in_ms = period_in_ms
        self._kalman_position_config.start()

    def log_motor_power(self, period_in_ms=100):
        """ Starts MotorPower log config """
        self._motor_power_config.period_in_ms = period_in_ms
        self._motor_power_config.start()

    def log_pos_ctl(self, period_in_ms=100):
        """ Starts MotorPower log config """
        self._posCtl_config.period_in_ms = period_in_ms
        self._posCtl_config.start()

    def log_stabilizer(self, period_in_ms=100):
        """ Starts Stabilizer log config """
        self._stabilizer_config.period_in_ms = period_in_ms
        self._stabilizer_config.start()

    def stop_log_controller_rpy_rate(self):
        """ Stops ControllerRPYRate log config"""
        self._controller_rpy_rate_config.stop()

    def stop_log_controller_rpyt(self):
        """ Stops ControllerRPYT log config """
        self._controller_rpyt_config.stop()

    def stop_log_state_estimate(self):
        """ Stops StateEstimate log config """
        self._state_estimate_config.stop()

    def stop_log_kalman_position_est(self):
        """ Stops KalmanPositionEst log config """
        self._kalman_position_config.stop()

    def stop_log_motor_power(self):
        """ Stops MotorPower log config """
        self._motor_power_config.stop()

    def stop_log_pos_ctl(self):
        """ Stops MotorPower log config """
        self._posCtl_config.stop()

    def stop_log_stabilizer(self):
        """ Stops Stabilizer log config """
        self._stabilizer_config.stop()

    def _controller_rpy_rate_cb(self, timestamp, data, logconfig):
        """ Callback from CrazyflieLibPython, publishes ControllerRPYRate messages """
        msg = ControllerRPYRate()
        msg.stamp.stamp = self.get_clock().now().to_msg()
        msg.cfstamp = timestamp
        msg.roll_rate = data['controller.rollRate']
        msg.pitch_rate = data['controller.pitchRate']
        msg.yaw_rate = data['controller.yawRate']
        self._controller_rpy_rate_pub.publish(msg)

    def _controller_rpyt_cb(self, timestamp, data, logconfig):
        """ Callback from CrazyflieLibPython, publishes ControllerRPYT messages """
        msg = ControllerRPYT()
        msg.stamp.stamp = self.get_clock().now().to_msg()
        msg.cfstamp = timestamp
        msg.roll = data['controller.roll']
        msg.pitch = data['controller.pitch']
        msg.yaw = data['controller.yaw']
        msg.actuator_thrust = data['controller.actuatorThrust']
        self._controller_rpyt_pub.publish(msg)

    def _state_estimate_cb(self, timestamp, data, logconfig):
        """ Callback from CrazyflieLibPython, publishes StateEstimate messages """
        msg = StateEstimate()
        msg.stamp.stamp = self.get_clock().now().to_msg()
        msg.cfstamp = timestamp
        msg.x = data['stateEstimate.x']
        msg.y = data['stateEstimate.y']
        msg.z = data['stateEstimate.z']
        # msg.vx = data['stateEstimate.vx']
        # msg.vy = data['stateEstimate.vy']
        # msg.vz = data['stateEstimate.vz']
        # msg.ax = data['stateEstimate.ax']
        # msg.ay = data['stateEstimate.ay']
        # msg.az = data['stateEstimate.az']
        # msg.yaw = data['stateEstimate.yaw']
        # msg.pitch = data['stateEstimate.pitch']
        # msg.roll = data['stateEstimate.roll']
        # msg.qx = data['stateEstimate.qx']
        # msg.qy = data['stateEstimate.qy']
        # msg.qz = data['stateEstimate.qz']
        # msg.qw = data['stateEstimate.qw']
        self._state_estimate_pub.publish(msg)

    def _kalman_position_cb(self, timestamp, data, logconfig):
        """ Callback from CrazyflieLibPython, publishes KalmanPositionEst messages """
        msg = KalmanPositionEst()
        msg.stamp.stamp = self.get_clock().now().to_msg()
        msg.cfstamp = timestamp
        msg.state_x = data['kalman.stateX']
        msg.state_y = data['kalman.stateY']
        msg.state_z = data['kalman.stateZ']
        self._kalman_position_pub.publish(msg)

    def _motor_power_cb(self, timestamp, data, logconfig):
        """ Callback from CrazyflieLibPython, publishes MotorPower messages """
        msg = MotorPower()
        msg.stamp.stamp = self.get_clock().now().to_msg()
        msg.cfstamp = timestamp
        msg.m4 = data['motor.m4']
        msg.m1 = data['motor.m1']
        msg.m2 = data['motor.m2']
        msg.m3 = data['motor.m3']
        self._motor_power_pub.publish(msg)

    def _pos_ctl_cb(self, timestamp, data, logconfig):
        """ Callback from CrazyflieLibPython, publishes posCtl messages """
        msg = PosCtl()
        msg.stamp.stamp = self.get_clock().now().to_msg()
        msg.cfstamp = timestamp
        msg.target_vx = data['posCtl.targetVX']
        msg.target_vy = data['posCtl.targetVY']
        msg.target_vy = data['posCtl.targetVZ']
        msg.target_x = data['posCtl.targetX']
        msg.target_y = data['posCtl.targetY']
        msg.target_z = data['posCtl.targetZ']
        self._posCtl_pub.publish(msg)

    def _stabilizer_cb(self, timestamp, data, logconfig):
        """ Callback from CrazyflieLibPython, publishes Stabilizer messages """
        msg = Stabilizer()
        msg.stamp.stamp = self.get_clock().now().to_msg()
        msg.cfstamp = timestamp
        msg.roll = data['stabilizer.roll']
        msg.pitch = data['stabilizer.pitch']
        msg.yaw = data['stabilizer.yaw']
        msg.thrust = data['stabilizer.thrust']
        self._stabilizer_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    log_node = CrazyflieLog()
    rclpy.spin(log_node)
    log_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()