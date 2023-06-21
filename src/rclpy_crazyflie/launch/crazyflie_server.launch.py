import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import json

with open('install/rclpy_crazyflie/share/rclpy_crazyflie/data/uris.json') as f:
    data = json.load(f)
    uris = [agent["uri"] for agent in data["info"]]
    f.close()

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='rclpy_crazyflie', executable='server',
            name='server_node',
            parameters=[
                {
                'uris': uris,
                'log_rpy_rate': True,
                'log_rpyt': True,
                'log_kpe': True,
                'log_pc': True,
                'log_mp': True,
                'log_sta': True
                }
            ])
    ])