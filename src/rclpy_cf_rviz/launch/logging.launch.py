import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import json

with open('install/rclpy_crazyflie/share/rclpy_crazyflie/data/uris.json') as f:
    data = json.load(f)
    uris = [agent["uri"] for agent in data["info"]]
    f.close()

with open('install/rclpy_crazyflie/share/rclpy_crazyflie/data/info.json') as f:
    data = json.load(f)
    log = data["logging"]
    control = data["control"]

def generate_launch_description():
    logging_nodes = [
            launch_ros.actions.Node(
                package='rclpy_cf_rviz', executable='logging',
                name='logging_node',
                parameters=[
                    {
                    'name': uri.split('/')[-1],
                    'log_rpy_rate': log['log_rpy_rate'],
                    'log_rpyt' : log['log_rpyt'],
                    'log_se': log['log_se'],
                    'log_kpe': log['log_kpe'],
                    'log_pc': log['log_pc'],
                    'log_mp': log['log_mp'],
                    'log_sta': log['log_sta'],
                    'pose': control['pose'],
                    'waypoints': control['waypoints'],
                    'velocities': control['velocities']
                    }
                ])
        for uri in uris]
    return launch.LaunchDescription(logging_nodes)