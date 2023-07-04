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
    lighthouse = data["coordinates"]["lighthouse"]

def generate_launch_description():
    client_nodes = [
        launch_ros.actions.Node(
            package='rclpy_cf_examples', executable='swarm_takeoff',
            name=uri.split('/')[-1] + '_swarm_takeoff',
            arguments=[uri.split('/')[-1]]
        )
    for uri in uris
    ]   

    return launch.LaunchDescription(client_nodes)
