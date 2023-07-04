import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import json

URIS = [
    'radio://0/80/2M/E7E7E7E7E7'
    ]

with open('install/rclpy_crazyflie/share/rclpy_crazyflie/data/uris.json') as f:
    data = json.load(f)
    uris = [agent["uri"] for agent in data["info"]]
    f.close()


with open('install/rclpy_crazyflie/share/rclpy_crazyflie/data/info.json') as f:
    data = json.load(f)
    global_coords = data["coordinates"]["global"]

def generate_launch_description():

    client_nodes = [
        launch_ros.actions.Node(
            package='rclpy_crazyflie', executable='client',
            name=uri.split('/')[-1] + '_client_node',
            parameters=[
                {
                'uri': uri,
                'global_coords': global_coords
                }
            ])
    for uri in uris
    ]   

    return launch.LaunchDescription(client_nodes)