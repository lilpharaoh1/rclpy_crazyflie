import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import json

URIS = [
    'radio://0/80/2M/E7E7E7E7E7'
    ]
try:
    with open('install/rclpy_crazyflie/share/rclpy_crazyflie/data/uris.json') as f:
        data = json.load(f)
        uris = [agent["uri"] for agent in data["info"]]
        f.close()
except: 
    uris = URIS

def generate_launch_description():

    client_nodes = [
        launch_ros.actions.Node(
            package='rclpy_cf_examples', executable='position_control',
            name='pc_cf_node',
            parameters=[
                {
                'name': uri.split('/')[-1],
                }
            ])
    for uri in uris
    ]   

    return launch.LaunchDescription(client_nodes)