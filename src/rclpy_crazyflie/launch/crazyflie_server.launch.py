import rclpy
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import json

def server_launch():
    with open('install/rclpy_crazyflie/share/rclpy_crazyflie/data/uris.json') as f:
        data = json.load(f)
        uris = [agent["uri"] for agent in data["info"]]
        names = [uri.split('/')[-1]  for uri in uris]
        poses = [agent["initial_pose"] for agent in data["info"]]
        f.close()

    with open('install/rclpy_crazyflie/share/rclpy_crazyflie/data/info.json') as f:
        data = json.load(f)
        log = data["logging"]
        global_coords = data["coordinates"]["global"]

    nodes_to_launch = []

    nodes_to_launch.append(
            launch_ros.actions.Node(
            package='rclpy_crazyflie', executable='server',
            name='server_node',
            parameters=[
                {
                'uris': uris,
                'log_rpy_rate': log['log_rpy_rate'],
                'log_rpyt': log['log_rpyt'],
                'log_se': log['log_se'],
                'log_kpe': log['log_kpe'],
                'log_pc': log['log_pc'],
                'log_mp': log['log_mp'],
                'log_sta': log['log_sta']
                }
            ])
    )

    if global_coords:
        for name, pose in zip(names, poses):
            nodes_to_launch.append(
                launch_ros.actions.Node(
                package='rclpy_crazyflie', executable='pose_tf',
                name='pose_tf_' + name,
                parameters=[
                    {
                        'name': name,
                        'transform_xyzyaw': [-var for var in pose]  
                    }
                ]
            ))

    return nodes_to_launch

def generate_launch_description():
    return launch.LaunchDescription(server_launch())