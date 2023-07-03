import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import json

with open('install/rclpy_crazyflie/share/rclpy_crazyflie/data/uris.json') as f:
    data = json.load(f)
    uris = [agent["uri"] for agent in data["info"]]
    poses = [agent["initial_pose"] for agent in data["info"]]
    f.close()

with open('install/rclpy_crazyflie/share/rclpy_crazyflie/data/info.json') as f:
    data = json.load(f)
    log = data["logging"]
    global_coords = data["coordinates"]["global"]

def launch_server():
    nodes_to_launch = []

    nodes_to_launch.append(launch_ros.actions.Node(
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
        for (uri, pose) in zip(uris, poses):
            pose_tf_node = \
                launch_ros.actions.Node(
                package='rclpy_crazyflie', executable='pose_transform',
                name='pose_tf_' + uri.split('/')[-1],
                arguments=[uri.split('/')[-1], str(pose[0]), str(pose[1]), str(pose[2]), str(pose[3])]
                )

            nodes_to_launch.append(pose_tf_node)

    return nodes_to_launch

def generate_launch_description():
    return launch.LaunchDescription(launch_server())