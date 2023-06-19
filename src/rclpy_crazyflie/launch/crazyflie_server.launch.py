import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='crazyflie_server', executable='server',
            name='server_node',
            parameters=[
                {
                'uris': ['radio://0/80/2M/E7E7E7E7E7'],
                }
            ])
    ])