import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import json

with open('install/rclpy_crazyflie/share/rclpy_crazyflie/data/info.json') as f:
    data = json.load(f)
    rviz = data["rviz"]

def launch_setup():
    rviz_launch = launch_ros.actions.Node(
            package='rviz2', executable='rviz2',
            name='cf_rviz_node')

    logging_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("rclpy_cf_rviz"), "/launch", "/logging.launch.py"]
        )
    )

    nodes_to_start = []
    if rviz["anything"]:
        nodes_to_start.append(rviz_launch)
        if rviz["logging"]:
            nodes_to_start.append(logging_launch)

    return nodes_to_start

def generate_launch_description():
    return launch.LaunchDescription(launch_setup())