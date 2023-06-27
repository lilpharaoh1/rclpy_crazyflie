import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def launch_setup():
    server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("rclpy_crazyflie"), "/launch", "/crazyflie_server.launch.py"]
        )
    )

    client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("rclpy_crazyflie"), "/launch", "/crazyflie_client.launch.py"]
        )
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("rclpy_cf_rviz"), "/launch", "/cf_rviz.launch.py"]
        )
    )

    nodes_to_start = [
        server_launch,
        client_launch,
        rviz_launch
    ]

    return nodes_to_start

def generate_launch_description():
    return launch.LaunchDescription(launch_setup())

