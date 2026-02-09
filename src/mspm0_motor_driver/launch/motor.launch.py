from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('mspm0_motor_driver')
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='mspm0_motor_driver',
            executable='mspm0_motor_driver',
            name='mspm0_motor_driver',
            output='screen',
            parameters=[params_file]
        )
    ])
