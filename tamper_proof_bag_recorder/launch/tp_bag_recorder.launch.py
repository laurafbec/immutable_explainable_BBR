import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('tamper_proof_bag_recorder'),
        'config',
        'bc_rates.yaml'
        )

    return LaunchDescription([
        Node(
            package='tamper_proof_bag_recorder',
            executable='tp_bag_recorder_srv',
            output='screen',
            parameters = [config],
            arguments=['--ros-args', '--log-level', 'info']
        )
    ])

