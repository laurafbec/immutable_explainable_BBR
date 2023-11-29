import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('bag_reader'),
        'config',
        'params.yaml'
        )
    
    return LaunchDescription([
        Node(
            package='bag_reader',
            executable='bag_reader_node',
            output='screen',
            parameters = [config]
        )
    ])