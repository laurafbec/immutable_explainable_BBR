
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tamper_proof_bag_recorder',
            executable='proof_checker_srv',
            output='screen'),
    ])

