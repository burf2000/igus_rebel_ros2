from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='igus_rebel_c_api',
            executable='arm_api_node',
            name='arm_api_node',
            output='screen'
        )
    ])
