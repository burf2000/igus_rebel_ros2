from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
          package='igus_rebel_pymoveit_api',
          executable='api',
          name='rebel_api', output='screen'
        )
    ])
