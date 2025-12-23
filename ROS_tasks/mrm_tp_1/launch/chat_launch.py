from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mrm_tp_1',
            executable='talker_node',
            name='talker'
        ),
        Node(
            package='mrm_tp_1',
            executable='listener_node',
            name='listener'
        )
    ])