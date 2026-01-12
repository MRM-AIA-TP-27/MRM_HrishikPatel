import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'group_robot' # Verify your package name matches here
    
    # Process the Xacro file
    xacro_path = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'robot.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_path).toxml()

    return LaunchDescription([
    # Start Gazebo
        ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'], output='screen'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_raw, 'use_sim_time': True}]
        ),

    # Spawn the Robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'simple_rover', '-topic', 'robot_description'],
            output='screen'
        ),
        Node(
            package='group_robot',
            executable='gps_navigator',
            output='screen'
        )
    ])