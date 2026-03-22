import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'my_robot_controller' 
    pkg_share = get_package_share_directory(pkg_name)
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'nav_config.rviz')

    # --- 1. Robot State Publisher (URDF) ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', xacro_file])}]
    )
    use_sim_time = {'use_sim_time': True}
    # --- 2. Gazebo Simulation ---
    # Running with verbose to catch hardware/driver errors early

# ... inside generate_launch_description ...

    pkg_share = get_package_share_directory('my_robot_controller')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    # Define the path to your world file
    world_path = os.path.join(pkg_share, 'worlds', 'ground.world')

    # Include Gazebo Launch with the 'world' argument
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_path,
            'verbose': 'true'
        }.items()
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'my_robot', '-topic', 'robot_description', '-timeout', '300'],
        output='screen'
    )

    # --- 3. Your Custom C++ Nodes ---
    
    # Node A: The Obstacle Detector (PCL Heavy)
    obstacle_detector = Node(
    package='my_robot_controller',
    executable='obstacle_detector', # Check if this matches your CMakeLists.txt
    name='obstacle_detector_node',
    output='screen',
    parameters=[use_sim_time],
    # This is the "No Relay" solution:
    remappings=[
        ('/d430/depth/points', '/depth/colour/points')
    ]
)

    # Node B: The GPS Navigator (State Machine)
    gps_navigator = Node(
        package=pkg_name,
        executable='gps_navigator',
        name='tangent_bug_lite',
        output='screen',
        parameters=[use_sim_time]
    )

    # --- 4. RViz ---
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_entity,
        obstacle_detector,
        gps_navigator,
        rviz
    ])