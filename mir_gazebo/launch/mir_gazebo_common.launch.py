import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Define package paths
    mir_description_dir = FindPackageShare('mir_description')
    mir_gazebo_dir = FindPackageShare('mir_gazebo')

    # 2. Process the Xacro file into a raw URDF string
    xacro_file = PathJoinSubstitution([mir_description_dir, 'urdf', 'mir.urdf.xacro'])
    robot_description_content = Command(
        [PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', xacro_file]
    )
    robot_description = {'robot_description': robot_description_content}

    # 3. Node: Robot State Publisher (Calculates TF tree from URDF)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # 4. Node: Spawn the robot into Gazebo
    # Gazebo reads the /robot_description topic and builds the 3D model
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'mir_100',
            '-z', '0.1' # Drop the robot slightly above the ground
        ],
        output='screen'
    )

    # 5. Node: The ROS-Gazebo Bridge (Using the YAML we wrote)
    bridge_config = PathJoinSubstitution([mir_gazebo_dir, 'config', 'gz_bridge.yaml'])
    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': True
        }],
        output='screen'
    )

    #6. Merge our lasers (/f_scan and /b_scan)
    #path to the merger
    laser_merger_params = os.path.join(
        get_package_share_directory('mir_gazebo'),
        'config',
        'laser_merger_params.yaml'
    )

    laser_merger_node = Node(
        package = 'dual_laser_merger',
        executable = 'dual_laser_merger_node',
        name = 'dual_laser_merger_node',
        output = 'screen',
        parameters=[laser_merger_params, {'use_sim_time': True}],
        remappings = [('/merged', '/scan')]
    )

    #7. Open RViz2 with the rviz config as well
    rviz_config_file = os.path.join(
        get_package_share_directory('mir_gazebo'),
        'rviz',
        'rviz_config.rviz'
    )

    rviz_node = Node(
        package = 'rviz2',
        executable = 'rviz2',
        name = 'rviz2',
        output = 'screen',
        arguments = ['-d', rviz_config_file],
        parameters = [{'use_sim_time': True}]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        spawn_entity_node,
        ros_gz_bridge_node,
        laser_merger_node,
        rviz_node
    ])