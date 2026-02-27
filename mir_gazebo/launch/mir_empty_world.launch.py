import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Paths to required packages
    ros_gz_sim_dir = FindPackageShare('ros_gz_sim')
    mir_gazebo_dir = FindPackageShare('mir_gazebo')

    # Get the parent directory of the 'mir_description' share folder
    # This is usually ~/mir_ws/install/share
    mir_desc_share = get_package_share_directory('mir_description')
    workspace_share_dir = os.path.dirname(mir_desc_share)

    # 2. Tell Ignition Gazebo where to find the 'model://mir_description' meshes
    set_ign_resource_path = AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=workspace_share_dir
    )

    # Set the world path
    world_path = PathJoinSubstitution([
        mir_gazebo_dir,
        'worlds',
        'empty.world.sdf' 
    ])

    # 3. Launch Modern Gazebo (Fortress) with an empty world
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim_dir, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': ['-r ', world_path]}.items()
    )

    # 4. Include our common launch file to spawn the robot and bridge
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([mir_gazebo_dir, 'launch', 'mir_gazebo_common.launch.py'])
        )
    )

    return LaunchDescription([
        set_ign_resource_path,
        gazebo_server,
        spawn_robot
    ])