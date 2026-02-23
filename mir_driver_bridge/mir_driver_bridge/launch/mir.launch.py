import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- 1. Launch Configurations (Arguments) ---
    mir_type = LaunchConfiguration('mir_type')
    tf_prefix = LaunchConfiguration('tf_prefix')
    mir_hostname = LaunchConfiguration('mir_hostname')
    disable_map = LaunchConfiguration('disable_map')

    # --- 2. Declare Arguments ---
    declare_mir_type_cmd = DeclareLaunchArgument(
        'mir_type', default_value='mir_100', description="The MiR variant. Can be 'mir_100' or 'mir_250'")
    
    declare_tf_prefix_cmd = DeclareLaunchArgument(
        'tf_prefix', default_value='', description="TF prefix to use for all of MiR's TF frames")
    
    declare_mir_hostname_cmd = DeclareLaunchArgument(
        'mir_hostname', default_value='192.168.12.20', description="IP address of the MiR robot")
    
    declare_disable_map_cmd = DeclareLaunchArgument(
        'disable_map', default_value='false', description="Disable the map topic and map -> odom TF")

    # --- 3. Process URDF/Xacro ---
    # Assuming your xacro file is inside 'mir_description/urdf/mir.urdf.xacro'
    mir_description_dir = get_package_share_directory('mir_description')
    xacro_file = os.path.join(mir_description_dir, 'urdf', 'mir.urdf.xacro')
    
    robot_description_content = Command(['xacro ', xacro_file, ' mir_type:=', mir_type])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # --- 4. Define Nodes ---

    # Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
        remappings=[
            ('/tf', 'tf_rss'),
            ('/tf_static', 'tf_static_rss')
        ]
    )

    # TF Remove State Publisher Frames
    tf_remove_state_publisher_frames_node = Node(
        package='mir_driver',
        executable='tf_remove_child_frames.py',
        output='screen',
        remappings=[
            ('tf_in', 'tf_rss'),
            ('tf_out', '/tf'),
            ('tf_static_in', 'tf_static_rss'),
            ('tf_static_out', '/tf_static')
        ],
        parameters=[{
            'remove_frames': [
                'base_link',
                'front_laser_link',
                'back_laser_link',
                'camera_top_link',
                'camera_top_depth_optical_frame',
                'camera_floor_link',
                'camera_floor_depth_optical_frame',
                'imu_link'
            ]
        }]
    )

    # MiR Bridge (When disable_map is TRUE)
    mir_bridge_no_map_node = Node(
        package='mir_driver',
        executable='mir_bridge.py',
        output='screen',
        condition=IfCondition(disable_map),
        parameters=[{
            'mir_ip': mir_hostname,
            'tf_prefix': tf_prefix
        }],
        remappings=[
            ('map', 'map_mir'),
            ('map_metadata', 'map_metadata_mir'),
            ('rosout', '/rosout'),
            ('rosout_agg', '/rosout_agg'),
            ('tf', 'tf_mir')
        ]
    )

    # TF Remove MiR Map Frame (When disable_map is TRUE)
    tf_remove_mir_map_frame_node = Node(
        package='mir_driver',
        executable='tf_remove_child_frames.py',
        output='screen',
        condition=IfCondition(disable_map),
        remappings=[
            ('tf_in', 'tf_mir'),
            ('tf_out', '/tf')
        ],
        parameters=[{
            'remove_frames': ['odom']
        }]
    )

    # MiR Bridge (When disable_map is FALSE)
    mir_bridge_with_map_node = Node(
        package='mir_driver',
        executable='mir_bridge.py',
        output='screen',
        condition=UnlessCondition(disable_map),
        parameters=[{
            'mir_ip': mir_hostname,
            'tf_prefix': tf_prefix
        }],
        remappings=[
            ('map', '/map'),
            ('map_metadata', '/map_metadata'),
            ('rosout', '/rosout'),
            ('rosout_agg', '/rosout_agg'),
            ('tf', '/tf')
        ]
    )

    # Laser Filters
    b_rep117_laser_filter_node = Node(
        package='mir_driver',
        executable='rep117_filter.py',
        output='screen',
        remappings=[
            ('scan', 'b_scan'),
            ('scan_filtered', 'b_scan_rep117')
        ]
    )

    # Front Laser Filter
    f_rep117_laser_filter_node = Node(
        package='mir_driver',
        executable='rep117_filter.py',
        output='screen',
        remappings=[
            ('scan', 'f_scan'),
            ('scan_filtered', 'f_scan_rep117')
        ]
    )

    # Fake Joint Publisher
    fake_mir_joint_publisher_node = Node(
        package='mir_driver',
        executable='fake_mir_joint_publisher.py',
        output='screen'
    )

    # --- 5. Build and Return the Launch Description ---
    return LaunchDescription([
        declare_mir_type_cmd,
        declare_tf_prefix_cmd,
        declare_mir_hostname_cmd,
        declare_disable_map_cmd,
        
        rsp_node,
        tf_remove_state_publisher_frames_node,
        
        mir_bridge_no_map_node,
        tf_remove_mir_map_frame_node,
        mir_bridge_with_map_node,
        
        b_rep117_laser_filter_node,
        f_rep117_laser_filter_node,
        fake_mir_joint_publisher_node
    ])