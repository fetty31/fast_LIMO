# reloca.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Whether to run an RViz instance'
    )

    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='campus_nord.pcd',
        description='Name of the map file inside fast_limo/maps/'
    )

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='false',
        description='Relocation mode flag'
    )

    rviz_cfg = PathJoinSubstitution([
        FindPackageShare('fast_limo'),
        'config',
        'rviz',
        'limo.rviz'
    ])

    map_path = PathJoinSubstitution([
        FindPackageShare('fast_limo'),
        'maps',
        LaunchConfiguration('map_name')
    ])

    reloca_node = Node(
        package='fast_limo',
        executable='fast_limo_reloca_exec',
        name='fast_limo_reloca',
        output='screen',
        parameters=[{
            'mode': LaunchConfiguration('mode'),
            'map_path': map_path,
            'distance_threshold': 10.0,
            'inliers_threshold': 5,
            'score': 10000.0,
            'downsample_leaf': 3.0,
            'initialpose_sync_tolerance': 0.25,
            'odom_history_duration': 30.0,
            'local.yaw_step_deg': 15.0,
            'local.crop_margin': 6.0,
            'local.coarse_voxel': 0.8,
            'local.fine_voxel': 0.3,
            'local.coarse_max_correspondence': 3.0,
            'local.fine_max_correspondence': 1.0,
            'local.refine_candidates': 4,
            'local.min_inliers': 300,
            'local.min_inlier_ratio': 0.35,
            'local.max_rmse': 0.5,
            'local.max_position_correction': 5.0,
            'local.max_z_correction': 1.5,
            'local.max_roll_pitch_deg': 8.0,
            'local.min_solution_separation': 0.10,
            'frames.map': "ona2/map",
            'frames.world': "ona2/odom"
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_limo',
        output='screen',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        rviz_arg,
        map_name_arg,
        mode_arg,
        reloca_node,
        rviz_node,
    ])
