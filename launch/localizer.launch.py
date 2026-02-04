from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    rviz_config = LaunchConfiguration('rviz')

    rviz_config_arg = DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description = 'Whether to run an rviz instance'
    )

    limo_node = Node(
        package='fast_limo',
        namespace='',
        executable='fast_limo_multi_exec',
        name='fast_limo',
        output='screen',
        parameters=[PathJoinSubstitution([
                FindPackageShare('fast_limo'),
                'config',
                'ona.yaml'
            ])]
    )

    rviz_conditioned = ExecuteProcess(
        condition=IfCondition(
            PythonExpression([
                rviz_config
            ])
        ),
        cmd=[[
            'ros2 run rviz2 rviz2 -d ',
             PathJoinSubstitution([
                FindPackageShare('fast_limo'),
                'config',
                'rviz',
                'limo.rviz'
            ])
        ]],
        shell=True
    )

    
    return LaunchDescription([
        rviz_config_arg,
        limo_node,
        rviz_conditioned
    ])