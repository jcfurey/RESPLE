import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_foxglove_arg = DeclareLaunchArgument(
        'use_foxglove', default_value='true',
        description='Start foxglove_bridge WebSocket server for Trillium/Foxglove clients.')
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='Start rviz2 with the default RESPLE config.')
    foxglove_port_arg = DeclareLaunchArgument(
        'foxglove_port', default_value='8765',
        description='Port for foxglove_bridge WebSocket server.')

    config_rviz = os.path.join(
        get_package_share_directory('resple'), 'config', 'config.rviz')

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{'port': LaunchConfiguration('foxglove_port')}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_foxglove')))

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', config_rviz, '--ros-args', '--log-level', 'WARN'],
        condition=IfCondition(LaunchConfiguration('use_rviz')))

    return LaunchDescription([
        use_foxglove_arg,
        use_rviz_arg,
        foxglove_port_arg,
        foxglove_bridge,
        rviz,
    ])
