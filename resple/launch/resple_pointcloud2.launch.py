import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    # Generic PointCloud2 launch: works with any sensor that publishes
    # sensor_msgs/PointCloud2 (field layout introspected at runtime).
    # Point it at your sensor with:
    #   ros2 launch resple resple_pointcloud2.launch.py \
    #       lidar_frame:=os_sensor
    # and set the topics in config_pointcloud2.yaml (or pass a copied config).
    # Runtime health is published on `resple_diagnostics`
    # (estimate_msgs/Diagnostics, ~20 Hz) — see doc/PARAMETERS.md for the
    # full parameter & topic reference.
    start_delay_arg = DeclareLaunchArgument(
        'start_delay', default_value='0.0',
        description='Seconds to wait before launching RESPLE.')
    mapping_delay_arg = DeclareLaunchArgument(
        'mapping_delay', default_value=LaunchConfiguration('start_delay'),
        description='Seconds to wait before launching Mapping (defaults to start_delay).')
    lidar_frame_arg = DeclareLaunchArgument(
        'lidar_frame', default_value='lidar',
        description='The sensor frame_id stamped on the PointCloud2 messages '
                    '(used for the static base_link->lidar TF).')
    _start_delay = PythonExpression(['float(', LaunchConfiguration('start_delay'), ')'])
    _mapping_delay = PythonExpression(['float(', LaunchConfiguration('mapping_delay'), ')'])

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(
            get_package_share_directory('resple'), 'config', 'config_pointcloud2.yaml'),
        description='Parameter YAML for both nodes — pass a copied/adapted '
                    'config without editing the installed one.')
    config_yaml_fusion = LaunchConfiguration('config_file')
    viz_launch = os.path.join(
        get_package_share_directory('resple'),
        'launch',
        'resple_viz.launch.py')
    return launch.LaunchDescription([
        config_file_arg,
        start_delay_arg,
        mapping_delay_arg,
        lidar_frame_arg,
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
                '--frame-id', 'base_link',
                '--child-frame-id', LaunchConfiguration('lidar_frame')]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(viz_launch)),
        TimerAction(
            period=_start_delay,
            actions=[
                launch_ros.actions.Node(
                package='resple',
                executable='RESPLE',
                name='RESPLE',
                emulate_tty=True,
                output='log',
                parameters=[config_yaml_fusion],
                arguments=['--ros-args', '--log-level', 'info'])
            ]),
        TimerAction(
            period=_mapping_delay,
            actions=[
                launch_ros.actions.Node(
                package='resple',
                executable='Mapping',
                name='Mapping',
                emulate_tty=True,
                output='log',
                parameters=[config_yaml_fusion],
                arguments=['--ros-args', '--log-level', 'info'])
            ])
    ])
