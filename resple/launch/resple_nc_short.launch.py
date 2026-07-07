import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():    
    # Optional delay (seconds) before launching RESPLE / Mapping. Lets you
    # wait for /clock under sim time, sensor drivers, or TF publishers to
    # come up before the estimator starts consuming data.
    start_delay_arg = DeclareLaunchArgument(
        'start_delay', default_value='0.0',
        description='Seconds to wait before launching RESPLE.')
    mapping_delay_arg = DeclareLaunchArgument(
        'mapping_delay', default_value=LaunchConfiguration('start_delay'),
        description='Seconds to wait before launching Mapping (defaults to start_delay).')
    # The Mapping node is visualization-only: it consumes RESPLE's est_window
    # and rebuilds a global cloud for rviz/Foxglove. The odometry path
    # (/odom, /current_scan, odom->base_link TF) lives entirely in the RESPLE
    # node and is bit-identical without it — see doc/MAPPING_NODE.md.
    use_mapping_arg = DeclareLaunchArgument(
        'use_mapping', default_value='false',
        description='Start the Mapping node (global map visualization + '
                    'map->odom TF). Not needed for odometry.')
    _start_delay = PythonExpression(['float(', LaunchConfiguration('start_delay'), ')'])
    _mapping_delay = PythonExpression(['float(', LaunchConfiguration('mapping_delay'), ')'])

    config_yaml_fusion = os.path.join(
        get_package_share_directory('resple'),
        'config',
        'config_nc_short.yaml')
    viz_launch = os.path.join(
        get_package_share_directory('resple'),
        'launch',
        'resple_viz.launch.py')
    return launch.LaunchDescription([
        start_delay_arg,
        mapping_delay_arg,
        use_mapping_arg,
        # Make the silent default obvious: without Mapping there is no
        # /global_map and no map->odom TF (doc/MAPPING_NODE.md).
        LogInfo(
            condition=UnlessCondition(LaunchConfiguration('use_mapping')),
            msg='Mapping node disabled (use_mapping:=false): no /global_map or '
                'map->odom TF will be published. Pass use_mapping:=true to enable.'),
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
            condition=IfCondition(LaunchConfiguration('use_mapping')),
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

