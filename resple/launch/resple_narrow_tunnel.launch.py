import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


# NARROW-BORE / confined-space template (tunnel or pipe width ~0.8-1.5 m).
# Pairs with config_narrow_tunnel.yaml — see that file's [NARROW] comments
# for the rationale behind every deviation from the production values.
#
# Extrinsics follow the production convention: the identity lidar_tf below is
# a PLACEHOLDER — replace it with your rig's real base_link->os_sensor pose
# (leave the config's q_lb/t_lb identity), or delete the TF node and run with
# tf_extrinsics: false + calibrated q_lb/t_lb in the config. Never both.
def generate_launch_description():
    start_delay_arg = DeclareLaunchArgument(
        'start_delay', default_value='0.0',
        description='Seconds to wait before launching RESPLE.')
    mapping_delay_arg = DeclareLaunchArgument(
        'mapping_delay', default_value=LaunchConfiguration('start_delay'),
        description='Seconds to wait before launching Mapping (defaults to start_delay).')
    use_mapping_arg = DeclareLaunchArgument(
        'use_mapping', default_value='false',
        description='Start the Mapping node (global map visualization + '
                    'map->odom TF). Not needed for odometry.')
    _start_delay = PythonExpression(['float(', LaunchConfiguration('start_delay'), ')'])
    _mapping_delay = PythonExpression(['float(', LaunchConfiguration('mapping_delay'), ')'])

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(
            get_package_share_directory('resple'), 'config', 'config_narrow_tunnel.yaml'),
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
        use_mapping_arg,
        LogInfo(
            condition=UnlessCondition(LaunchConfiguration('use_mapping')),
            msg='Mapping node disabled (use_mapping:=false): no /global_map or '
                'map->odom TF will be published. Pass use_mapping:=true to enable.'),
        # PLACEHOLDER identity extrinsic — replace with the rig's real
        # base_link->os_sensor pose (see file header).
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
                '--frame-id', 'base_link', '--child-frame-id', 'os_sensor']),
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
