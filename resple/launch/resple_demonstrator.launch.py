import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(
            get_package_share_directory('resple'), 'config', 'config_demonstrator.yaml'),
        description='Parameter YAML for both nodes — pass a copied/adapted '
                    'config without editing the installed one.')
    config_yaml_fusion = LaunchConfiguration('config_file')
    viz_launch = os.path.join(
        get_package_share_directory('resple'),
        'launch',
        'resple_viz.launch.py')

    # Delay (seconds) before spawning RESPLE and Mapping. 0.0 = launch
    # immediately (original behavior). Useful when other nodes must come up
    # first — e.g. wait for /clock under sim time, or wait for a sensor
    # driver / TF static publisher to be ready.
    start_delay_arg = DeclareLaunchArgument(
        'start_delay',
        default_value='0.0',
        description='Seconds to wait before launching RESPLE and Mapping nodes.')
    # Optional independent delay for Mapping. Defaults to start_delay so a
    # single arg is enough for the common case; override only if you need
    # Mapping to come up after RESPLE has had a moment to publish.
    mapping_delay_arg = DeclareLaunchArgument(
        'mapping_delay',
        default_value=LaunchConfiguration('start_delay'),
        description='Seconds to wait before launching the Mapping node.')
    # The Mapping node is visualization-only: it consumes RESPLE's est_window
    # and rebuilds a global cloud for rviz/Foxglove. The odometry path
    # (/odom, /current_scan, odom->base_link TF) lives entirely in the RESPLE
    # node and is bit-identical without it — see doc/MAPPING_NODE.md.
    use_mapping_arg = DeclareLaunchArgument(
        'use_mapping', default_value='false',
        description='Start the Mapping node (global map visualization + '
                    'map->odom TF). Not needed for odometry.')

    start_delay = LaunchConfiguration('start_delay')
    mapping_delay = LaunchConfiguration('mapping_delay')

    resple_node = launch_ros.actions.Node(
        package='resple',
        executable='RESPLE',
        name='RESPLE',
        emulate_tty=True,
        output='log',
        parameters=[config_yaml_fusion, {'tf_extrinsics': False}],
        arguments=['--ros-args', '--log-level', 'info'])

    mapping_node = launch_ros.actions.Node(
        package='resple',
        executable='Mapping',
        name='Mapping',
        emulate_tty=True,
        output='log',
        parameters=[config_yaml_fusion, {'tf_extrinsics': False}],
        arguments=['--ros-args', '--log-level', 'info'])

    # Wrap each in a TimerAction. period accepts a Substitution that
    # resolves to a numeric string at launch time, so users can override
    # via: ros2 launch resple resple_demonstrator.launch.py start_delay:=3.0
    delayed_resple = TimerAction(
        period=PythonExpression(['float(', start_delay, ')']),
        actions=[resple_node])
    delayed_mapping = TimerAction(
        period=PythonExpression(['float(', mapping_delay, ')']),
        condition=IfCondition(LaunchConfiguration('use_mapping')),
        actions=[mapping_node])

    return launch.LaunchDescription([
        config_file_arg,
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
        delayed_resple,
        delayed_mapping,
    ])
