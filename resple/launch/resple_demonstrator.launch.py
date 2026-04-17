import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    config_yaml_fusion = os.path.join(
        get_package_share_directory('resple'),
        'config',
        'config_demonstrator.yaml')
    config_rviz = os.path.join(
        get_package_share_directory('resple'),
        'config',
        'config.rviz')

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

    start_delay = LaunchConfiguration('start_delay')
    mapping_delay = LaunchConfiguration('mapping_delay')

    resple_node = launch_ros.actions.Node(
        package='resple',
        executable='RESPLE',
        name='RESPLE',
        emulate_tty=True,
        output='log',
        parameters=[config_yaml_fusion],
        arguments=['--ros-args', '--log-level', 'info'])

    mapping_node = launch_ros.actions.Node(
        package='resple',
        executable='Mapping',
        name='Mapping',
        emulate_tty=True,
        output='log',
        parameters=[config_yaml_fusion],
        arguments=['--ros-args', '--log-level', 'info'])

    # Wrap each in a TimerAction. period accepts a Substitution that
    # resolves to a numeric string at launch time, so users can override
    # via: ros2 launch resple resple_demonstrator.launch.py start_delay:=3.0
    delayed_resple = TimerAction(
        period=PythonExpression(['float(', start_delay, ')']),
        actions=[resple_node])
    delayed_mapping = TimerAction(
        period=PythonExpression(['float(', mapping_delay, ')']),
        actions=[mapping_node])

    return launch.LaunchDescription([
        start_delay_arg,
        mapping_delay_arg,
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf',
            arguments=[
                '--x', '0.011', '--y', '0.02329', '--z', '-0.04412',
                '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
                '--frame-id', 'base_link', '--child-frame-id', 'livox_frame']),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', config_rviz, '--ros-args', '--log-level', 'WARN']),
        delayed_resple,
        delayed_mapping,
    ])
