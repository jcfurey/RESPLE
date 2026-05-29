import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
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
    _start_delay = PythonExpression(['float(', LaunchConfiguration('start_delay'), ')'])
    _mapping_delay = PythonExpression(['float(', LaunchConfiguration('mapping_delay'), ')'])

    config_yaml_fusion = os.path.join(
        get_package_share_directory('resple'),
        'config',
        'config_heap_testsite_hoenggerberg.yaml')
    viz_launch = os.path.join(
        get_package_share_directory('resple'),
        'launch',
        'resple_viz.launch.py')
    return launch.LaunchDescription([
        start_delay_arg,
        mapping_delay_arg,
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='livox_tf',
            arguments=[
                '--x', '0.010345', '--y', '0.022305', '--z', '-0.033211',
                '--qx', '0.000002', '--qy', '0.000670', '--qz', '0.006906', '--qw', '0.999976',
                '--frame-id', 'base_link', '--child-frame-id', 'livox_frame']),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='hesai_tf',
            arguments=[
                '--x', '-0.016286160655580', '--y', '-0.010352248240829', '--z', '0.128925315833111',
                '--qx', '-0.703743110426531', '--qy', '0.710453977622183', '--qz', '-0.000387167175118', '--qw', '0.000793920391926',
                '--frame-id', 'base_link', '--child-frame-id', 'hesai_lidar']),
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
                output='both',
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
                output='both',
                parameters=[config_yaml_fusion],
                arguments=['--ros-args', '--log-level', 'info'])
            ])                
  ])

