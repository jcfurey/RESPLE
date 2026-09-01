"""Decode and replay Super SACS Run 1 through RESPLE, Mapping, and RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


DEFAULT_BAG = (
    '/run/media/jcfurey/Bulk-Storage1/SENSAI/super_sacs_jan20_run1/'
    'super_sacs_jan20_run1_combined_uncompressed.mcap'
)


def generate_launch_description():
    resple_share = get_package_share_directory('resple')
    ouster_share = get_package_share_directory('ouster_ros')

    bag = LaunchConfiguration('bag')
    rate = LaunchConfiguration('rate')
    duration = LaunchConfiguration('duration')
    discovery_delay = LaunchConfiguration('discovery_delay')
    startup_delay = LaunchConfiguration('startup_delay')
    read_ahead = LaunchConfiguration('read_ahead_queue_size')
    config_file = LaunchConfiguration('config_file')
    rviz_config = LaunchConfiguration('rviz_config')

    qos_overrides = os.path.join(
        resple_share, 'config', 'super_sacs_jan20_run1_qos.yaml')
    decoder_launch = os.path.join(
        ouster_share, 'launch', 'replay.launch.xml')

    decoder = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                XMLLaunchDescriptionSource(decoder_launch),
                launch_arguments={
                    # The sentinel disables replay.launch.xml's embedded bag
                    # player; the filtered/remapped player below owns /clock.
                    'bag_file': 'b',
                    'viz': 'false',
                    'ouster_ns': 'sensor/lidar/lidar0',
                    'proc_mask': 'PCL|IMU',
                    'point_type': 'original',
                    'organized': 'true',
                    'v_reduction': '1',
                    'sensor_frame': 'lidar0/sensor_frame',
                    'lidar_frame': 'lidar0/lidar_frame',
                    'imu_frame': 'lidar0/imu_frame',
                    'pub_static_tf': 'true',
                    'use_system_default_qos': 'true',
                    'timestamp_mode': 'TIME_FROM_ROS_TIME',
                }.items(),
            ),
        ],
    )

    player = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', bag,
            '--rate', rate,
            '--clock', '1000',
            '--delay', discovery_delay,
            '--playback-duration', duration,
            '--message-order', 'received',
            '--read-ahead-queue-size', read_ahead,
            '--qos-profile-overrides-path', qos_overrides,
            '--wait-for-all-acked', '10000',
            '--disable-keyboard-controls',
            '--progress-bar-update-rate', '0',
            '--topics',
            '/record/sensor/lidar/lidar0/metadata',
            '/record/sensor/lidar/lidar0/lidar_packets',
            '/record/sensor/lidar/lidar0/imu_packets',
            '--remap',
            '/record/sensor/lidar/lidar0/metadata:='
            '/sensor/lidar/lidar0/metadata',
            '/record/sensor/lidar/lidar0/lidar_packets:='
            '/sensor/lidar/lidar0/lidar_packets',
            '/record/sensor/lidar/lidar0/imu_packets:='
            '/sensor/lidar/lidar0/imu_packets',
        ],
        condition=IfCondition(LaunchConfiguration('play')),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'domain_id', default_value='44',
            description='Isolated ROS domain for this replay graph.'),
        DeclareLaunchArgument('bag', default_value=DEFAULT_BAG),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument(
            'duration', default_value='-1',
            description='Bag-time seconds to play; -1 runs through EOF.'),
        DeclareLaunchArgument(
            'startup_delay', default_value='3.0',
            description='Wall seconds before starting the rosbag process.'),
        DeclareLaunchArgument(
            'discovery_delay', default_value='2.0',
            description='Additional rosbag publisher-discovery delay.'),
        DeclareLaunchArgument(
            'read_ahead_queue_size', default_value='2048'),
        DeclareLaunchArgument('play', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('mapping', default_value='true'),
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(
                resple_share, 'config',
                'config_super_sacs_jan20_run1.yaml')),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(
                resple_share, 'config',
                'super_sacs_jan20_run1.rviz')),

        SetEnvironmentVariable(
            name='ROS_DOMAIN_ID', value=LaunchConfiguration('domain_id')),

        decoder,

        # Standalone replay body: align the estimator's base_link with the
        # Ouster reference frame. The decoder owns both calibrated child edges.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='resple_super_sacs_body_tf',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'lidar0/sensor_frame',
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='resple',
            executable='RESPLE',
            name='RESPLE',
            parameters=[config_file, {'use_sim_time': True}],
            arguments=['--ros-args', '--log-level', 'info'],
            output='screen',
        ),
        Node(
            package='resple',
            executable='Mapping',
            name='Mapping',
            parameters=[config_file, {'use_sim_time': True}],
            arguments=['--ros-args', '--log-level', 'info'],
            condition=IfCondition(LaunchConfiguration('mapping')),
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='resple_super_sacs_rviz',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen',
        ),

        TimerAction(
            period=PythonExpression(['float(', startup_delay, ')']),
            actions=[player],
        ),
    ])
