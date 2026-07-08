import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


# Launch for the 07052026_4_an dataset (Ouster OS-1-64 + IMU, LIO).
# Unlike resple_06042026.launch.py, this bag holds RAW record-mode packets
# (/ouster/{lidar,imu}_packets + /ouster/metadata), not already-decoded
# points/imu — so this launch also brings up ouster_ros's os_cloud/os_image
# composable nodes (src/ouster-ros submodule, replay.launch.xml) to decode
# them into /ouster/points + /ouster/imu before RESPLE ever sees them.
# Same TF handling as 06042026, EXCEPT the synthetic root frame is named
# "resple_base_link", not "base_link": this bag's own /tf_static already
# parents "base_link" under "base_link_correction" (the rig's own nav/camera
# TF tree), so RESPLE claiming "base_link" too would be a two-parent TF
# conflict (tf2 rejects one side as TF_OLD_DATA) — see config_07052026.yaml's
# header. Publish resple_base_link->os_sensor (identity) and let the bag's
# /tf_static supply os_sensor->os_lidar and os_sensor->os_imu (including the
# 180deg os_lidar<->os_imu yaw). ouster_ros's own pub_static_tf is turned OFF
# so it doesn't double-publish that same pair from the metadata (see
# resple_06042026.launch.py's warning about two
# static publishers on one TF pair latching nondeterministically).
# REPLAY MUST INCLUDE /tf_static — see config_07052026.yaml.
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

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(
            get_package_share_directory('resple'), 'config', 'config_07052026.yaml'),
        description='Parameter YAML for both nodes — pass a copied/adapted '
                    'config without editing the installed one.')
    config_yaml_fusion = LaunchConfiguration('config_file')
    viz_launch = os.path.join(
        get_package_share_directory('resple'),
        'launch',
        'resple_viz.launch.py')
    ouster_replay_launch = os.path.join(
        get_package_share_directory('ouster_ros'),
        'launch',
        'replay.launch.xml')
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
        # base_link -> os_sensor (identity). The bag's /tf_static completes the
        # chain to os_lidar / os_imu. Replay /tf_static for this to resolve.
        # If a URDF / robot_state_publisher already provides the sensor
        # mounting (integrated stacks), DELETE this node: two static
        # publishers on one TF pair latch nondeterministically (last writer
        # per listener wins), and an identity placeholder silently cancels
        # or fights the real URDF extrinsic.
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
                '--frame-id', 'resple_base_link', '--child-frame-id', 'os_sensor']),
        # Decode /ouster/{lidar,imu}_packets + /ouster/metadata (as recorded
        # in this bag) into /ouster/points + /ouster/imu. No bag_file arg —
        # we play the bag ourselves in a second terminal, same convention as
        # every other resple_*.launch.py. viz:=false — RESPLE has its own
        # rviz config via resple_viz.launch.py below; running ouster_ros's
        # rviz too would be redundant (and needs X11 we may not have).
        # Wrapped in a SCOPED group: replay.launch.xml does an unscoped
        # <set_parameter name="use_sim_time" value="true"/> at its top level,
        # which — since SetParameter applies forward to every node declared
        # afterward in the flattened launch tree, not just nodes inside that
        # XML file — would otherwise silently flip RESPLE/Mapping below into
        # sim-time mode. Every other resple_*.launch.py runs wall-clock;
        # keep this one consistent (confirmed via TF_OLD_DATA warnings on
        # base_link before this fix — RESPLE's tf2 buffer was comparing
        # sim-time-stamped broadcasts against its own wall-clock ones).
        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    XMLLaunchDescriptionSource(ouster_replay_launch),
                    launch_arguments={
                        'viz': 'false',
                        'pub_static_tf': 'false',
                        # No default upstream despite being conditionally
                        # used — pass empty so its embedded `ros2 bag play`
                        # stays disabled; we play the bag ourselves in a
                        # second terminal.
                        'bag_file': '',
                    }.items()),
            ]),
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
