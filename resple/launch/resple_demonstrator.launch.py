import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():    
    config_yaml_fusion = os.path.join(
        get_package_share_directory('resple'),
        'config',
        'config_demonstrator.yaml')
    config_rviz = os.path.join(
        get_package_share_directory('resple'),
        'config',
        'config.rviz')        
    return launch.LaunchDescription([
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
        launch_ros.actions.Node(
            package='resple',
            executable='RESPLE',
            name='RESPLE',
            emulate_tty=True,
            output='log',
            parameters=[config_yaml_fusion],
            arguments=['--ros-args', '--log-level', 'warn']),                 
        launch_ros.actions.Node(
            package='resple',
            executable='Mapping',
            name='Mapping',
            emulate_tty=True,
            output='log',
            parameters=[config_yaml_fusion],
            arguments=['--ros-args', '--log-level', 'warn'])                
  ])

