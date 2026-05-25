from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('oculus_driver')
    params = os.path.join(pkg, 'config', 'slam_toolbox_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('sonar_ip',  default_value='192.168.2.6'),
        DeclareLaunchArgument('range_m',   default_value='1.0'),
        DeclareLaunchArgument('gain',      default_value='50.0'),

        # 1) Sonar driver node
        Node(
            package='oculus_driver',
            executable='sonar_node',
            name='oculus_sonar',
            parameters=[{
                'sonar_ip':  LaunchConfiguration('sonar_ip'),
                'range_m':   LaunchConfiguration('range_m'),
                'gain':      LaunchConfiguration('gain'),
                'salinity':  0.0,    # 0=fresh water
                'sonar_mode': 1,
            }]
        ),

        # 2) Static TF
        Node(
            package='oculus_driver',
            executable='tf_static',
            name='static_tf',
        ),

        # 3) SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[params],
            output='screen',
        ),

        # 4) RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg, 'config', 'sonar_slam.rviz')],
        ),
    ])
