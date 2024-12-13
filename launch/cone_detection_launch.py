from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    pkg_share = os.path.join(os.path.dirname(__file__), '..', 'share', 'cone_detection')
    pkg_shared = get_package_share_directory('cone_detection')

    return LaunchDescription([
        # Declare an argument for the RViz configuration file
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(pkg_shared, 'launch', 'viz-reliable.rviz'),
            description='Path to the RViz config file.'
        ),

        DeclareLaunchArgument(
            'use_system_default_qos',
            default_value = "true" ,
        ),

        # Launch the cone_detection_node
        Node(
            package='cone_detection',
            executable='cone_detection_node',
            name='cone_detection_node',
            output='screen',
            parameters=[
                {'use_sim_time': True}
            ]
        ),

        # Launch RViz with the specified configuration and fixed frame
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'fixed_frame': 'os_lidar'}]  # Set the default fixed frame
        )
    ])
