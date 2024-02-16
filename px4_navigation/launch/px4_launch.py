from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_dir = get_package_share_directory('px4_navigation')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true'
    )

    px4_broadcaster_cmd = Node(
            package='px4_navigation',
            executable='px4_broadcaster',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )
    
    points_subscriber_cmd = Node(
            package='px4_navigation',
            executable='points_subscriber'
        )
    
    # point_to_laserscan_cmd = Node(
    #         package='pointcloud_to_laserscan',
    #         executable='pointcloud_to_laserscan'
    #     )
    
    slam_launch_file = os.path.join(package_dir, 'launch', 'slam_launch.py')
    slam_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file)
    )

    navigation_launch_file = os.path.join(package_dir, 'launch', 'navigation_launch.py')
    navigation_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_file)
    )

    ld = LaunchDescription()

    ld.add_action(use_sim_time_arg)

    ld.add_action(px4_broadcaster_cmd)
    ld.add_action(points_subscriber_cmd)
    # ld.add_action(point_to_laserscan_cmd)
    ld.add_action(slam_launch_cmd)
    ld.add_action(navigation_launch_cmd)

    return ld