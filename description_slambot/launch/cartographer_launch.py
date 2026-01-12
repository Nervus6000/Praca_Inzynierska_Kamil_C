import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('description_slambot')
    

    cartographer_config_dir = os.path.join(pkg_share, 'config')
    configuration_basename = 'cartographer_config.lua'

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{"use_sim_time": True}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', configuration_basename,
        ],
        remappings=[
            ('odom', '/slambot_controller/odom'), 
            ('scan', '/scan'),                    
        ]
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[
            {"use_sim_time": True},
            {'resolution': 0.05} 
        ]
    )

    return LaunchDescription([
        cartographer_node,
        occupancy_grid_node
    ])