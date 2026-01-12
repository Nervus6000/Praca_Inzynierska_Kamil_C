import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():


    pkg_share = get_package_share_directory('description_slambot') 

    # Ścieżka do pliku konfiguracyjnego
    slam_params_file = os.path.join(
        pkg_share, 
        'config', 
        'description_slambot/config/slam_toolbox_config.yaml' # Nazwa pliku z kroku 1.
    )

    slam = Node(
    package="slam_toolbox",
    executable="sync_slam_toolbox_node",
    name="slam_toolbox",
    parameters=[slam_params_file,                   # można po prostu podać ścieżkę
                {'use_sim_time': True} ]
    )

    return LaunchDescription([
        slam
    ])