from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path
from launch_ros.actions import Node
import os




def generate_launch_description():
    pkg_share = get_package_share_directory('description_slambot')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_share, 'launch', 'gazebo_launch.py'])]
        )
    )

    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_share, 'launch', 'controllers_launch.py'])]
        )
    )

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_share, 'launch', 'joystick_teleop_launch.py'])]
        )
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_share, 'launch', 'slam_launch.py'])]
        )
    )


    cartographer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([pkg_share, 'launch', 'cartographer_launch.py'])]
        )
    )

    rviz2_node_slam_toolbox = Node(
        package="rviz2",
        executable="rviz2",
        name='rviz2',
        arguments=["-d", os.path.join(get_package_share_path("description_slambot"), "rviz", "slam_toolbox.rviz")],
        parameters=[{"use_sim_time": True}]
    )

    rviz2_node_cartographer = Node(
        package="rviz2",
        executable="rviz2",
        name='rviz2',
        arguments=["-d", os.path.join(get_package_share_path("description_slambot"), "rviz", "cartographer.rviz")],
        parameters=[{"use_sim_time": True}]
    )


    return LaunchDescription([
        gazebo, controllers, joystick,
        

        # przygotowana konfiguracja rviz:

        rviz2_node_slam_toolbox,                
        #rviz2_node_cartographer,


        # nieużywany algorytm zakomentować:

        slam_toolbox,
        #cartographer,
    ])