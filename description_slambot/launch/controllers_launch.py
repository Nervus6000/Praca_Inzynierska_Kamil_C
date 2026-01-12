from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():



    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        parameters=[{"use_sim_time": True}],
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    imu = Node(
        package='controller_manager',
        executable='spawner',
        parameters=[{"use_sim_time": True}],
        arguments=[
            "imu_sensor_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    simple_diff_drive_controller_py = Node(
        package="description_slambot",
        parameters=[{"use_sim_time": True}],
        executable="simple_diff_drive_controller.py",
    )

    publish_path_node_py = Node(
        package="description_slambot",
        parameters=[{"use_sim_time": True}],
        executable="publish_path_node.py",
    )

    simple_vel_controller = Node(                                 
        package='controller_manager',
        executable='spawner',
        parameters=[{"use_sim_time": True}],
        arguments=[
            "simple_velocity_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )



    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{"use_sim_time": True}],
        arguments=[
            "/model/SlamBot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        remappings=[
            ('/model/SlamBot/odometry', '/model/SlamBot/odometry_perfect')
        ],
    )


    


    return LaunchDescription([
        joint_state_broadcaster_spawner,
        imu,
        simple_diff_drive_controller_py, simple_vel_controller,  
        publish_path_node_py,
        bridge_node
    ])