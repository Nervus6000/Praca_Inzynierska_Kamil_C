import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path

from pathlib import Path

from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression 
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    slambot_description_path = get_package_share_path("description_slambot")
    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"
     
    # model URDF
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            slambot_description_path,     
            "urdf", "my_slambot.urdf.xacro"
        ),
        description="Absolute path to urdf file"
    )

    robot_description = ParameterValue(Command(["xacro ", 
                                                LaunchConfiguration("model"), 
                                                " is_ignition:=",
                                                is_ignition]),
                                                value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description,
                    'use_sim_time': True}]
    )





#                GAZEBO

    gazebo_resource_path = SetEnvironmentVariable(
    name='GZ_SIM_RESOURCE_PATH',
    value=os.path.join(get_package_share_directory('description_slambot'), 'models') 
)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
                launch_arguments={
                    'gz_args': ['-r -v 4 ' + os.path.join(
                    get_package_share_directory('description_slambot'),
                    'worlds_gazebo',
                    'small_house.world'            #    tutaj podajemy model świata
                    )],
                    'on_exit_shutdown': 'true'
                }.items()
                )
    
    gazebo_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                    '-name', 'SlamBot'],
        output='screen')


    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        gazebo_resource_path,
        gazebo,
        gazebo_spawn_entity
    ]) 
