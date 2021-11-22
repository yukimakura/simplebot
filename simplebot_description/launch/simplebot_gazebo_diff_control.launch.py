import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, LaunchConfiguration ,PathJoinSubstitution
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    package_name = 'simplebot_description'


    simplebot_description_path = os.path.join(
        get_package_share_directory(package_name))
        
    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
            launch_arguments={
                'world': os.path.join(simplebot_description_path, 'worlds','cafe.world')
            }.items()
            )

    robot_desc = xacro.process_file(
        os.path.join(simplebot_description_path,
                              'urdf',
                              'simplebot_gazebo.urdf.xacro'))


    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc.toxml(), 
            'use_sim_time': True}])

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',"-entity", "simplebot"],
                        output='screen')

    rviz_config_file = os.path.join(
        get_package_share_directory(package_name),
        'rviz','viz.rviz')

    rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_file],
        )

    return LaunchDescription([
        rviz_node,
        gazebo,
        start_robot_state_publisher_cmd,
        spawn_entity
    ])