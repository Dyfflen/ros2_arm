# launch/visualizar_robot.launch.py

import os
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name_description = 'robotarm_description'
    package_name_gazebo = 'robotarm_gazebo'

    urdf_file = 'urdf/ros2_control/gazebo/robotarm.urdf.xacro'
    rviz_config_file = 'rviz/robotarm_view_description.rviz'
    
    pkg_share_description = FindPackageShare(package=package_name_description).find(package_name_description)
    pkg_share_gazebo = FindPackageShare(package=package_name_gazebo).find(package_name_gazebo)

    default_urdf_model_path = os.path.join(pkg_share_gazebo, urdf_file)
    default_rviz_config_path = os.path.join(pkg_share_description, rviz_config_file)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', default_urdf_model_path])}]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config_path],
    )

    ld = LaunchDescription()

    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(rviz_node)

    return ld