#! /usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  package_name_description = 'robotarm_description'
  package_name_gazebo = 'robotarm_gazebo'

  default_robot_name = 'robotarm'
  gazebo_launch_file_path = 'launch'
  gazebo_models_path = 'models'
  ros_gz_bridge_config_file_path = 'config/ros_gz_bridge.yaml'
  rviz_config_file_path = 'rviz/robotarm_view_description.rviz'
  urdf_file_path = 'urdf/robotarm_gazebo.urdf.xacro' 
  world_file_path = 'worlds/empty.world'
  pkg_ros_gz_sim = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')  
  pkg_share_description = FindPackageShare(package=package_name_description).find(package_name_description)
  pkg_share_gazebo = FindPackageShare(package=package_name_gazebo).find(package_name_gazebo)

  default_ros_gz_bridge_config_file_path = os.path.join(pkg_share_gazebo, ros_gz_bridge_config_file_path)
  default_rviz_config_path = os.path.join(pkg_share_description, rviz_config_file_path)  
  default_urdf_model_path = os.path.join(pkg_share_gazebo, urdf_file_path)
  gazebo_launch_file_path = os.path.join(pkg_share_gazebo, gazebo_launch_file_path)   
  gazebo_models_path = os.path.join(pkg_share_gazebo, gazebo_models_path)
  world_path = os.path.join(pkg_share_gazebo, world_file_path)
  
  headless = LaunchConfiguration('headless')
  robot_name = LaunchConfiguration('robot_name')
  rviz_config_file = LaunchConfiguration('rviz_config_file')
  urdf_model = LaunchConfiguration('urdf_model')
  use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
  use_rviz = LaunchConfiguration('use_rviz')
  use_sim_time = LaunchConfiguration('use_sim_time')
  use_simulator = LaunchConfiguration('use_simulator')
  world = LaunchConfiguration('world')
  
  x = LaunchConfiguration('x')
  y = LaunchConfiguration('y')
  z = LaunchConfiguration('z')
  roll = LaunchConfiguration('roll')
  pitch = LaunchConfiguration('pitch')
  yaw = LaunchConfiguration('yaw')
  
  declare_robot_name_cmd = DeclareLaunchArgument(
    name='robot_name',
    default_value=default_robot_name,
    description='The name for the robot')

  declare_rviz_config_file_cmd = DeclareLaunchArgument(
    name='rviz_config_file',
    default_value=default_rviz_config_path,
    description='Full path to the RVIZ config file to use')

  declare_simulator_cmd = DeclareLaunchArgument(
    name='headless',
    default_value='False',
    description='Display the Gazebo GUI if False, otherwise run in headless mode')

  declare_urdf_model_path_cmd = DeclareLaunchArgument(
    name='urdf_model', 
    default_value=default_urdf_model_path, 
    description='Absolute path to robot urdf file')
    
  declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    name='use_robot_state_pub',
    default_value='True',
    description='Whether to start the robot state publisher')

  declare_use_rviz_cmd = DeclareLaunchArgument(
    name='use_rviz',
    default_value='True',
    description='Whether to start RVIZ')
    
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='true',
    description='Use simulation (Gazebo) clock if true')

  declare_use_simulator_cmd = DeclareLaunchArgument(
    name='use_simulator',
    default_value='True',
    description='Whether to start Gazebo')

  declare_world_cmd = DeclareLaunchArgument(
    name='world',
    default_value=world_path,
    description='Full path to the world model file to load')

  declare_x_cmd = DeclareLaunchArgument(
    name='x',
    default_value='0.0',
    description='x component of initial position, meters')

  declare_y_cmd = DeclareLaunchArgument(
    name='y',
    default_value='0.0',
    description='y component of initial position, meters')
    
  declare_z_cmd = DeclareLaunchArgument(
    name='z',
    default_value='0.05',
    description='z component of initial position, meters')
    
  declare_roll_cmd = DeclareLaunchArgument(
    name='roll',
    default_value='0.0',
    description='roll angle of initial orientation, radians')

  declare_pitch_cmd = DeclareLaunchArgument(
    name='pitch',
    default_value='0.0',
    description='pitch angle of initial orientation, radians')

  declare_yaw_cmd = DeclareLaunchArgument(
    name='yaw',
    default_value='0.0',
    description='yaw angle of initial orientation, radians')
  
  set_env_vars_resources = AppendEnvironmentVariable(
    'GZ_SIM_RESOURCE_PATH',
    gazebo_models_path)
  
  start_gazebo_server_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
    condition=IfCondition(use_simulator),
    launch_arguments={'gz_args': ['-r -s -v4 ', world], 'on_exit_shutdown': 'true'}.items())

  start_gazebo_client_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
    launch_arguments={'gz_args': '-g -v4 '}.items(),
    condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))
    
  robot_description_content = ParameterValue(Command(['xacro ', urdf_model]), value_type=str)
  start_robot_state_publisher_cmd = Node(
    condition=IfCondition(use_robot_state_pub),
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='screen',
    parameters=[{
      'use_sim_time': use_sim_time, 
      'robot_description': robot_description_content}])

  start_rviz_cmd = Node(
    condition=IfCondition(use_rviz),
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_file])  
    
  start_gazebo_ros_spawner_cmd = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
      '-name', robot_name,
      '-topic', 'robot_description',
      '-x', x,
      '-y', y,
      '-z', z,
      '-R', roll,
      '-P', pitch,
      '-Y', yaw
      ],
    output='screen')  
    
  start_gazebo_ros_bridge_cmd = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    parameters=[{
      'config_file': default_ros_gz_bridge_config_file_path,
    }],
    output='screen'
  )  
    
  ld = LaunchDescription()

  ld.add_action(declare_robot_name_cmd)
  ld.add_action(declare_rviz_config_file_cmd)
  ld.add_action(declare_simulator_cmd)
  ld.add_action(declare_urdf_model_path_cmd)
  ld.add_action(declare_use_robot_state_pub_cmd)  
  ld.add_action(declare_use_rviz_cmd) 
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_use_simulator_cmd)
  ld.add_action(declare_world_cmd)

  ld.add_action(declare_x_cmd)
  ld.add_action(declare_y_cmd)
  ld.add_action(declare_z_cmd)
  ld.add_action(declare_roll_cmd)
  ld.add_action(declare_pitch_cmd)
  ld.add_action(declare_yaw_cmd)  

  ld.add_action(set_env_vars_resources)
  ld.add_action(start_gazebo_server_cmd)
  ld.add_action(start_gazebo_client_cmd)
  ld.add_action(start_robot_state_publisher_cmd)
  ld.add_action(start_rviz_cmd)
  
  ld.add_action(start_gazebo_ros_spawner_cmd)
  ld.add_action(start_gazebo_ros_bridge_cmd)

  return ld