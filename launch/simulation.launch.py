#!/usr/bin/env python3
# Simulation Launch File Template for Adaptive Surface Contact
# This launch file serves as a template for setting up the simulation environment

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    adaptive_simulation_pkg_dir = get_package_share_directory('adaptive_simulation')
    lbr_description_pkg_dir = get_package_share_directory('lbr_description')
    lbr_bringup_pkg_dir = get_package_share_directory('lbr_bringup')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    
    obstacle_config_arg = DeclareLaunchArgument(
        'obstacle_config',
        default_value=os.path.join(adaptive_simulation_pkg_dir, 'config', 'obstacles.yaml'),
        description='Path to obstacle configuration file'
    )
    
    robot_model_arg = DeclareLaunchArgument(
        'model',
        default_value='iiwa14',
        description='Robot model to use (iiwa7 or iiwa14)'
    )
    
    controller_arg = DeclareLaunchArgument(
        'ctrl',
        default_value='joint_trajectory_controller',
        description='Controller to use'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='/lbr',
        description='Namespace for the robot'
    )
    
    # Ceiling-mounted argument (default to true as per requirements)
    ceiling_mounted_arg = DeclareLaunchArgument(
        'ceiling_mounted',
        default_value='true',
        description='Whether the robot is mounted on the ceiling (true) or floor (false)'
    )
    
    # Configuration parameters for Surface Contact Simulator
    surface_contact_simulator_params = {
        'robot_base_frame': 'world',
        'end_effector_frame': 'lbr_link_7',
        'force_sensor_frame': 'lbr_link_7',
        'force_threshold': 5.0,
        'approach_distance': 0.1,
        'contact_force_max': 10.0,
        'publish_frequency': 100.0,
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'ceiling_mounted': LaunchConfiguration('ceiling_mounted')  # Add ceiling mount parameter
    }
    
    # Configuration parameters for Obstacle Manager
    obstacle_manager_params = {
        'publish_frequency': 10.0,
        'reference_frame': 'world',
        'obstacle_config_path': LaunchConfiguration('obstacle_config'),
        'use_sim_time': LaunchConfiguration('use_sim_time')
    }
    
    # Configuration parameters for Simulation Visualization
    visualization_params = {
        'robot_base_frame': 'world',
        'end_effector_frame': 'lbr_link_7',
        'force_sensor_frame': 'lbr_link_7',
        'force_arrow_scale': 0.1,
        'torque_arrow_scale': 0.1,
        'text_scale': 0.1,
        'update_frequency': 50.0,
        'force_color_r': 204,
        'force_color_g': 51,
        'force_color_b': 51,
        'torque_color_r': 51,
        'torque_color_g': 51,
        'torque_color_b': 204,
        'history_size': 100,
        'use_sim_time': LaunchConfiguration('use_sim_time')
    }
    
    # Include LBR Gazebo simulation
    lbr_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(lbr_bringup_pkg_dir, 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'model': LaunchConfiguration('model'),
            'ctrl': LaunchConfiguration('ctrl'),
            'namespace': LaunchConfiguration('namespace'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'ceiling_mounted': LaunchConfiguration('ceiling_mounted')  # Pass the ceiling mount parameter
        }.items()
    )
    
    # Surface Contact Simulator Node
    surface_contact_simulator_node = Node(
        package='adaptive_simulation',
        executable='surface_contact_simulator_node',
        name='surface_contact_simulator_node',
        output='screen',
        parameters=[surface_contact_simulator_params],
    )
    
    # Obstacle Manager Node
    obstacle_manager_node = Node(
        package='adaptive_simulation',
        executable='obstacle_manager_node',
        name='obstacle_manager_node',
        output='screen',
        parameters=[obstacle_manager_params],
    )
    
    # Simulation Visualization Node
    simulation_visualization_node = Node(
        package='adaptive_simulation',
        executable='simulation_visualization_node',
        name='simulation_visualization_node',
        output='screen',
        parameters=[visualization_params],
    )
    
    # RViz with custom configuration
    rviz_config_file = os.path.join(
        adaptive_simulation_pkg_dir,
        'config',
        'adaptive_simulation.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
    )
    
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        obstacle_config_arg,
        robot_model_arg,
        controller_arg,
        namespace_arg,
        ceiling_mounted_arg,  # Added ceiling mount argument
        
        # Launch included files
        lbr_gazebo_launch,
        
        # Nodes
        surface_contact_simulator_node,
        obstacle_manager_node,
        simulation_visualization_node,
        rviz_node,
    ])
