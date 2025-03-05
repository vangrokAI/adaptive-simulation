#!/usr/bin/env python3
# Launch file for surface contact demonstration with ATI Axia80-M20 F/T sensor

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # Get package directories
    adaptive_simulation_pkg_dir = get_package_share_directory('adaptive_simulation')
    lbr_bringup_pkg_dir = get_package_share_directory('lbr_bringup')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    
    ceiling_mounted_arg = DeclareLaunchArgument(
        'ceiling_mounted',
        default_value='true',
        description='Whether the robot is mounted on the ceiling (true) or floor (false)'
    )
    
    use_hardware_arg = DeclareLaunchArgument(
        'use_hardware',
        default_value='false',
        description='Use real hardware (true) or simulation (false)'
    )
    
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='iiwa14',
        description='Robot model to use (iiwa7 or iiwa14)'
    )
    
    controller_arg = DeclareLaunchArgument(
        'ctrl',
        default_value='joint_trajectory_controller',
        description='Controller to start (joint_trajectory_controller, joint_position_controller, ...)'
    )
    
    # Use the appropriate launch file based on use_hardware
    use_hardware = LaunchConfiguration('use_hardware')
    
    # Hardware launch
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([lbr_bringup_pkg_dir, '/launch/hardware.launch.py']),
        launch_arguments={
            'model': LaunchConfiguration('model'),
            'ctrl': LaunchConfiguration('ctrl'),
            'use_sim_time': 'false'  # Always use real time with hardware
        }.items(),
        condition=launch.conditions.IfCondition(use_hardware)
    )
    
    # Gazebo simulation launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([lbr_bringup_pkg_dir, '/launch/gazebo.launch.py']),
        launch_arguments={
            'model': LaunchConfiguration('model'),
            'ctrl': LaunchConfiguration('ctrl'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items(),
        condition=launch.conditions.UnlessCondition(use_hardware)
    )
    
    # F/T sensor launch
    ft_sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([adaptive_simulation_pkg_dir, '/launch/ft_sensor.launch.py']),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'ceiling_mounted': LaunchConfiguration('ceiling_mounted'),
            'use_hardware': LaunchConfiguration('use_hardware')
        }.items()
    )
    
    # Surface contact node
    surface_contact_node = Node(
        package='adaptive_simulation',
        executable='surface_contact_node',
        name='surface_contact_node',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'ceiling_mounted': LaunchConfiguration('ceiling_mounted'),
                'use_ft_sensor': True,
                'ft_topic': '/ati_axia80/wrench',
                'contact_force_threshold': 5.0,  # N
                'max_force': 25.0,  # N
                'approach_speed': 0.05,  # m/s
                'retract_speed': 0.1,  # m/s
                'controller_namespace': '/lbr/controller_manager',
                'robot_frame': 'lbr_base_link',
                'end_effector_frame': 'ati_axia80_measurement_link'
            }
        ]
    )
    
    # RViz configuration
    rviz_config_file = os.path.join(adaptive_simulation_pkg_dir, 'config', 'surface_contact_ft.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # Create and return launch description
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        ceiling_mounted_arg,
        use_hardware_arg,
        model_arg,
        controller_arg,
        
        # Launch files
        hardware_launch,
        gazebo_launch,
        ft_sensor_launch,
        
        # Nodes
        surface_contact_node,
        rviz_node
    ])
