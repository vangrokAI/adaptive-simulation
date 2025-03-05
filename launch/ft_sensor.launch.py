#!/usr/bin/env python3
# Launch file for the ATI Axia80-M20 Force/Torque sensor

import os
from pathlib import Path
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
    
    sensor_config_arg = DeclareLaunchArgument(
        'sensor_config',
        default_value=os.path.join(adaptive_simulation_pkg_dir, 'config', 'ati_axia80_m20.yaml'),
        description='Path to sensor configuration file'
    )
    
    use_hardware_arg = DeclareLaunchArgument(
        'use_hardware',
        default_value='false',
        description='Use actual hardware sensor instead of simulation'
    )
    
    sensor_ip_arg = DeclareLaunchArgument(
        'sensor_ip',
        default_value='192.168.1.1',
        description='IP address of the ATI Axia sensor (only used if use_hardware is true)'
    )
    
    # Nodes to launch
    nodes = []
    
    # Choose between simulated and hardware sensor
    use_hardware = LaunchConfiguration('use_hardware')
    
    # Simulated F/T sensor node using our custom implementation
    ft_sensor_sim_node = Node(
        package='adaptive_simulation',
        executable='ft_sensor_node',
        name='ft_sensor_node',
        output='screen',
        parameters=[
            LaunchConfiguration('sensor_config'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'ceiling_mounted': LaunchConfiguration('ceiling_mounted'),
                'simulated_sensor': True,
            }
        ],
        condition=launch.conditions.UnlessCondition(use_hardware)
    )
    
    # Hardware F/T sensor node with EtherCAT support for ATI Axia80-M20
    # This supports EtherCAT communication via TwinCAT 3
    ft_sensor_hw_node = Node(
        package='adaptive_simulation',
        executable='ft_sensor_node',
        name='ft_sensor_hardware',
        output='screen',
        parameters=[
            LaunchConfiguration('sensor_config'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'ceiling_mounted': LaunchConfiguration('ceiling_mounted'),
                'simulated_sensor': False,
                'hardware_interface_type': 'ethercat',
                'use_ethercat': True,
                'use_hardware_interface': True,
                'ip_address': LaunchConfiguration('sensor_ip'), # nur für Diagnose
                'ethercat_slave_id': 1,
                'twincat.enabled': True
            }
        ],
        condition=launch.conditions.IfCondition(use_hardware)
    )
    
    # TF publisher for the sensor
    # This publishes a static transform from robot flange to sensor
    sensor_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='sensor_tf_publisher',
        arguments=['0', '0', '0.05', '0', '0', '0', 'lbr_link_7', 'ati_axia80_link']
    )
    
    # Visualization node
    ft_visualization_node = Node(
        package='adaptive_simulation',
        executable='ft_visualization_node',
        name='ft_visualization_node',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'sensor_frame': 'ati_axia80_measurement_link',
                'force_arrow_scale': 0.01,
                'torque_arrow_scale': 0.02,
                'force_topic': '/ati_axia80/wrench',
                'ceiling_mounted': LaunchConfiguration('ceiling_mounted')
            }
        ]
    )
    
    # Add nodes to launch description
    nodes.append(ft_sensor_sim_node)
    nodes.append(ft_sensor_hw_node)
    nodes.append(sensor_tf_publisher)
    nodes.append(ft_visualization_node)
    
    # Create and return launch description
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        ceiling_mounted_arg,
        sensor_config_arg,
        use_hardware_arg,
        sensor_ip_arg,
        
        # Nodes
        *nodes
    ])
