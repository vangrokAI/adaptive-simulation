from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            description='Verwende simulierten F/T-Sensor statt ADS-Verbindung'
        ),
        
        DeclareLaunchArgument(
            'twincat_netid',
            default_value='192.168.2.100.1.1',
            description='AmsNetId des TwinCAT Systems'
        ),
        
        DeclareLaunchArgument(
            'twincat_port',
            default_value='851',
            description='ADS-Port für PLC Runtime'
        ),
        
        DeclareLaunchArgument(
            'ft_data_varname',
            default_value='MAIN.FT_Data',
            description='Name der F/T-Daten-Variable im TwinCAT PLC'
        ),
        
        DeclareLaunchArgument(
            'sim_mode',
            default_value='sine',
            description='Simulationsmodus: sine, constant, random'
        ),
        
        DeclareLaunchArgument(
            'publish_rate',
            default_value='500.0',
            description='Veröffentlichungsrate in Hz'
        ),
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Verwende Simulationszeit'
        ),
        
        Node(
            package='ft_twincat_bridge',
            executable='ft_sensor_node',
            name='ft_sensor',
            output='screen',
            parameters=[{
                'data_source': LaunchConfiguration('use_sim', default='sim'),
                'twincat_netid': LaunchConfiguration('twincat_netid'),
                'twincat_port': LaunchConfiguration('twincat_port'),
                'ft_data_varname': LaunchConfiguration('ft_data_varname'),
                'sim_mode': LaunchConfiguration('sim_mode'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        )
    ])
