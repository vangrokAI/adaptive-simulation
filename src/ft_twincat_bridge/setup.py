from setuptools import setup, find_packages

package_name = 'ft_twincat_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ft_sensor.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adaptive Simulation Team',
    maintainer_email='team@example.com',
    description='Bridge between TwinCAT ADS and ROS2 for ATI Axia80-M20 F/T sensor',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ft_sensor_node = ft_twincat_bridge.ft_sensor_node:main',
        ],
    },
)
