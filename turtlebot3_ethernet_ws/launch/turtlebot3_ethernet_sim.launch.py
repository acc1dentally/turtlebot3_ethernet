import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    
    turtlebot3_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
        ),
    )
    
    ethernet_bridge_node = Node(
        package='turtlebot3_ethernet_bridge',
        executable='ethernet_bridge_node',
        name='ethernet_bridge',
        parameters=[{
            'ip_address': '192.168.1.100',
            'port': 5000,
            'bandwidth_limit': 1000000,  # 1 Mbps
            'latency_ms': 2,  # 2ms latency
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )
    
    ethernet_controller_node = Node(
        package='turtlebot3_ethernet_bridge',
        executable='ethernet_controller_node',
        name='ethernet_controller',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        turtlebot3_gazebo_launch,
        ethernet_bridge_node,
        ethernet_controller_node,
    ])