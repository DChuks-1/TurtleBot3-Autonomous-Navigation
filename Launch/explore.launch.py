from launch import LaunchDescription
from launch_ros.actions import Node

import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription( 
            PythonLaunchDescriptionSource( 
                os.path.join( 
                    get_package_share_directory("tuos_simulations"), 
                    "launch", "acs6121.launch.py" 
                )
            )
        ),
         IncludeLaunchDescription( 
             PythonLaunchDescriptionSource( 
                 os.path.join( 
                     get_package_share_directory("tuos_simulations"), 
                     "launch", "cartographer.launch.py" 
                 )
             )
         ),
         IncludeLaunchDescription( 
             PythonLaunchDescriptionSource( 
                 os.path.join( 
                     get_package_share_directory("nav2_map_server"), 
                     "launch", "map_saver_server.launch.py" 
                 )
             )
         ),
        Node(
        package='acs6121_team24_2025',
        executable='exploration.py',
        name='exploration',
        output='screen',
        ),
        Node(
            package='acs6121_team24_2025',
            executable='lidar.py',
            name='lidar'
        ),
        Node(
            package='acs6121_team24_2025',
            executable='move.py',
            name='move'
        ),
        Node(
            package='acs6121_team24_2025',
            executable='decision.py',
            name='decision'
        ),
        Node(
             package='acs6121_team24_2025',
             executable='map_saver_client.py',
             name='map_saver_client'
         )
    ])