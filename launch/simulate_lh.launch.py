import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_lh_description = get_package_share_directory('little_helper_description')
    pkg_lh_sim = get_package_share_directory('simulate_little_helper')

    
    sdf_file_path = os.path.join(pkg_lh_description, 'model', 'model', 'little_helper.sdf')
    
    #read the sdf file as a string 
    with open(sdf_file_path, 'r') as sdf_file:
        robot_description = sdf_file.read()

    ign_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ), 
        # set it to use the empty world specified in the pgk 
        launch_arguments={'ign_args': os.path.join(pkg_lh_sim, 'worlds', 'box_world.sdf') +  ' -v'}.items(),
    )

    spawn_little_helper = Node(package='ros_gz_sim',
                               executable='create',
                               arguments=['-name', 'little_helper', '-file', sdf_file_path],
                               output='screen')

    
    
    return LaunchDescription([ign_sim, spawn_little_helper ])
