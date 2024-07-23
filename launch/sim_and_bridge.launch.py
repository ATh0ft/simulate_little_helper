import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.descriptions import executable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ign_ros_bridge = get_package_share_directory('ros_gz_bridge')
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



    # bridges 
    robot_name = 'little_helper'
    model_prefix = f'/model/{robot_name}'

    lidar_bridge_b = Node(package='ros_gz_bridge', 
                          executable = 'parameter_bridge',
                          arguments=["/b_lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan"],
                          output = 'screen')

    # the static lidar transform 
    b_lidar_yaw = 3.92+3.1415
    b_lidar_static_transform = Node(package='tf2_ros',
                                    executable = 'static_transform_publisher',
                                    arguments= [
                                    '-0.392', '-0.2358', '0.1914', f'{b_lidar_yaw}', '3.1415', '0',
                                               # '0', '0', '0', '3.92', '0', '0',
                                    robot_name,
                                    robot_name + '/b_laser_link/gpu_lidar',
                        
                                    ])
    
    lidar_bridge_f = Node(package='ros_gz_bridge', 
                          executable = 'parameter_bridge',
                          arguments=["/f_lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan"],
                          output = 'screen')   
    # the static lidar transform 
    f_lidar_yaw = 0.79 + 3.1415
    f_lidar_static_transform = Node(package='tf2_ros',
                                    executable = 'static_transform_publisher',
                                    arguments= [
                                    '0.392', '0.2358', '0.1914', f'{f_lidar_yaw}', '3.1415', '0',
                                                # '0', '0', '0', '0.79', '0', '0',
                                    robot_name,
                                    robot_name + '/f_laser_link/gpu_lidar',
                        
                                    ])

    odom_bridge = Node(package='ros_gz_bridge',
                       executable='parameter_bridge',
                       arguments=['/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry'],
                       output='screen')
    

    tf_bridge = Node(package='ros_gz_bridge',
                     executable='parameter_bridge',
                     arguments=[model_prefix + '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'],
                     remappings=[(model_prefix+'/tf', '/tf')],
                     output='screen')

    return LaunchDescription([ign_sim, 
                              spawn_little_helper, 
                              lidar_bridge_b,
                              b_lidar_static_transform,
                              lidar_bridge_f,
                              f_lidar_static_transform,
                              odom_bridge,
                              tf_bridge])
