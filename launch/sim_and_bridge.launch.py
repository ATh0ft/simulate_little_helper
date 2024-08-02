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

import xacro 


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ign_ros_bridge = get_package_share_directory('ros_gz_bridge')
    pkg_lh_description = get_package_share_directory('little_helper_description')
    pkg_lh_sim = get_package_share_directory('simulate_little_helper')
    pkg_lh_urdf = get_package_share_directory('little_helper_urdf')

    sdf_file_path = os.path.join(pkg_lh_description, 'model', 'model', 'little_helper.sdf')
    urdf_file_path = os.path.join(pkg_lh_urdf, 'little_helper10_arm_on_left.urdf')
    
    #read the urdf file as a string 
    with open(urdf_file_path, 'r') as urdf_file:
        robot_description_urdf = urdf_file.read()
    
    with open(sdf_file_path, 'r') as sdf_file:
        robot_description_sdf = sdf_file.read().replace('model://', 'package://little_helper_description/model/')

    robot_name = 'little_helper'

    ign_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ), 
        # set it to use the empty world specified in the pgk 
        launch_arguments={'ign_args': os.path.join(pkg_lh_sim, 'worlds', 'workspace1.sdf') +  ' -v'}.items(),
    )

    spawn_little_helper = Node(package='ros_gz_sim',
                               executable='create',
                               name='spawn_lh',
                               arguments=['-name', 'little_helper', '-file', sdf_file_path],
                               output='screen')
    
    robot_describer = Node(package='robot_state_publisher',
                           executable='robot_state_publisher',
                           parameters=[{'robot_description':robot_description_sdf,'frame_prefix':f'{robot_name}/' }],
                           output='screen'
                           )


    # bridges 
    
    model_prefix = f'/model/{robot_name}'

    lidar_bridge_b = Node(package='ros_gz_bridge', 
                          executable = 'parameter_bridge',
                          name='b_scan_bridge',
                          arguments=["/b_scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan"],
                          output = 'screen')

    # the static lidar transform the same as in the sdf, the full tf tree should be generated from the sdf so this is hopefully temporary  
    b_lidar_yaw = 3.92+3.1415
    b_lidar_static_transform = Node(package='tf2_ros',
                                    executable = 'static_transform_publisher',
                                    name='b_lidar_tf_pub',
                                    arguments= [
                                    # '-0.392', '-0.2358', '0.1914', f'{b_lidar_yaw}', '3.1415', '0',
                                               '0', '0', '0', '0', '0', '0',
                                    robot_name + '/b_laser_link',
                                    robot_name + '/b_laser_link/gpu_lidar',
                        
                                    ])
    
    lidar_bridge_f = Node(package='ros_gz_bridge', 
                          executable = 'parameter_bridge',
                          name='f_scan_bridge',
                          arguments=["/f_scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan"],
                          output = 'screen')   
    # the static lidar transform 
    f_lidar_yaw = 0.79 + 3.1415
    f_lidar_static_transform = Node(package='tf2_ros',
                                    executable = 'static_transform_publisher',
                                    name='f_lidar_tf_pub',
                                    arguments= [
                                    # '0.392', '0.2358', '0.1914', f'{f_lidar_yaw}', '3.1415', '0',
                                                '0', '0', '0', '0', '0', '0',
                                    robot_name + '/f_laser_link',
                                    robot_name + '/f_laser_link/gpu_lidar',
                        
                                    ])

    odom_bridge = Node(package='ros_gz_bridge',
                       executable='parameter_bridge',
                       name='odom_bridge',
                       arguments=['/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry'],
                       output='screen')
    

    odom_tf_bridge = Node(package='ros_gz_bridge',
                          executable='parameter_bridge',
                          name='odom_tf_bridge',
                          arguments=[model_prefix + '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'],
                          remappings=[(model_prefix+'/tf', '/tf')],
                          output='screen')

    # static_tf_chassis = Node(package='tf2_ros',
    #                          executable = 'static_transform_publisher',
    #                          name=''
    #                          arguments=['0','0','0','0','0' '0',
    #                                     robot_name,
    #                                     robot_name + '/b_laser_link/gpu_lidar',
    #                                     ]
    #                          )
    vt_laser_link_tf = Node(package='tf2_ros',
                            executable= 'static_transform_publisher',
                            name='virtual_laser_link_tf_pub',
                            arguments=[
                            '0', '0', '0', '0', '0', '0',
                            robot_name + '/chassis',
                            'virtual_laser_link'
                            ])
    
 

    joint_state_bridge = Node(package='ros_gz_bridge',
                              executable='parameter_bridge',
                              name='joint_state_bridge',
                              arguments=['joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model'],
                              output='screen')

    clock_bridge = Node(package='ros_gz_bridge',
                        executable='parameter_bridge',
                        name='clock_bridge',
                        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
                        output='screen'
                        )

    cmd_vel_bridge = Node(package='ros_gz_bridge',
                          executable='parameter_bridge',
                          name='cmd_vel_bridge',
                          arguments=['/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'],
                          output='screen')

        
    
    # ur joint position bridges 

    ur_joint_pos_topic_names = ['shoulder_pan_joint_cmd',
                                'shoulder_lift_joint_cmd',
                                'elbow_joint_cmd',
                                'wrist_1_joint_cmd',
                                'wrist_2_joint_cmd',
                                'wrist_3_joint_cmd']
    
    ur_join_bridge_nodes = []

    for joint_topic_name in ur_joint_pos_topic_names:
        ur_join_bridge_nodes.append(
            Node(package='ros_gz_bridge',
                 executable='parameter_bridge',
                 name=f'{joint_topic_name}_bridge',
                 arguments=[f'/{joint_topic_name}@std_msgs/msg/Float64]ignition.msgs.Double'],
                 output='screen')
        )

    
    fortress_trj_control = Node(package='simulate_little_helper',
                                executable='fortress_trajectory_controller',
                                output='screen')




    return LaunchDescription([ign_sim, 
                              spawn_little_helper,
                              robot_describer,
                              joint_state_bridge,
                              lidar_bridge_b,
                              b_lidar_static_transform,
                              lidar_bridge_f,
                              f_lidar_static_transform,
                              odom_bridge,
                              odom_tf_bridge,
                              clock_bridge,
                              cmd_vel_bridge,
                              vt_laser_link_tf,
                              fortress_trj_control,
                              # laser_scan_merger
                              ]+ur_join_bridge_nodes)
