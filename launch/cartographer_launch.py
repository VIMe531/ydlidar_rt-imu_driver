#!/usr/bin/python3

from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import LogInfo

import lifecycle_msgs.msg
import os

def generate_launch_description():
	# File paths
    lidar_share_dir = get_package_share_directory('ydlidar_ros2_driver')
    lidar_parameter_file = LaunchConfiguration('lidar_params_file')
    driver_config_dir = get_package_share_directory('lidar_imu_driver')
    rviz_config_file = os.path.join(driver_config_dir, 'config','cartographer_2d.rviz')
    xacro_file = os.path.join(driver_config_dir, 'urdf', 'hand_lidar.xacro')
    
    params_declare = DeclareLaunchArgument('lidar_params_file',
                                           default_value=os.path.join(
                                               lidar_share_dir, 'params', 'TG.yaml'),
                                           description='FPath to the ROS2 parameters file to use.')

	# Nodes
    robot_state_publisher_node = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        parameters=[
            {'robot_description': Command(['xacro ', xacro_file])},
            {'use_sim_time': False}],
        output = 'screen'
        )
    
    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        parameters = [{'use_sim_time': False}],
        arguments = [
            '-configuration_directory', driver_config_dir + '/config',
            '-configuration_basename', 'cartographer_2d.lua',
            # '-load_state_filename', LaunchConfiguration('load_state_filename')
            ],
        # remappings = [
        #    ('/imu/data_raw',	'/imu')
        #    ],
        output = 'screen'
        )
    
    cartographer_occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        parameters = [
            {'use_sim_time': False},
            {'resolution': 0.05}],
        )

    lidar_driver_node = Node(package='ydlidar_ros2_driver',
                                executable='ydlidar_ros2_driver_node',
                                name='ydlidar_ros2_driver_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[lidar_parameter_file],
                                # node_namespace='/',
                                )
	
    imu_driver_node = Node(name='rt_usb_9axisimu_driver',
        					package='rt_usb_9axisimu_driver',
        					executable='rt_usb_9axisimu_driver',
        					output='screen'
    						)
    
    tf2_node = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_pub_laser',
                    arguments=['0', '0', '0.02','0', '0', '0', '1','base_link','laser_frame'],
                    )
    
    rviz2_node = Node(package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config_file],
                    )
    
    # imu lifecycle set
    imu_lifecycle_configure = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', 'rt_usb_9axisimu_driver', 'configure'],
        output='screen'
    )

    imu_lifecycle_activate = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', 'rt_usb_9axisimu_driver', 'activate'],
        output='screen'
    )
	                
    return LaunchDescription([
        # Launch Parameters
        params_declare,
        # Nodes
        robot_state_publisher_node,
        cartographer_node,
        cartographer_occupancy_grid_node,
        lidar_driver_node,
        imu_driver_node,
        tf2_node,
        rviz2_node,
        imu_lifecycle_configure,
        imu_lifecycle_activate,
    ])
