#!/usr/bin/python3

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo

import lifecycle_msgs.msg
import os

def generate_launch_description():
	params_declare = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(
                                               share_dir, 'params', 'TG.yaml'),
                                           description='FPath to the ROS2 parameters file to use.')

    lidar_driver_node = Node(package='ydlidar_ros2_driver',
                                executable='ydlidar_ros2_driver_node',
                                name='ydlidar_ros2_driver_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[parameter_file],
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
	                
	return LaunchDescription([
        params_declare,
        lidar_driver_node,
        imu_driver_node,
        tf2_node,
        rviz2_node,
    ])
