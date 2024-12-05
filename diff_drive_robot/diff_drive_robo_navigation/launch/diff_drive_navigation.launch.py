#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml,ReplaceString

def launch_setup(context,*args,**kwargs):
    namespace=LaunchConfiguration('namespace').perform(context)
    bringup_dir = get_package_share_directory('diff_drive_robo_navigation')
    launch_dir = os.path.join(bringup_dir, 'launch')
    nav2_params=os.path.join(bringup_dir,'config','nav2_params.yaml') 
    nav2_launch_file=os.path.join(launch_dir,'navigation.launch.py')
    odom_frame='odom'
    map_frame='map'
    base_frame='base_footprint'
    
    params_={'global_frame': map_frame,'local_frame': odom_frame,'robot_base_frame': base_frame}
    
    
    diff_drive_robo_nav_launch=IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(nav2_launch_file),
                            launch_arguments={'namespace': namespace,'params_file':nav2_params}.items(),)  #'use_sim_time':'True','autostart':'True',
    return [diff_drive_robo_nav_launch]

def generate_launch_description():
    namespace=DeclareLaunchArgument('namespace',default_value='')
    function=OpaqueFunction(function=launch_setup)
 
    ld=LaunchDescription()
    ld.add_action(namespace)
    ld.add_action(function)
    return ld
