#!/usr/bin/env python3
import os
from launch.actions import DeclareLaunchArgument,OpaqueFunction,RegisterEventHandler,TimerAction
from launch.event_handlers import OnProcessExit
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml






def launch_setup(context,*args,**kwargs):
    use_sim_time=LaunchConfiguration('use_sim_time')
    namespace=LaunchConfiguration('namespace').perform(context)

    autostart = LaunchConfiguration('autostart')
    bond_timeout = LaunchConfiguration('bond_timeout')    
    bond_respawn_max_duration = LaunchConfiguration('bond_respawn_max_duration')
    attempt_respawn_reconnection = LaunchConfiguration('attempt_respawn_reconnection')    
    # x_location=LaunchConfiguration('x_location').perform(context)
    # y_location=LaunchConfiguration('y_location').perform(context)
    # yaw=LaunchConfiguration('yaw').perform(context)
    
    x_location="0.0"
    y_location="0.0"
    yaw="0.0"
    
    # map_file_name=LaunchConfiguration('map_file').perform(context)
    map_file_name = "room.yaml"
    map_dir=get_package_share_directory('diff_drive_robo_localization')
    map_file=os.path.join(map_dir,'map',map_file_name)

    amcl_file_name='amcl.yaml'
    amcl_dir=get_package_share_directory('diff_drive_robo_localization')
    amcl_file=os.path.join(amcl_dir,'config',amcl_file_name)

    odom_frame_id='odom'
    base_frame_id='base_footprint'
    params_={'odom_frame_id': odom_frame_id,'base_frame_id': base_frame_id,'x': x_location,'y':y_location,'yaw': yaw}
    updated_params_=RewrittenYaml(source_file=amcl_file,root_key=namespace,param_rewrites=params_,convert_types=True)
    lifecycle_nodes=['map_server','amcl']
    map_server_node=Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='map_server',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time},{'yaml_filename': map_file}],)
    amcl_node=Node(
                 package='nav2_amcl',
                 executable='amcl',
               
                 name='amcl',
                 namespace=namespace,
                 output='screen',
                 parameters=[updated_params_],)
    
    lifecycle_node=Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name="diff_drive_robo_localization_lifecycle_manager",
                    namespace=namespace,
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time},
                                {'autostart': autostart},
                                {'node_names': lifecycle_nodes},
                                {'bond_timeout': bond_timeout},
                                {'bond_respawn_max_duration': bond_respawn_max_duration},
                                {'attempt_respawn_reconnection': attempt_respawn_reconnection}])
    
    
    
   
    return [map_server_node,amcl_node,lifecycle_node]






def generate_launch_description():
    namespace=DeclareLaunchArgument('namespace',default_value="")
    # x_location_=DeclareLaunchArgument('x_location',default_value='0.0')
    # y_location_=DeclareLaunchArgument('y_location',default_value='0.0')
    # yaw_=DeclareLaunchArgument('yaw',default_value='0.0')
    # map_file_declare_arg=DeclareLaunchArgument('map_file',default_value='room.yaml')
    use_sim_time=DeclareLaunchArgument('use_sim_time',default_value='true')


    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup')
    
    declare_bond_timeout_cmd = DeclareLaunchArgument(
        'bond_timeout', default_value='10.0',
        description='Bond timeout')
    
    declare_bond_respawn_max_duration_cmd = DeclareLaunchArgument(
        'bond_respawn_max_duration', default_value='20.0',
        description='Bond respawn max duration')
    
    declare_attempt_respawn_reconnection_cmd = DeclareLaunchArgument(
        'attempt_respawn_reconnection', default_value='true',
        description='Attempt respawn reconnection')

    function=OpaqueFunction(function=launch_setup)
    ld=LaunchDescription()
    ld.add_action(namespace)
    ld.add_action(use_sim_time)

    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_bond_timeout_cmd)
    ld.add_action(declare_bond_respawn_max_duration_cmd)
    ld.add_action(declare_attempt_respawn_reconnection_cmd)
    # ld.add_action(x_location_)
    # ld.add_action(y_location_)
    # ld.add_action(yaw_)
    ld.add_action(function)
    # ld.add_action(map_file_declare_arg)
    return ld





















