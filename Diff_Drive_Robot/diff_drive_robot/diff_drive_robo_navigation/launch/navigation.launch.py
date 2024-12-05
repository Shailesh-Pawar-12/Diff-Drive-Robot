import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable,GroupAction,OpaqueFunction
from launch_ros.actions import LoadComposableNodes, SetParameter,PushRosNamespace
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_setup(context,*args,**kwargs):
    bringup_dir = get_package_share_directory('diff_drive_robo_navigation')

    namespace = LaunchConfiguration('namespace').perform(context)
    output_type='log' #screen
    use_sim_time=True
    autostart=False
    
    autostart = LaunchConfiguration('autostart')
    bond_timeout = LaunchConfiguration('bond_timeout')    
    bond_respawn_max_duration = LaunchConfiguration('bond_respawn_max_duration')
    attempt_respawn_reconnection = LaunchConfiguration('attempt_respawn_reconnection')
    
    params_file = LaunchConfiguration('params_file')
    

    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')

   

    lifecycle_nodes = ['controller_server',
                       'smoother_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower',
                       'velocity_smoother'
                       ]

    nav2_controller=Node(
                package='nav2_controller',
                executable='controller_server',
                output=output_type,
                name='controller_server',
                parameters=[params_file],
                namespace=namespace,
                remappings= [('cmd_vel', '/diff_drive_base_controller/cmd_vel_unstamped')])
    
    nav2_smoother=Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                namespace=namespace,
                output=output_type,
                parameters=[params_file],
                # arguments=['--ros-args', '--log-level', log_level],
                )
    nav2_planner=Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                namespace=namespace,
                output=output_type,
                parameters=[params_file],
                # arguments=['--ros-args', '--log-level', log_level],
                )
    nav2_behaviours=Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                namespace=namespace,
                output=output_type,
                parameters=[params_file],
                remappings=[('cmd_vel','mobile_base_controller/cmd_vel_unstamped')],
                # arguments=['--ros-args', '--log-level', log_level],
                )
    nav2_bt=Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                namespace=namespace,
                output=output_type,
                parameters=[params_file,
                            {'default_nav_through_poses_bt_xml':LaunchConfiguration('bt_path_arg_xml')}],
                # arguments=['--ros-args', '--log-level', log_level],
                )
    nav2_waypoint=Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                namespace=namespace,
                output='screen',
                parameters=[params_file],
                # arguments=['--ros-args', '--log-level', log_level],
                )
    nav2_velocity_smoother=Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                namespace=namespace,
                output=output_type,
                parameters=[params_file],
                # arguments=['--ros-args', '--log-level', log_level],
                remappings=[('cmd_vel', '/diff_drive_base_controller/cmd_vel_unstamped'), ('cmd_vel_smoothed', 'mobile_base_controller/cmd_vel_unstamped')])

    nav2_lifecycle=Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='diff_drive_robo_navigation_lifecycle_manager',
                namespace=namespace,
                output='screen',
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes},
                            {'bond_timeout': bond_timeout},
                            {'bond_respawn_max_duration': bond_respawn_max_duration},
                            {'attempt_respawn_reconnection': attempt_respawn_reconnection}])    


    return [nav2_controller,nav2_smoother,nav2_planner,nav2_behaviours,nav2_bt,nav2_velocity_smoother,nav2_waypoint,nav2_lifecycle]


def generate_launch_description():
    namespace=DeclareLaunchArgument('namespace',default_value="")
    bt_path = PathJoinSubstitution([FindPackageShare('diff_drive_behavior_tree'), 'behavior_trees', 'start.xml'])
    bt_path_arg = DeclareLaunchArgument(name='bt_path_arg_xml', default_value=bt_path, description='Absolute path to diff_drive behavior file xml')
    
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
    ld.add_action(bt_path_arg)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_bond_timeout_cmd)
    ld.add_action(declare_bond_respawn_max_duration_cmd)
    ld.add_action(declare_attempt_respawn_reconnection_cmd)
    ld.add_action(function)
    return ld









