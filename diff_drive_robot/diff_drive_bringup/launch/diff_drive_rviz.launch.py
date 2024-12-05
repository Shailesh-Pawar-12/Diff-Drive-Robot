import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,GroupAction,OpaqueFunction,TimerAction,RegisterEventHandler

def launch_setup(context, *args, **kwargs):
    print("######## Launching RViz ###########")
    
    # Path to the RViz configuration file
    rviz_dir = get_package_share_directory('diff_drive_description') 
    rviz_config_file = 'diff_drive.rviz'

    # Launch RViz with the specified configuration file
    rviz_launch = ExecuteProcess(
        cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d', os.path.join(rviz_dir, 'rviz', rviz_config_file)],
        output='screen'
    )
    
    rviz_timer = TimerAction(period=5.0, actions=[rviz_launch])

    return [rviz_timer]

def generate_launch_description():
    function = OpaqueFunction(function=launch_setup)
    ld = LaunchDescription()
    ld.add_action(function)
    return ld
