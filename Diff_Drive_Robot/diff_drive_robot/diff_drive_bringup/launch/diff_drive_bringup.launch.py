import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,OpaqueFunction,TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_setup(context,*args,**kwargs):
    
    print("######## Launching Robot ###########")
    description_dir=get_package_share_directory('diff_drive_description')
    localization_dir=get_package_share_directory('diff_drive_robo_localization')
    navigation_dir=get_package_share_directory('diff_drive_robo_navigation')

    description_file='diff_drive.launch.py'
    ekf_file='diff_drive_ekf.launch.py'
    map_amcl_file='diff_drive_map_amcl.launch.py'
    navigation_file='diff_drive_navigation.launch.py'    
   
    include_description=IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(description_dir,'launch',description_file)))
    include_ekf=IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(localization_dir,'launch',ekf_file)))
    include_map_amcl=IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(localization_dir,'launch',map_amcl_file)))
    include_navigation=IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(navigation_dir,'launch',navigation_file)),)
        
    amcl_timer=TimerAction(period=10.0,actions=[include_map_amcl])
    navigation_timer=TimerAction(period=13.0,actions=[include_navigation])

    return [include_description,include_ekf,amcl_timer,navigation_timer]


def generate_launch_description():
    function=OpaqueFunction(function=launch_setup)
    ld=LaunchDescription()
    ld.add_action(function)
    return ld
    
