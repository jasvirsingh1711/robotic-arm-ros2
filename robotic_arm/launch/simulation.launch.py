import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'robotic_arm'
    pkg_share = get_package_share_directory(pkg_name)

    install_base_path = os.path.dirname(pkg_share) 
    
    gz_resource_path_env = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', 
        value=install_base_path
    )

    # --- PATHS ---
    gz_launch_path = os.path.join(pkg_share, 'launch', 'gz.launch.py')
    spawn_launch_path = os.path.join(pkg_share, 'launch', 'spawn_robot.launch.py')
    display_launch_path = os.path.join(pkg_share, 'launch', 'display.launch.py')
    controllers_launch_path = os.path.join(pkg_share, 'launch', 'controllers.launch.py')

    # --- ACTIONS ---

    # 1. Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path)
    )

    # 2. Spawn Robot (Wait 3 seconds)
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(spawn_launch_path)
            )
        ]
    )

    # 3. Launch Controllers (Wait 8 seconds)
    launch_controllers = TimerAction(
        period=8.0, 
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(controllers_launch_path)
            )
        ]
    )

    # 4. RViz + RSP
    display_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(display_launch_path),
        launch_arguments={
            'use_sim_time': 'true',
            'use_gui': 'false' 
        }.items()
    )

    # 5. Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )

    return LaunchDescription([
        gz_resource_path_env, # <--- Added this to fix the mesh error
        bridge,
        gazebo,
        spawn_robot,
        launch_controllers,
        display_stack
    ])