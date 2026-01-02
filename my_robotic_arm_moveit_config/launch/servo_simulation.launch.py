import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # --- PATHS ---
    pkg_description = get_package_share_directory('my_robotic_arm')
    pkg_moveit_config = get_package_share_directory('my_robotic_arm_moveit_config')

    # 1. ROBOT STATE PUBLISHER (Crucial Missing Piece!)
    # This publishes the URDF to the /robot_description topic so Gazebo can see it
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 2. LAUNCH GAZEBO
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'gazebo.launch.py')
        )
    )

    # 3. SPAWN ROBOT
    spawn_robot_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_description, 'launch', 'spawn_robot.launch.py')
                )
            )
        ]
    )

    # 4. LOAD CONTROLLERS
    controllers_launch = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_description, 'launch', 'controllers.launch.py')
                )
            )
        ]
    )

    # 5. START MOVEIT SERVO
    servo_launch = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_moveit_config, 'launch', 'moveit_servo.launch.py')
                )
            )
        ]
    )

    return LaunchDescription([
        rsp_launch,      # <--- Added this
        gazebo_launch,
        spawn_robot_launch,
        controllers_launch,
        servo_launch
    ])