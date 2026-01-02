#!/usr/bin/env python3
"""
Launch ros2_control controllers
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    """Launch the controllers with delays to ensure proper startup"""
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # 1. Spawn Arm Controller (Load it, but keep it INACTIVE so it doesn't conflict)
    arm_controller_spawner = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["arm_controller", "--inactive"], # <--- ADDED --inactive
                output="screen",
            )
        ]
    )

    # 2. Spawn Velocity Controller (ACTIVE by default for Servo)
    velocity_controller_spawner = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["velocity_controller"], # Active by default
                output="screen",
            )
        ]
    )

    return LaunchDescription([
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        velocity_controller_spawner, # <--- THIS WAS MISSING!
    ])