from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # 1. Spawner for Joint State Broadcaster (The "Reporter")
    # This reads data from the hardware/sim and publishes to /joint_states
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # 2. Spawner for Arm Controller (The "Driver")
    # This listens to move commands and drives the joints
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
    )

    # 3. Spawner for Gripper Controller 
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
        output="screen",
    )

    return LaunchDescription([
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner
    ])