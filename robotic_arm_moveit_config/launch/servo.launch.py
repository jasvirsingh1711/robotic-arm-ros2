import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. Load the Robot Configuration
    moveit_config = MoveItConfigsBuilder("robotic_arm", package_name="robotic_arm_moveit_config").to_moveit_configs()

    # 2. Get the path to the config file
    pkg_share = get_package_share_directory("robotic_arm_moveit_config")
    
    # We try to load from the 'servo' subfolder
    servo_yaml_path = os.path.join(pkg_share, "config", "servo", "moveit_servo_config.yaml")

    # 3. Load the YAML content safely
    try:
        with open(servo_yaml_path, 'r') as f:
            servo_params = yaml.safe_load(f)
    except EnvironmentError:
        print(f"[WARN] Could not find config at {servo_yaml_path}, using empty defaults.")
        servo_params = {}

    
    servo_params["move_group_name"] = "arm_group"  # <--- FORCE THIS
    servo_params["planning_frame"] = "base_link"
    servo_params["robot_link_command_frame"] = "gripper-base"
    servo_params["command_in_type"] = "speed_units"


    print(f"-------- FORCING MOVE GROUP TO: {servo_params['move_group_name']} --------")

    # 4. Create the Servo Node
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )

    return LaunchDescription([servo_node])