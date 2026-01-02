import os
import yaml
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    moveit_config_pkg = 'my_robotic_arm_moveit_config'
    description_pkg = 'my_robotic_arm'

    
    def load_yaml(package_name, file_path):
        package_path = get_package_share_directory(package_name)
        absolute_file_path = os.path.join(package_path, file_path)
        try:
            with open(absolute_file_path, 'r') as file:
                return yaml.safe_load(file)
        except EnvironmentError:
            return None

    kinematics_yaml = load_yaml(moveit_config_pkg, 'config/kinematics.yaml')

    # Robot Description
    controllers_file = os.path.join(get_package_share_directory(description_pkg), 'config', 'my_controllers.yaml')
    urdf_path = os.path.join(get_package_share_directory(description_pkg), 'urdf', 'arm_urdf.urdf')
    doc = xacro.process_file(urdf_path, mappings={'controllers_yaml': controllers_file})
    robot_description = {'robot_description': doc.toxml()}

    srdf_path = os.path.join(get_package_share_directory(moveit_config_pkg), 'config', 'arm_urdf.srdf')
    with open(srdf_path, 'r') as f:
        robot_description_semantic = {"robot_description_semantic": f.read()}

    # --- USING OUR CUSTOM C++ NODE ---
    servo_node = Node(
        package="my_robotic_arm",  # <-- Ab ye description package se chalega
        executable="simple_servo_node", # <-- Humara compile kiya hua executable
        name="custom_servo_node",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            # Parameters ab C++ mein default set hain, par hum yahan bhi pass kar sakte hain
            {"move_group_name": "arm_group"}, 
        ],
    )

    return LaunchDescription([servo_node])