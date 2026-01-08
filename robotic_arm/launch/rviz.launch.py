import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'robotic_arm' 
    urdf_file_name = 'arm_gripper_urdf.urdf' 

    # 1. Get the path to the URDF file
    urdf_path = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        urdf_file_name
    )

    # 2. Read the URDF file content
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # 3. Define the nodes
    
    # Node A: Robot State Publisher (Calculates Link Positions - Fixes "No Transform")
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # Node B: Joint State Publisher GUI 
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # Node C: RViz2
    # Ensure you have a 'view_robot.rviz' file, or remove the arguments list to open empty RViz
    

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        
    )

    # 4. Return the Launch Description
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])