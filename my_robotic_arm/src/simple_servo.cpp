#include <rclcpp/rclcpp.hpp>
#include <moveit_servo/servo.h>
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/status_codes.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <tf2_ros/transform_listener.h> 

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<rclcpp::Node>("custom_servo_node");
  RCLCPP_INFO(node->get_logger(), "STARTING CUSTOM SERVO NODE");

  // 1. PLANNING SCENE MONITOR
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      node, 
      "robot_description", 
      tf_buffer, 
      "planning_scene_monitor"
  );

  if (planning_scene_monitor->getPlanningScene())
  {
      planning_scene_monitor->startStateMonitor();
      planning_scene_monitor->startSceneMonitor();
      planning_scene_monitor->startWorldGeometryMonitor();
      RCLCPP_INFO(node->get_logger(), "Planning Scene Monitor Started!");
  }
  else
  {
      RCLCPP_ERROR(node->get_logger(), "Planning Scene Monitor failed to initialize!");
      return -1;
  }

  // 2. LOAD DEFAULT PARAMETERS
  auto default_params = moveit_servo::ServoParameters::makeServoParameters(node);
  
  if (!default_params) {
      RCLCPP_ERROR(node->get_logger(), "Could not load servo parameters object!");
      return -1;
  }

  // 3. CREATE MUTABLE COPY & OVERWRITE
  moveit_servo::ServoParameters modified_params = *default_params;

  RCLCPP_WARN(node->get_logger(), "Overwriting 'move_group_name' to 'arm_group' manually...");

  // --- CRITICAL OVERRIDES ---
  // 1. The Group Name (FIXED)
  modified_params.move_group_name = "arm_group";

  // 2. The Frames (FIXING NOW)
  // "panda_link8" error comes from here. We change it to YOUR end effector link.
  // Based on your previous logs, your arm ends at "Link_6".
  modified_params.ee_frame_name = "Link_6"; 

  // The frame your commands are given in (usually base_link)
  modified_params.robot_link_command_frame = "base_link"; 
  
  // The planning frame (usually base_link)
  modified_params.planning_frame = "base_link";
  modified_params.incoming_command_timeout = 1000000.0; // 1 million seconds
  
  

  // 3. Other Settings
  modified_params.command_in_type = "speed_units";
  modified_params.command_out_topic = "/velocity_controller/commands";
  modified_params.cartesian_command_in_topic = "/delta_twist_cmds";
  modified_params.check_collisions = true;
  modified_params.publish_period = 0.034;
  modified_params.linear_scale = 0.4;
  modified_params.rotational_scale = 0.8;

  // 4. RE-PACKAGE
  auto params_ptr = std::make_shared<moveit_servo::ServoParameters>(modified_params);

  RCLCPP_INFO(node->get_logger(), "Parameter overwrite complete. Group is now: %s", params_ptr->move_group_name.c_str());

  // 5. START SERVO
  auto servo = std::make_unique<moveit_servo::Servo>(node, params_ptr, planning_scene_monitor);
  
  servo->start();
  RCLCPP_INFO(node->get_logger(), "Servo started successfully!");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}