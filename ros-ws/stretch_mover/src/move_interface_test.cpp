#include <chrono>
#include <memory>

#include <fmt/format.h>

// #include <cmath>
// #include <geometry_msgs/msg/point_stamped.hpp>
// #include <geometry_msgs/msg/transform_stamped.hpp>
// #include <rclcpp/client.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
// #include <rclcpp_action/rclcpp_action.hpp>
#include <thread>

// #include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/utils/moveit_error_code.h>
// #include <moveit_msgs/moveit_msgs/msg/planning_scene_world.hpp>
// #include <moveit_msgs/msg/motion_plan_request.hpp>
// #include <moveit_msgs/msg/position_constraint.hpp>
// #include <moveit_msgs/msg/orientation_constraint.hpp>
// #include <moveit_msgs/msg/bounding_volume.hpp>
// #include <moveit_msgs/msg/position_ik_request.hpp>
// #include <moveit_msgs/srv/get_position_ik.hpp>

// #include <moveit_msgs/action/move_group.hpp>

// #include <shape_msgs/msg/solid_primitive.hpp>

// #include <octomap/octomap.h>
// #include <octomap_msgs/conversions.h>
// #include <octomap_msgs/octomap_msgs/msg/octomap.hpp>
// #include <octomap_msgs/octomap_msgs/msg/octomap_with_pose.hpp>
// #include <rclcpp/utilities.hpp>
// #include <std_msgs/msg/header.hpp>
// #include <std_srvs/srv/trigger.hpp>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Transform.h>
// #include <tf2/convert.h>
// #include <tf2/transform_datatypes.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <tf2_ros/buffer.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
// #include <trajectory_msgs/msg/joint_trajectory.hpp>
// #include <visualization_msgs/msg/marker.hpp>
// #include <sensor_msgs/msg/multi_dof_joint_state.hpp>



class ArmCmder : public rclcpp::Node {
  public:
  // Member variables
  rclcpp::Node::SharedPtr node_ptr;
  moveit::planning_interface::MoveGroupInterface move_group_interface;
rclcpp::TimerBase::SharedPtr timer;

  ArmCmder(rclcpp::Node::SharedPtr node_ptr):
  Node("cmder"),
  node_ptr(node_ptr),
  move_group_interface(moveit::planning_interface::MoveGroupInterface(node_ptr, "mobile_base_arm"))
  {
    timer = create_wall_timer(std::chrono::milliseconds{500},  std::bind(&ArmCmder::timercb,this) );
  };

  void timercb(){
      auto state = move_group_interface.getCurrentState();


      auto ee_name = move_group_interface.getEndEffectorLink();
      RCLCPP_INFO_STREAM(get_logger(), fmt::format("ee name ", ee_name));

      auto current_pose = move_group_interface.getCurrentPose("link_grasp_center");
      RCLCPP_INFO_STREAM(get_logger(), fmt::format("Current ee pose \n {}", geometry_msgs::msg::to_yaml(current_pose)));
  }

};


int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  auto const interface_node = std::make_shared<rclcpp::Node>(
      "interface_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // moveit::planning_interface::MoveGroupInterface group_interface(interface_node,"mobile_base_arm");

  auto const arm_cmder = std::make_shared<ArmCmder>(interface_node);

  rclcpp::executors::MultiThreadedExecutor executor1;
  executor1.add_node(interface_node);
  executor1.add_node(arm_cmder);
  executor1.spin();

  rclcpp::shutdown();
  return 0;
}
