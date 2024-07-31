#include <chrono>
#include <memory>

#include <rclcpp/client.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/utilities.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <visualization_msgs/msg/marker.hpp>

// #include "interbotix_xs_msgs/Reboot.h"

using moveit::planning_interface::MoveGroupInterface;

namespace {
// Only can use std string until cpp20
constexpr int kDebugArrowId = 10;

template <class T> std::ostream &operator<<(std::ostream &os, const std::vector<T> &v) {
  os << "[";
  for (typename std::vector<T>::const_iterator ii = v.begin(); ii != v.end(); ++ii) {
    os << " " << *ii;
  }
  os << "]";
  return os;
}

} // namespace


class ListenerNode : public rclcpp::Node {
  public:
  ListenerNode(): Node("listener") {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // Let's try to do some dynamixal register fun
  }
  // This two thing only work if the owning class is a ros node 
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::optional<geometry_msgs::msg::TransformStamped> GetTF(std::string frame_name_1 , std::string frame_name_2){
    try {
      auto map_base_tf = tf_buffer_->lookupTransform(frame_name_1 , frame_name_2, tf2::TimePointZero);
      return map_base_tf;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_STREAM(get_logger(), "Could not transform map to base_link: " << ex.what());
      // TODO ignore it if can't find a good transform
      return std::nullopt;
    }

  }
  
};


class HammerMover {
public:
  HammerMover(rclcpp::Node::SharedPtr node_ptr , std::shared_ptr<ListenerNode> listener_node)
      : node_ptr(node_ptr),
        // Using this kind of params need automatically declare params
        group_name(node_ptr->get_parameter_or<std::string>("group_name", "interbotix_arm")),
        ee_link_name(
            node_ptr->get_parameter_or<std::string>("ee_link_name", "wx200/ee_gripper_link")),
        xz_debug(node_ptr->get_parameter_or<bool>("xz_debug", false)),
        logger(node_ptr->get_logger()),
        use_moveit_execute(node_ptr->get_parameter_or<bool>("use_moveit_execute", false)),
        // I can't use *this when making the interface. So not doing inheriting.

        listener_node_(listener_node),
        move_group_interface(MoveGroupInterface(node_ptr, group_name))

  {

    point_subscriber = node_ptr->create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point", 10, std::bind(&HammerMover::ClickedPointCB, this, std::placeholders::_1));

    debug_marker_pub = node_ptr->create_publisher<visualization_msgs::msg::Marker>("debug", 10);

    // From tutorial, not sure if needed.
    // rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)

    RCLCPP_INFO_STREAM(logger, "use_moveit_execute: " << use_moveit_execute);
    RCLCPP_INFO_STREAM(logger, "Using group: " << group_name);
    RCLCPP_INFO_STREAM(logger, "Planning with ee link: " << ee_link_name);
    RCLCPP_WARN_STREAM(logger, "xz_debug option: " << xz_debug);
    // some simple debug
    RCLCPP_INFO_STREAM(logger,
                       "planing group ee_link_name: " << move_group_interface.getEndEffectorLink());
    RCLCPP_WARN_STREAM(logger, "planing group ee name: " << move_group_interface.getEndEffector());

  };

private:
  // This is actually our main logic here. for now.
  void ClickedPointCB(const geometry_msgs::msg::PointStamped &msg_in) {

    // TODO consider do a user input queue in future.
    RCLCPP_INFO_STREAM(logger, "Received User Command :\n" << geometry_msgs::msg::to_yaml(msg_in));

    // Move the robot to this point.
    

    geometry_msgs::msg::PointStamped msg_transformed = msg_in;
    auto map_base_tf = listener_node_->GetTF("base_link", "odom");
    if (map_base_tf) {
      RCLCPP_INFO_STREAM(logger, "Got tf: " << geometry_msgs::msg::to_yaml(map_base_tf.value()));
      RCLCPP_INFO_STREAM(logger, "Got point: " << geometry_msgs::msg::to_yaml(msg_in));
      
      tf2::doTransform(msg_in, msg_transformed, map_base_tf.value());
      RCLCPP_INFO_STREAM(logger, "Transformed Point: " << geometry_msgs::msg::to_yaml(msg_transformed));

    } else {
      RCLCPP_INFO_STREAM(logger, "No TF, exit early");
      return;
    }

    // Set the xyz target
    geometry_msgs::msg::Pose target_pose;
    target_pose.position = msg_transformed.point;


    //  This needs a complete re-done, specially if Base is not aligning with world frame.
    // need to change


    double z_angle = std::atan2(target_pose.position.y, target_pose.position.x);

    tf2::Quaternion target_q;
    // Here is the magic, the description of setEuler is mis-leading!
    target_q.setRPY(0.0, 0.0, z_angle);
    // Example of a little angle down, Use this for toggle switch.
    // target_q.setRPY(0.0, angle_down, z_angle);

    target_pose.orientation.w = target_q.w();
    target_pose.orientation.x = target_q.x();
    target_pose.orientation.y = target_q.y();
    target_pose.orientation.z = target_q.z();
    // Set the rotation target

    RCLCPP_INFO_STREAM(logger, "Computed target angle: " << z_angle);
    RCLCPP_INFO_STREAM(logger, "Commanded target pose: \n"
                                   << geometry_msgs::msg::to_yaml(target_pose));

    // Publish a arrow to show and debug the quaternion stuff. Very Very useful!

    auto arrow_header = msg_transformed.header;
    arrow_header.stamp = node_ptr->get_clock()->now();
    PublishArrow(arrow_header, target_pose, 0.4, kDebugArrowId);

    // geometry_msgs::msg::Pose actual_planned_pose = target_pose;
    geometry_msgs::msg::PoseStamped actual_planned_pose;
    actual_planned_pose.pose = target_pose;
    actual_planned_pose.header = msg_transformed.header;

    // We assume push mode is just slamming into it.

    // if (!push_mode) {

      // Now we calculate a reduced target.
      // Assume all switches are mounted on vertical wall.
      // Cheating by moving back in xy direction, instead of actually finding
      // the surface orientation and back up.

      auto retracted_target_pose = target_pose;
      // double hammer_rad = 8.0 / 1000; // radius of round hammer head
                                      // Hide variable name
        double retract_amount = 20/1000;
      {
        double xy_mag =
            std::sqrt(retracted_target_pose.position.x * retracted_target_pose.position.x +
                      retracted_target_pose.position.y * retracted_target_pose.position.y);

        // Find xy direction unit vector
        double unit_x = retracted_target_pose.position.x / xy_mag;
        double unit_y = retracted_target_pose.position.y / xy_mag;
        RCLCPP_INFO_STREAM(logger, "mag " << xy_mag);
        RCLCPP_INFO_STREAM(logger, "unit_x " << unit_x);
        RCLCPP_INFO_STREAM(logger, "unit_y " << unit_y);

        // Move the target xy back by this much.
        RCLCPP_INFO_STREAM(logger, "unit_x reduced " << unit_x * retract_amount);
        retracted_target_pose.position.x -= unit_x * retract_amount;
        retracted_target_pose.position.y -= unit_y * retract_amount;
      }
      // Publish a new arrow head, 1 radius of hammer head away. So hammer
      // surface is just touching the objec.
      PublishArrow(arrow_header, retracted_target_pose, 0.95, kDebugArrowId + 1);
      // actual_planned_pose.pose = retracted_target_pose;

    // } else {
    //   RCLCPP_INFO_STREAM(logger, "Planning directly to target pose: \n"
    //                                  << geometry_msgs::msg::to_yaml(actual_planned_pose));
    // }

    // Judging from the arg, If it's taking just a pose it is likely to plan against wx200/base_link
    // We should use the overloaded posestamp.
      RCLCPP_INFO_STREAM(logger, "Planning (retraced) target pose: \n"
                                     << geometry_msgs::msg::to_yaml(retracted_target_pose));


    move_group_interface.setPoseTarget(retracted_target_pose, ee_link_name);

    if (!PlanAndMove()) {
      // If failed, skip the rest.
      RCLCPP_ERROR(logger, "Planning to retracted failed!, aborting everything else");
      return;
    }

      RCLCPP_ERROR(logger, "Planning to actual goal now");

    rclcpp::sleep_for(std::chrono::seconds{1});

    move_group_interface.setPoseTarget(actual_planned_pose, ee_link_name);

    // move_group_interface.setMaxVelocityScalingFactor(1);
    move_group_interface.setMaxAccelerationScalingFactor(1);
    if (!PlanAndMove()) {
      // If failed, skip the rest.
      RCLCPP_ERROR(logger, "Planning failed!, aborting everything else");
      return;
    }
    // move_group_interface.setGoalOrientationTolerance(1.0);
    // Create a plan to that target pose
    // Second part of the motion, Push the button or not
    // TODO we cheat on this for now.

    rclcpp::sleep_for(std::chrono::seconds{1});

    RCLCPP_INFO(logger, "Planning back to sleep");

    // From config file:
    // joint_order: [waist, shoulder, elbow, wrist_angle, wrist_rotate, gripper]
    // sleep_positions: [0, -1.88, 1.5, 0.8, 0, 0]

    // Go back to home pose
    const std::map<std::string, double> sleep_joint_map{
        {"waist", 0},         {"shoulder", -1.88}, {"elbow", 1.5},
        {"wrist_angle", 0.8}, {"wrist_rotate", 0},
    };
    move_group_interface.setJointValueTarget(sleep_joint_map);

    PlanAndMove();
  }

  bool PlanAndMove() {
    moveit::planning_interface::MoveGroupInterface::Plan plan_msg;
    auto const plan_success = static_cast<bool>(move_group_interface.plan(plan_msg));

    // Execute the plan
    if (!plan_success) {
      return false;
    }
    RCLCPP_INFO(logger, "======== Planning SUCCEED !!!!! !");

    // This is the name list

    // Note the difference between humble and iron !
    // Iron use plan_msg.trajectory, humble use plan_msg.trajectory_

    std::vector<std::string> j_names = plan_msg.trajectory_.joint_trajectory.joint_names;

    // This give the last joint trajectory_ point object.
    std::vector<double> final_js = plan_msg.trajectory_.joint_trajectory.points.back().positions;
    std::vector<double> start_js = plan_msg.trajectory_.joint_trajectory.points.front().positions;

    RCLCPP_INFO_STREAM(logger, "js_name" << j_names);
    RCLCPP_INFO_STREAM(logger, "Starting js " << start_js);
    RCLCPP_INFO_STREAM(logger, "Ending js " << final_js);

    // Let's step through the commands ourself
    // Find the cmd intervel

    std::chrono::nanoseconds last_time{0};
    auto start_time = std::chrono::steady_clock::now();
    for (auto cmd_point : plan_msg.trajectory_.joint_trajectory.points) {
      std::chrono::nanoseconds new_time{
          uint64_t(cmd_point.time_from_start.nanosec + cmd_point.time_from_start.sec * 1e9)};

      // RCLCPP_WARN_STREAM(logger,
      //                    "Sleep " << (new_time - last_time).count() / 1e6 << "ms before
      //                    sending");
      rclcpp::sleep_for(new_time - last_time);
      last_time = new_time; 
    }

      auto move_dur = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - start_time);
      RCLCPP_INFO_STREAM(logger, "Move completed after " << move_dur.count() << " ms");
      return true;
  }

  void PublishArrow(std_msgs::msg::Header header, geometry_msgs::msg::Pose arrow_pose,
                    double alpha = 0.5, int id = 1) {
    visualization_msgs::msg::Marker marker;
    marker.type = marker.ARROW;
    marker.header = header;
    marker.action = 0;
    marker.id = id;
    marker.pose = arrow_pose;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = alpha;
    marker.scale.x = 0.05;
    marker.scale.y = 0.007;
    marker.scale.z = 0.007;

    debug_marker_pub->publish(marker);
  }

  // ##########################

  // Member variables
  rclcpp::Node::SharedPtr node_ptr;
  std::string group_name;
  std::string ee_link_name;
  bool xz_debug;
  rclcpp::Logger logger;
  // when false, will use interbotix JointGroupCommand
  bool use_moveit_execute = false;

  // Ros interface objects
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_subscriber;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr debug_marker_pub;

  // std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  // std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Move group interface
  std::shared_ptr<ListenerNode> listener_node_;
  moveit::planning_interface::MoveGroupInterface move_group_interface;
};

// ##########################
// Main
// ##########################

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  auto listen_node = std::make_shared<ListenerNode>();

  auto const node = std::make_shared<rclcpp::Node>(
      "hammer_to_point",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto const hammer_mover = std::make_shared<HammerMover>(node , listen_node);


  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(listen_node);

  executor.spin();
  // rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}