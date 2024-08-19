#include <chrono>
#include <memory>

#include <fmt/format.h>

#include <rclcpp/client.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>
#include <cmath>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/moveit_msgs/msg/planning_scene_world.hpp>
#include <octomap_msgs/octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/octomap_msgs/msg/octomap_with_pose.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <rclcpp/utilities.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_srvs/srv/trigger.hpp>
#include<moveit/utils/moveit_error_code.h>
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



  visualization_msgs::msg::Marker MakeDebugArrow(geometry_msgs::msg::PoseStamped arrow_pose,
                    double alpha = 0.5, int id = 1) {
    visualization_msgs::msg::Marker marker;
    marker.type = marker.ARROW;
    marker.header = arrow_pose.header;
    marker.action = 0;
    marker.id = id;
    marker.pose = arrow_pose.pose;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = alpha;
    marker.scale.x = 0.05;
    marker.scale.y = 0.007;
    marker.scale.z = 0.007;
    return marker;
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

  std::optional<geometry_msgs::msg::PoseStamped>
  PoseIntoPoint(const geometry_msgs::msg::PointStamped &input_point,
                const std::string &target_frame) {
    try {
      geometry_msgs::msg::PointStamped new_point = tf_buffer_->transform(input_point, target_frame);
      double z_angle = std::atan2(new_point.point.y, new_point.point.x);

      tf2::Quaternion target_q;
      // Here is the magic, the description of setEuler is mis-leading!
      target_q.setRPY(0.0, 0.0, z_angle);
      // Example of a little angle down, Use this for toggle switch.
      // target_q.setRPY(0.0, angle_down, z_angle);
      geometry_msgs::msg::PoseStamped out_pose;
      out_pose.header = new_point.header;
      out_pose.pose.position = new_point.point;

      out_pose.pose.orientation.w = target_q.w();
      out_pose.pose.orientation.x = target_q.x();
      out_pose.pose.orientation.y = target_q.y();
      out_pose.pose.orientation.z = target_q.z();

      return out_pose;

    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_STREAM(get_logger(), "Could not transform : " << input_point.header.frame_id
                                                                << "to" << target_frame << " \n"
                                                                << ex.what());
      // TODO ignore it if can't find a good transform
      return std::nullopt;
    }
  }

  std::optional<geometry_msgs::msg::PointStamped>
  TransPoint(const geometry_msgs::msg::PointStamped &input_point, const std::string &target_frame) {
    try {

      return tf_buffer_->transform(input_point, target_frame);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_STREAM(get_logger(), "Could not transform point from "
                                           << input_point.header.frame_id << "to" << target_frame
                                           << " \n"
                                           << ex.what());
      // TODO ignore it if can't find a good transform
      return std::nullopt;
    }
  }

  std::optional<geometry_msgs::msg::TransformStamped>
  GetTF(const std::string &target_frame, const std::string &source_frame,
        const rclcpp::Time &time = rclcpp::Time()) {
    try {
      auto map_base_tf = tf_buffer_->lookupTransform(target_frame, source_frame, time);

      return map_base_tf;

    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_STREAM(get_logger(), "Could not transform : " << source_frame << "to"
                                                                << target_frame << " \n"
                                                                << ex.what());
      // TODO ignore it if can't find a good transform
      return std::nullopt;
    }
  }
};

class OctomapRelay : public rclcpp::Node {
public:
  OctomapRelay() : Node("OctomapRelay") {

    z_threshold_ = declare_parameter("z_threshold",0.05 );
    // get_parameter_or<double>("group_name", "mobile_base_arm");

    // I think leave this subscribing shouldn't hurt too much of performance.
    octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
        "octomap_binary", 2, std::bind(&OctomapRelay::OctomapCB, this, std::placeholders::_1));
    planning_scene_w_pub_ =
        create_publisher<moveit_msgs::msg::PlanningSceneWorld>("planning_scene_world", 2);
  }

  void SetPubNum(uint8_t num){

    num_to_publish_ = num;
    RCLCPP_INFO_STREAM(
        get_logger(),
        fmt::format("About to publish planning scene world for {} times", num_to_publish_));
  }

  void OctomapCB(const octomap_msgs::msg::Octomap &msg) {
    if (num_to_publish_ == 0){
      // Skip publishing
      return;
    }
    auto octree_ptr = std::shared_ptr<octomap::OcTree>(
        static_cast<octomap::OcTree *>(octomap_msgs::binaryMsgToMap(msg)));
    size_t counter =0;
    for (auto it = octree_ptr->begin(), end = octree_ptr->end(); it != end; ++it) {
        // Get the z-coordinate of the node's center
        double z = it.getZ();

        // Check if the z-coordinate is below the threshold
        if (z < 0.8) {
          counter ++; 
            // Mark the node as free space (can also remove the node)
          octree_ptr->updateNode(it.getKey(), false, true); // false marks it as free
          
        }
    }
    octree_ptr->updateInnerOccupancy();
    octree_ptr->prune();
    RCLCPP_ERROR_STREAM(get_logger(),fmt::format(" {} Point removed below z threshold" , counter));
    // Optionally prune the octree to remove unnecessary nodes



    moveit_msgs::msg::PlanningSceneWorld planning_scene_world_msg;

    octomap_msgs::msg::Octomap output_msg;
    octomap_msgs::binaryMapToMsg<octomap::OcTree>(*octree_ptr,
                                                  planning_scene_world_msg.octomap.octomap);
    planning_scene_world_msg.octomap.header.frame_id = "map";
    planning_scene_world_msg.octomap.octomap.id = "OcTree";
    planning_scene_w_pub_->publish(planning_scene_world_msg);

    RCLCPP_INFO(get_logger(), "Sent octomap msg");
    num_to_publish_ --;

  }

  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningSceneWorld>::SharedPtr planning_scene_w_pub_;
  // On start we don't publish. Only start publishing if requested by others
  std::atomic<uint8_t> num_to_publish_ = 0;
  double z_threshold_;
};

class ArmCmder {

  // Member variables
  rclcpp::Node::SharedPtr node_ptr;
  std::string group_name;
  std::string ee_link_name;
  std::string world_frame;
  std::string robot_base_frame;
  const uint8_t OCTO_MAP_UPDATE_TIMES = 3; 

  bool xz_debug;
  bool push_mode = false;
  rclcpp::Logger logger;
  bool use_moveit_execute = false;
  std::string hardware_type;
  // Ros interface objects

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_subscriber;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr debug_marker_pub;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr switch_traj_mode_client_;

  // std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  // std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Move group interface
  moveit::planning_interface::MoveGroupInterface move_group_interface;
  std::shared_ptr<ListenerNode> listener_node_;
  std::shared_ptr<OctomapRelay> octo_relay_node_;

  rclcpp::TimerBase::SharedPtr timer_;

rclcpp::CallbackGroup::SharedPtr cmd_recv_cb_group;
public:
  ArmCmder(rclcpp::Node::SharedPtr node_ptr, std::shared_ptr<ListenerNode> listener_node,
           std::shared_ptr<OctomapRelay> octo_relay_node)
      : node_ptr(node_ptr),
        group_name(node_ptr->get_parameter_or<std::string>("group_name", "mobile_base_arm")),
        ee_link_name(node_ptr->get_parameter_or<std::string>("ee_link_name", "link_grasp_center")),
        world_frame(node_ptr->get_parameter_or<std::string>("world_frame", "map")),
        robot_base_frame(node_ptr->get_parameter_or<std::string>("robot_base_frame", "base_link")),
        logger(node_ptr->get_logger()),
        move_group_interface(MoveGroupInterface(node_ptr, group_name)),
        listener_node_(listener_node),
        octo_relay_node_(octo_relay_node)
  {
    cmd_recv_cb_group = node_ptr->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    switch_traj_mode_client_ = node_ptr->create_client<std_srvs::srv::Trigger>("switch_to_trajectory_mode");
    debug_marker_pub = node_ptr->create_publisher<visualization_msgs::msg::Marker>("VisualizationMarker", 10);


    // Object that will make us receive stuff.
    point_subscriber = node_ptr->create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point", 10, std::bind(&ArmCmder::CmdCb, this, std::placeholders::_1));
  };

  void CmdCb(const geometry_msgs::msg::PointStamped & in_msg){

    RCLCPP_INFO_STREAM(logger, "Got Command toward " << geometry_msgs::msg::to_yaml(in_msg));

    // First update moveit with the planning scene.
    octo_relay_node_->SetPubNum(OCTO_MAP_UPDATE_TIMES);
    // Then wait for the 3 publishes
    while (octo_relay_node_->num_to_publish_ != 0){
      std::this_thread::sleep_for(std::chrono::microseconds{500});
    }

    RCLCPP_INFO(logger, "Formatting input data");

    // Convert it into robot frame.
    auto maybe_target_pose = listener_node_->PoseIntoPoint(in_msg , robot_base_frame);

    if (maybe_target_pose ==  std::nullopt){
      // Failed! 
      RCLCPP_ERROR_STREAM(logger , fmt::format("Failed to Convert point into frame {} " ,world_frame));
      return;
    }

    auto target_pose = maybe_target_pose.value();
    auto arrow_marker = MakeDebugArrow(target_pose);

    RCLCPP_INFO(logger, "Planning!");
    move_group_interface.setPoseTarget(target_pose, ee_link_name);
    PlanAndMove();


  }

  bool PlanAndMove(){
    moveit::planning_interface::MoveGroupInterface::Plan plan_obj;
    auto const plan_success = static_cast<bool>(move_group_interface.plan(plan_obj));

    // Execute the plan
    if (!plan_success) {
    RCLCPP_INFO(logger, "======== Plan Failed !!!!! !");
      return false;
    }
    RCLCPP_INFO(logger, "======== Planning SUCCEED !!!!! !");

    std::vector<std::string> j_names = plan_obj.trajectory_.joint_trajectory.joint_names;

    // This give the last joint trajectory_ point object.
    std::vector<double> final_js = plan_obj.trajectory_.joint_trajectory.points.back().positions;
    std::vector<double> start_js = plan_obj.trajectory_.joint_trajectory.points.front().positions;

    RCLCPP_INFO_STREAM(logger, "js_name" << j_names);
    RCLCPP_INFO_STREAM(logger, "Starting js " << start_js);
    RCLCPP_INFO_STREAM(logger, "Ending js " << final_js);

    moveit::core::MoveItErrorCode ret_code = move_group_interface.execute(plan_obj);
    RCLCPP_ERROR_STREAM(logger, "Execute return " << moveit::core::error_code_to_string(ret_code));
    return true;
  }



};


int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  
  auto listen_node = std::make_shared<ListenerNode>();
  auto octo_relay_node = std::make_shared<OctomapRelay>();

  auto const arm_cmder_node = std::make_shared<rclcpp::Node>(
      "arm_cmder",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto const hammer_mover = std::make_shared<ArmCmder>(arm_cmder_node , listen_node ,octo_relay_node);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(arm_cmder_node);
  executor.add_node(octo_relay_node);
  executor.add_node(listen_node);

  executor.spin();

  // rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
