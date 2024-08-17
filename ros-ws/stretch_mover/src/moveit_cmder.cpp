#include <chrono>
#include <memory>

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

  std::optional<geometry_msgs::msg::TransformStamped> GetTF(const std::string & target_frame, const std::string & source_frame ,const rclcpp::Time & time = rclcpp::Time() ){
    try {
      auto map_base_tf = tf_buffer_->lookupTransform(target_frame, source_frame, time);
      
      return map_base_tf;

    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_STREAM(get_logger(), "Could not transform : " << source_frame << "to" << target_frame <<" \n" << ex.what());
      // TODO ignore it if can't find a good transform
      return std::nullopt;
    }
  }
  
};

class OctomapRelay : public rclcpp::Node{
    public:
    OctomapRelay(): Node("OctomapRelay") {

      octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
          "octomap_binary", 2, std::bind(&OctomapRelay::OctomapCB, this, std::placeholders::_1));
      planning_scene_w_pub_ = create_publisher<moveit_msgs::msg::PlanningSceneWorld>("planning_scene_world", 2);

    }

    void OctomapCB(const octomap_msgs::msg::Octomap &msg) {
      RCLCPP_ERROR(get_logger() , "Got msg");
      
      auto octree_ptr = std::shared_ptr<octomap::OcTree>(
          static_cast<octomap::OcTree *>(octomap_msgs::binaryMsgToMap(msg)));

      RCLCPP_ERROR(get_logger() , "casted into octree obj");

      moveit_msgs::msg::PlanningSceneWorld planning_scene_world_msg;
      
      octomap_msgs::msg::Octomap output_msg;
      octomap_msgs::binaryMapToMsg<octomap::OcTree>(*octree_ptr, planning_scene_world_msg.octomap.octomap);
      RCLCPP_ERROR(get_logger() , "casted into octree message");
      planning_scene_world_msg.octomap.header.frame_id = "map";
      planning_scene_world_msg.octomap.octomap.id = "OcTree";
      RCLCPP_ERROR(get_logger() , "sending it");
      planning_scene_w_pub_->publish(planning_scene_world_msg);
      RCLCPP_ERROR(get_logger() , "sent");

      
    }

    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
    rclcpp::Publisher<moveit_msgs::msg::PlanningSceneWorld>::SharedPtr planning_scene_w_pub_;
};

class ArmCmder {

  // Member variables
  rclcpp::Node::SharedPtr node_ptr;
  std::string group_name;
  std::string ee_link_name;
  bool xz_debug;
  bool push_mode = false;
  rclcpp::Logger logger;
  bool use_moveit_execute = false;
  std::string hardware_type;

  // Ros interface objects
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_subscriber;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr debug_marker_pub;
  //   rclcpp::Client<interbotix_xs_msgs::srv::RegisterValues>::SharedPtr get_motor_reg_cli;
  //   rclcpp::Client<interbotix_xs_msgs::srv::RegisterValues>::SharedPtr set_motor_reg_cli;
  //   rclcpp::Client<interbotix_xs_msgs::srv::MotorGains>::SharedPtr set_motor_pid_cli;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr switch_traj_mode_client_;

  // std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  // std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Move group interface
  // moveit::planning_interface::MoveGroupInterface move_group_interface;
  std::shared_ptr<ListenerNode> listener_node_;

  rclcpp::TimerBase::SharedPtr timer_;

rclcpp::CallbackGroup::SharedPtr timer_cb_group;
public:
  ArmCmder(rclcpp::Node::SharedPtr node_ptr, std::shared_ptr<ListenerNode> listener_node)
      : node_ptr(node_ptr),
        group_name(node_ptr->get_parameter_or<std::string>("group_name", "mobile_base_arm")),
        ee_link_name(node_ptr->get_parameter_or<std::string>("ee_link_name", "link_grasp_center")),
        logger(node_ptr->get_logger()),
        // move_group_interface(MoveGroupInterface(node_ptr, group_name)),
        listener_node_(listener_node)
  {

    timer_cb_group = node_ptr->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    switch_traj_mode_client_ = node_ptr->create_client<std_srvs::srv::Trigger>("switch_to_trajectory_mode");
    // timer_ = node_ptr->create_wall_timer(std::chrono::seconds{1} , std::bind(&ArmCmder::TimerCb , this) , timer_cb_group);
  };

  void TimerCb(){
    RCLCPP_ERROR(logger ,"Start timer");
    std_srvs::srv::Trigger_Request::SharedPtr req = std::make_shared<std_srvs::srv::Trigger_Request>();
    auto future = switch_traj_mode_client_->async_send_request(req);
    RCLCPP_ERROR(logger ,"req async sent");
    auto res = future.get();
    RCLCPP_ERROR_STREAM(logger,"got response" << res->success << " msg " << res->message   );


  }

};


int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  
  auto listen_node = std::make_shared<ListenerNode>();

  auto const arm_cmder_node = std::make_shared<rclcpp::Node>(
      "arm_cmder",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto const hammer_mover = std::make_shared<ArmCmder>(arm_cmder_node , listen_node);

  auto octo_relay_node = std::make_shared<OctomapRelay>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(arm_cmder_node);
  executor.add_node(octo_relay_node);
  executor.add_node(listen_node);

  executor.spin();

  // rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
