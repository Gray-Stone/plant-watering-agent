#include <chrono>
#include <memory>

#include <fmt/format.h>

#include <cmath>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/robot_state/conversions.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/utils/moveit_error_code.h>
#include <moveit_msgs/moveit_msgs/msg/planning_scene_world.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/bounding_volume.hpp>
#include <moveit_msgs/msg/position_ik_request.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>

#include <moveit_msgs/action/move_group.hpp>

#include <shape_msgs/msg/solid_primitive.hpp>

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/octomap_msgs/msg/octomap_with_pose.hpp>
#include <rclcpp/utilities.hpp>
#include <std_msgs/msg/header.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <std_srvs/srv/trigger.hpp>
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
#include <sensor_msgs/msg/multi_dof_joint_state.hpp>

namespace moveitmsg = moveit_msgs::msg;
namespace moveitaction = moveit_msgs::action;
namespace moveitsrv = moveit_msgs::srv;
namespace shapemsg = shape_msgs::msg;



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
                                               double alpha = 0.9, int id = 90) {
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
  marker.scale.x = 0.15;
  marker.scale.y = 0.09;
  marker.scale.z = 0.02;
  return marker;
}

geometry_msgs::msg::PoseStamped RetractPoseInXYPlane(const geometry_msgs::msg::PoseStamped& in , float retract_amount ){
    auto out = in;
    float x = in.pose.position.x;
    float y = in.pose.position.y;
    float magnitude = std::sqrt( x*x + y*y );
    float new_mag = magnitude - retract_amount;
    out.pose.position.x = x / magnitude * new_mag;
    out.pose.position.y = y / magnitude * new_mag;
    return out;
  }


} // namespace

class ListenerNode : public rclcpp::Node {
public:
  ListenerNode() : Node("listener") {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // Let's try to do some dynamixal register fun
  }
  // This two thing only work if the owning class is a ros node
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
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

  std::optional<geometry_msgs::msg::PoseStamped>

  // Similar as above, but not in the robot's frame. but world frame.
  PoseOriantFromTargetFrame(const geometry_msgs::msg::PointStamped &input_point,
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
      out_pose.header = input_point.header;

      out_pose.pose.position = input_point.point;

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

    z_threshold_ = declare_parameter("z_threshold", 0.12);
    // get_parameter_or<double>("group_name", "mobile_base_arm");

    // I think leave this subscribing shouldn't hurt too much of performance.
    octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
        "octomap_binary", 2, std::bind(&OctomapRelay::OctomapCB, this, std::placeholders::_1));
    planning_scene_w_pub_ =
        create_publisher<moveit_msgs::msg::PlanningSceneWorld>("planning_scene_world", 2);
  }

  void SetPubNum(uint8_t num) {

    num_to_publish_ = num;
    RCLCPP_INFO_STREAM(
        get_logger(),
        fmt::format("About to publish planning scene world for {} times", num_to_publish_));
  }

  void OctomapCB(const octomap_msgs::msg::Octomap &msg) {
    if (num_to_publish_ == 0) {
      // Skip publishing
      return;
    }
    auto octree_ptr = std::shared_ptr<octomap::OcTree>(
        static_cast<octomap::OcTree *>(octomap_msgs::binaryMsgToMap(msg)));
    size_t counter = 0;
    float min_clamp_log = octree_ptr->getClampingThresMinLog();
    for (auto it = octree_ptr->begin(), end = octree_ptr->end(); it != end; ++it) {
      // Get the z-coordinate of the node's center
      double z = it.getZ();

      // Check if the z-coordinate is below the threshold
      if (z < z_threshold_) {
        counter++;
        // Mark the node as free space (can also remove the node)
        it->setLogOdds(min_clamp_log);
      }
    }
    octree_ptr->updateInnerOccupancy();
    octree_ptr->prune();

    RCLCPP_ERROR_STREAM(get_logger(), fmt::format(" {} Point below z threshold {} set to free",
                                                  counter, z_threshold_));

    moveit_msgs::msg::PlanningSceneWorld planning_scene_world_msg;

    octomap_msgs::msg::Octomap output_msg;
    octomap_msgs::binaryMapToMsg<octomap::OcTree>(*octree_ptr,
                                                  planning_scene_world_msg.octomap.octomap);
    planning_scene_world_msg.octomap.header.frame_id = "map";
    planning_scene_world_msg.octomap.octomap.id = "OcTree";
    planning_scene_w_pub_->publish(planning_scene_world_msg);

    RCLCPP_INFO(get_logger(), "Sent octomap msg");
    num_to_publish_--;
  }

  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningSceneWorld>::SharedPtr planning_scene_w_pub_;
  // On start we don't publish. Only start publishing if requested by others
  // for debug init to 2
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
  const uint8_t OCTO_MAP_UPDATE_TIMES = 2;

  bool xz_debug;
  bool push_mode = false;
  rclcpp::Logger logger;
  bool use_moveit_execute = false;
  std::string hardware_type;
  // Ros interface objects

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_subscriber;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr debug_marker_pub;
  rclcpp::Publisher<moveit_msgs::msg::MotionPlanRequest>::SharedPtr plan_req_pub;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr switch_traj_mode_client_;
  rclcpp::Client<moveitsrv::GetPositionIK>::SharedPtr ik_client;
  rclcpp_action::Client<moveitaction::MoveGroup>::SharedPtr move_group_action_client;


  // std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  // std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Move group interface
  moveit::planning_interface::MoveGroupInterface move_group_interface;
  std::shared_ptr<ListenerNode> listener_node_;
  std::shared_ptr<OctomapRelay> octo_relay_node_;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::CallbackGroup::SharedPtr cmd_recv_cb_group;
  rclcpp::CallbackGroup::SharedPtr cmd_send_cb_group;

public:
  ArmCmder(rclcpp::Node::SharedPtr node_ptr, std::shared_ptr<ListenerNode> listener_node,
           std::shared_ptr<OctomapRelay> octo_relay_node)
      : node_ptr(node_ptr),
        group_name(node_ptr->get_parameter_or<std::string>("group_name", "mobile_base_arm")),
        ee_link_name(node_ptr->get_parameter_or<std::string>("ee_link_name", "link_grasp_center")),
        world_frame(node_ptr->get_parameter_or<std::string>("world_frame", "map")),
        robot_base_frame(node_ptr->get_parameter_or<std::string>("robot_base_frame", "base_link")),
        logger(node_ptr->get_logger()),
        move_group_interface(MoveGroupInterface(node_ptr, group_name,listener_node->tf_buffer_)),
        listener_node_(listener_node), octo_relay_node_(octo_relay_node) {

    // Just start, it can't receive in init since not spinning
    move_group_interface.startStateMonitor(0.001);

    auto multiDOF = move_group_interface.getRobotModel()->getMultiDOFJointModels();
    auto rm = move_group_interface.getRobotModel();
    RCLCPP_INFO_STREAM(logger , fmt::format("multiDOF vector length {}" , multiDOF.size()));
    RCLCPP_INFO_STREAM(logger , fmt::format("root joint name {}" , rm->getRootJoint()->getName() ) );


    cmd_recv_cb_group =node_ptr->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cmd_send_cb_group =node_ptr->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    switch_traj_mode_client_ = node_ptr->create_client<std_srvs::srv::Trigger>(
        "switch_to_trajectory_mode", rmw_qos_profile_services_default, cmd_recv_cb_group);

    ik_client = node_ptr->create_client<moveitsrv::GetPositionIK>(
        "compute_ik", rmw_qos_profile_services_default, cmd_send_cb_group);

    debug_marker_pub =
        node_ptr->create_publisher<visualization_msgs::msg::Marker>("VisualizationMarker", 10);
    plan_req_pub =
        node_ptr->create_publisher<moveit_msgs::msg::MotionPlanRequest>("/motion_plan_request", 10);

    move_group_action_client = rclcpp_action::create_client<moveitaction::MoveGroup>(
        node_ptr, "/move_action", cmd_send_cb_group);

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = cmd_recv_cb_group;
    
    // Object that will make us receive stuff.
    point_subscriber = node_ptr->create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point", 10, std::bind(&ArmCmder::CmdCb, this, std::placeholders::_1) , sub_option);


    auto param = move_group_interface.getPlannerParams("geometric::RRTConnect");
    RCLCPP_INFO_STREAM(logger, fmt::format("planner id {}", move_group_interface.getPlannerId()));
    for (auto const &[key, val] : param) {
      RCLCPP_INFO_STREAM(logger, fmt::format("key {} | value {}", key, val));
    }

  };



  moveitmsg::MotionPlanRequest BuildReqConstrains(geometry_msgs::msg::PoseStamped t_pos , std::string link_name){
    moveit_msgs::msg::MotionPlanRequest plan_req;
    moveitmsg::Constraints constraints;

    moveitmsg::OrientationConstraint ori_constraint;
    ori_constraint.orientation = t_pos.pose.orientation;
    ori_constraint.link_name = link_name;
    ori_constraint.header = t_pos.header;
    
    moveitmsg::PositionConstraint pos_constraint;
    pos_constraint.header = t_pos.header;
    pos_constraint.link_name = link_name;

    shapemsg::SolidPrimitive pos_sphere;
    pos_sphere.type = pos_sphere.SPHERE;
    // TODO make this value smaller
    pos_sphere.dimensions.push_back(0.01);
    pos_constraint.constraint_region.primitives.push_back(pos_sphere);
    pos_constraint.constraint_region.primitive_poses.push_back(t_pos.pose);
    constraints.position_constraints.push_back(pos_constraint);
    constraints.orientation_constraints.push_back(ori_constraint);

    plan_req.goal_constraints.push_back(constraints);

    return plan_req;
  }

  void CmdCb(const geometry_msgs::msg::PointStamped &in_msg) {

    RCLCPP_INFO_STREAM(logger, "Got Command toward " << geometry_msgs::msg::to_yaml(in_msg));
    // First update moveit with the planning scene.

    // octo_relay_node_->SetPubNum(OCTO_MAP_UPDATE_TIMES);


    RCLCPP_INFO(logger, "Formatting input data");
    // Convert it into robot frame.
    std::optional<geometry_msgs::msg::PoseStamped> maybe_target_pose;
    size_t retries = 3;
    while (maybe_target_pose == std::nullopt){
      // maybe_target_pose = listener_node_->PoseIntoPoint(in_msg, robot_base_frame);
      maybe_target_pose = listener_node_->PoseOriantFromTargetFrame(in_msg, robot_base_frame);
      retries -=1;
      if (retries == 0){
        RCLCPP_ERROR_STREAM(logger,
                            fmt::format("Failed to Convert point into frame {} ", world_frame));
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds{100});
    }

    auto target_pose = maybe_target_pose.value();


    RCLCPP_INFO(logger, "Updating moveit scene with octomap");
    // Then wait for the 3 publishes
    while (octo_relay_node_->num_to_publish_ != 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds{500});
    }

    // To prevent the sending octomap update took too long, I'll cheat the target's time stamp to now 
    // target_pose.header.stamp = node_ptr->now();


    auto arrow_marker = MakeDebugArrow(target_pose);
    debug_marker_pub->publish(arrow_marker);

    // Scale the pose back a little
    auto retraced_pose = RetractPoseInXYPlane(target_pose , 0.2);
    retraced_pose.pose.position.z += 0.05 ; 
    auto retractged_arrow = MakeDebugArrow(retraced_pose , 0.9, 57);
    debug_marker_pub->publish(retractged_arrow);

    RCLCPP_INFO(logger, "Planning!");

    auto current_pose = move_group_interface.getCurrentPose(ee_link_name);
    RCLCPP_INFO_STREAM(logger, fmt::format("Current ee pose \n {} \n Target pose \n {}" ,
    geometry_msgs::msg::to_yaml(current_pose) , geometry_msgs::msg::to_yaml(target_pose) ));

    // auto plan_req = BuildReqConstrains(target_pose , ee_link_name);
    // plan_req.group_name = group_name;

    // // plan_req_pub->publish(plan_req);

    // moveitaction::MoveGroup_Goal move_group_goal ;
    // move_group_goal.request = plan_req;
    // move_group_goal.planning_options.plan_only = true;


    // auto action_sent_future = move_group_action_client->async_send_goal(move_group_goal);
    // RCLCPP_INFO(logger,"action sent");
    // auto goal_handle = action_sent_future.get();


    // if(!goal_handle){
    //   RCLCPP_INFO(logger,"action rejected");
    //   return;
    // }

    // RCLCPP_INFO(logger,"action accepted");

    // auto result_future = move_group_action_client->async_get_result(goal_handle);
    // auto wrapped_result = result_future.get();
    // switch (wrapped_result.code) {
    // case rclcpp_action::ResultCode::SUCCEEDED:
    //   break;
    // case rclcpp_action::ResultCode::ABORTED:
    //   RCLCPP_ERROR(logger, "Goal was aborted");
    //   return;
    // case rclcpp_action::ResultCode::CANCELED:
    //   RCLCPP_ERROR(logger, "Goal was canceled");
    //   return;
    // default:
    //   RCLCPP_ERROR(logger, "Unknown result code");
    //   return;
    // }

    // auto planned_traj = wrapped_result.result->planned_trajectory;
    // RCLCPP_INFO_STREAM(logger, fmt::format("planning time {}" ,wrapped_result.result->planning_time ));


    // return;


    // move_group_interface.setApproximateJointValueTarget(retraced_pose, ee_link_name);

    // sensor_msgs::msg::MultiDOFJointState multi_dof_goal;
    // multi_dof_goal.joint_names.push_back("position");
    // geometry_msgs::msg::Transform base_pose;
    // base_pose.translation.x = 2.0;
    // base_pose.rotation.w = 1.0;
    // multi_dof_goal.transforms.push_back(base_pose);

    // Can confirm, using joint value target does work!. 
    // move_group_interface.setJointValueTarget("joint_lift" , 0.2);

    
    auto get_ik_req = std::make_shared<moveitsrv::GetPositionIK_Request>() ;

    auto current_state = move_group_interface.getCurrentState();
    moveit::core::robotStateToRobotStateMsg(*current_state,get_ik_req->ik_request.robot_state );


    get_ik_req->ik_request.group_name = group_name;
    get_ik_req->ik_request.pose_stamped = target_pose;
    get_ik_req->ik_request.ik_link_name = ee_link_name;
    get_ik_req->ik_request.timeout.sec = 5;

    auto ik_future = ik_client->async_send_request(get_ik_req);

    auto ik_resp  = ik_future.get();
    
    RCLCPP_INFO_STREAM(logger, fmt::format("compute ik error code {}" , moveitmsg::to_yaml(ik_resp->error_code)));

    RCLCPP_INFO_STREAM(logger, fmt::format("compute ik robot state {}" , moveitmsg::to_yaml(ik_resp->solution)));
    
    return ;

    // move_group_interface.setNumPlanningAttempts(5);
    // move_group_interface.setPlanningTime(50.0);

    // move_group_interface.setPoseTarget(retraced_pose, ee_link_name);
    // move_group_interface.setApproximateJointValueTarget(retraced_pose, ee_link_name);

    

    auto param = move_group_interface.getPlannerParams(move_group_interface.getPlannerId());
    RCLCPP_INFO_STREAM(logger, fmt::format("planner id {}" , move_group_interface.getPlannerId()));
    for (auto const& [key, val]:param){
      RCLCPP_INFO_STREAM(logger, fmt::format("key {} | value {}" , key,val ));
    }

    // This planner id is copied from terminal log
    // move_group_interface.setPlannerId("geometric::RRTConnect");
    move_group_interface.setGoalPositionTolerance(0.05);
    move_group_interface.setGoalOrientationTolerance(0.05);

    move_group_interface.setPoseTarget(target_pose , ee_link_name);
    // move_group_interface.setJointValueTarget(target_pose , ee_link_name);

    PlanAndMove();
    
    
        param = move_group_interface.getPlannerParams(move_group_interface.getPlannerId());
    RCLCPP_INFO_STREAM(logger, fmt::format("planner id {}" , move_group_interface.getPlannerId()));
    for (auto const& [key, val]:param){
      RCLCPP_INFO_STREAM(logger, fmt::format("key {} | value {}" , key,val ));
    }
  }

  bool PlanAndMove() {
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
  // plan_obj.trajectory_.multi_dof_joint_trajectory.

    RCLCPP_INFO_STREAM(logger, "js_name" << j_names);
    RCLCPP_INFO_STREAM(logger, "Starting js " << start_js);
    RCLCPP_INFO_STREAM(logger, "Ending js " << final_js);
    RCLCPP_INFO_STREAM(logger, fmt::format("point length {}" , plan_obj.trajectory_.joint_trajectory.points.size()));


    return true;
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
      "arm_cmder", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto const arm_cmder =
      std::make_shared<ArmCmder>(arm_cmder_node, listen_node, octo_relay_node);

  rclcpp::executors::MultiThreadedExecutor executor1;
  rclcpp::executors::MultiThreadedExecutor executor2;
  executor2.add_node(arm_cmder_node);
  auto executor2_thread_ = std::thread([&]() { executor2.spin(); });
  auto executor3_thread_ = std::thread([&]() {   geometry_msgs::msg::PointStamped p;
    p.point.x = 0.4;
    p.point.y = 0.2;
    p.point.z = 0.6;
    p.header.frame_id = "odom";
    arm_cmder->CmdCb(p);
   });

  executor1.add_node(octo_relay_node);
  executor1.add_node(listen_node);

  executor1.spin();

  rclcpp::shutdown();
  executor2_thread_.join();
  return 0;
}
