#! /usr/bin/env python3
import copy
import dataclasses
import enum
import time
from enum import Enum
from typing import Optional
from action_msgs.msg import GoalStatus
import rclpy.time
import math

import tf_transformations
from rclpy.executors import MultiThreadedExecutor
import numpy as np
import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from builtin_interfaces.msg import Time as RosTime
from geometry_msgs.msg import Point, PointStamped, TransformStamped, Transform, Vector3, PoseStamped, Pose, Twist
from geometry_msgs.msg import Quaternion
from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose
from nav_msgs.msg import MapMetaData, OccupancyGrid

from sensor_msgs.msg import JointState
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
# from sensor_msgs.msg import CameraInfo, Image as ImageMsg
from std_msgs.msg import ColorRGBA, Header
from std_msgs.msg import String as StringMsg
from std_srvs.srv import Trigger as TriggerSrv

from stretch_mover.msg import (KnownObject, KnownObjectList, YoloDetection, YoloDetectionList)
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from collections import deque

from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint, JointTrajectory, JointTrajectoryPoint

from stretch_mover_utils.grid_utils import OccupancyGridHelper, COLOR_MSG_LIST_RGBW
from stretch_mover_utils.marker_helper import MakeTextMarker , MakeCylinderMarker, MakeSphereMaker

from control_msgs.action import FollowJointTrajectory

from threading import Lock

from threading import Event

def get_z_angle_to_point(point: Point):
    return normalize_angle(math.atan2(point.y, point.x))


def point_point_distanec(point1: Point, point2: Point):
    return np.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2 + (point1.z - point2.z)**2)


def sign(num):
    return -1 if num < 0 else 1


def normalize_angle(angle: float) -> float:

    #   return rad - (std::ceil((rad + PI) / (2.0 * PI)) - 1.0) * 2.0 * PI;

    return angle - (math.ceil((angle + math.pi) / (2.0 * math.pi)) - 1.0) * 2.0 * math.pi


def map_marker_check(map_helper: OccupancyGridHelper) -> Marker:
    # Map data: -1 unknown, 0 free, 100 occupied.
    locs = []
    for ix in range(map_helper.map_info.width):
        for iy in range(map_helper.map_info.height):
            loc = (ix, iy)
            if not map_helper.ValidLoc(loc):
                raise ValueError(f"ixy {loc} is out of bound! ")
            locs.append(loc)

    return map_helper.color_sphere_gen(locs, scale=0.03)



class ModeSwitchNode(Node):

    def __init__(self):
        super().__init__("stretch_mode_switching")
        self.service_client_cb_group = ReentrantCallbackGroup()

        self.switch_nav_service = self.create_client(TriggerSrv,
                                                     "switch_to_navigation_mode",
                                                     callback_group=self.service_client_cb_group)
        self.switch_pos_service = self.create_client(TriggerSrv,
                                                     "switch_to_position_mode",
                                                     callback_group=self.service_client_cb_group)
        self.switch_traj_service = self.create_client(TriggerSrv,
                                                      "switch_to_trajectory_mode",
                                                      callback_group=self.service_client_cb_group)
        self.switch_gamepad_service = self.create_client(
            TriggerSrv, "switch_to_gamepad_mode", callback_group=self.service_client_cb_group)


class GoalMover(Node):

    PAUSE_DURATION = 3.0
    LIFT_VELOCITY_MAX = 0.14  # 0.15 from driver

    PLANT_CLASS_ID = 0
    WATERING_AREA_CLASS_ID = 1

    POI_MARKER_ID = 99
    NAV_PLANED_LOC_MARKER_ID = 89
    WAVE_FRONT_MARKER_ID = 34
    WATERED_PLANT_MARKER_ID = 78

    MARKER_NAMESPACE = "state_controller"


    PLAN_GOAL_CLEARANCE_RADIUS = 0.4
    FRONTIER_REACHABLE_RADIUS = 0.35
    MAX_PLAN_OFFSET_RADIUS = 1.1  # arm length is 0.52
    MIN_PLAN_OFFSET_RADIUS = 0.55  # We want to have some distance so arm could extend
    POT_TO_WATERING_DIS_THRESHOLD = 0.24

    TEXT_INFO_ID = 79

    class CmdStates(Enum):
        IDLE = enum.auto()

        # If all prefect, states should only linearly go down.
        EXPLORE_PLANNING = enum.auto()
        EXPLORE = enum.auto()

        PAUSE_BEFORE_NEXT = enum.auto()
        # If any of the following state failed, should jump to pause (instead of plant select for next plant)
        PLANT_SELECTION = enum.auto()
        NAV_PLANNING = enum.auto()
        MOVING = enum.auto()
        WATERING_AREA_FINDING = enum.auto()
        WATERING = enum.auto()
        RETURN_HOME = enum.auto

    def pub_marker(self, m: Marker):
        m.ns = self.MARKER_NAMESPACE
        self.marker_pub.publish(m)

    def __init__(self, mode_switch_node: ModeSwitchNode):
        super().__init__("move_to_plants")

        self.info = self.get_logger().info
        self.warn = self.get_logger().warn
        self.error = self.get_logger().error

        #### Parameters
        self.declare_parameter("pot_class_id", int(1))
        # world frame will also be the map's frame.
        self.declare_parameter("world_frame", 'map')
        self.declare_parameter("robot_baseframe", "base_footprint")
        self.declare_parameter("mast_frame", "link_mast")
        self.declare_parameter("ee_frame", "link_grasp_center")

        self.declare_parameter("known_object_topic", "yolo_ros/known_objects")

        self.pot_class_id = self.get_parameter("pot_class_id").get_parameter_value().integer_value
        self.world_frame = self.get_parameter("world_frame").get_parameter_value().string_value
        self.robot_baseframe = self.get_parameter(
            "robot_baseframe").get_parameter_value().string_value
        self.mast_frame = self.get_parameter("mast_frame").get_parameter_value().string_value
        self.ee_frame = self.get_parameter("ee_frame").get_parameter_value().string_value
        known_object_topic = self.get_parameter(
            "known_object_topic").get_parameter_value().string_value

        #### Member Vars
        self.first_explore = True

        self.map_helper: OccupancyGridHelper = None
        self.known_obj_list: KnownObjectList = None
        self.skipped_pot_objects: deque[KnownObject] = deque()
        self.watered_objects: deque[KnownObject] = deque()
        self.js_map: dict[str, float] = {}

        self.next_planning_object_idx = 0
        self.successful_planned_pose: PoseStamped = None
        self.current_chasing_plant: KnownObject = None
        self.current_watering_obj: KnownObject = None

        self.map_lock = Lock() # Specially when frontier expanding, Lock is necessary.
        self.ext_watering_event = Event()

        # List of skipped pots

        self.state = self.CmdStates.IDLE
        self.state_update(self.CmdStates.EXPLORE_PLANNING)
        # self.state_update(self.CmdStates.PLANT_SELECTION)

        # Time keeper for the pause state.
        self.pause_start_time = None

        #### Publisher
        self.state_change_publisher = self.create_publisher(StringMsg, "move_plant_state_change", 2)
        self.marker_pub = self.create_publisher(Marker, "VisualizationMarker", 1)
        self.cmd_vel_pub = self.create_publisher(Twist, "/stretch/cmd_vel", 1)

        #### Service client
        # /switch_to_navigation_mode [std_srvs/srv/Trigger]
        # /switch_to_position_mode [std_srvs/srv/Trigger]
        # /switch_to_trajectory_mode [std_srvs/srv/Trigger]
        # self.service_client_cb_group = ReentrantCallbackGroup()

        self.switch_nav_service = mode_switch_node.switch_nav_service
        self.switch_pos_service = mode_switch_node.switch_pos_service
        self.switch_traj_service = mode_switch_node.switch_traj_service
        self.switch_gamepad_service = mode_switch_node.switch_gamepad_service

        #### Action client
        action_cb_group = MutuallyExclusiveCallbackGroup()

        self.compute_path_client = ActionClient(self,
                                                ComputePathToPose,
                                                "compute_path_to_pose",
                                                callback_group=action_cb_group)
        self.navigate_to_pose_client = ActionClient(self,
                                                    NavigateToPose,
                                                    "/navigate_to_pose",
                                                    callback_group=action_cb_group)
        self.trajectory_action_client = ActionClient(self,
                                                     FollowJointTrajectory,
                                                     "/stretch_controller/follow_joint_trajectory",
                                                     callback_group=action_cb_group)

        # Subscribers

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.data_update_cb_group = ReentrantCallbackGroup()
        self.map_subs = self.create_subscription(OccupancyGrid,
                                                 "/map",
                                                 self.global_map_cb,
                                                 1,
                                                 callback_group=self.data_update_cb_group)
        # self.map_subs = self.create_subscription(OccupancyGrid, "/local_costmap/costmap" , self.global_map_cb , 1)
        self.known_obj_subs = self.create_subscription(KnownObjectList,
                                                       known_object_topic,
                                                       self.known_obj_cb,
                                                       1,
                                                       callback_group=self.data_update_cb_group)
        self.js_sub = self.create_subscription(JointState,
                                               "joint_states",
                                               self.joint_state_cb,
                                               2,
                                               callback_group=self.data_update_cb_group)

        self.watering_server = self.create_service(TriggerSrv , "watering_trigger" , self.trigger_water)

        ## main timer
        self.create_timer(0.5, self.main_timer)
        self.get_logger().info("Node configured, timer created!")

    def global_map_cb(self, map_msg: OccupancyGrid):
        # print(f"\n\n ================== ")
        # print(f"Got map , header {map_msg.header} , info {map_msg.info}")
        with self.map_lock:
            self.map_helper = OccupancyGridHelper(map_msg)
        # Map data: -1 unknown, 0 free, 100 occupied.
        # For debug purpose.
        # self.pub_marker(map_marker_check(self.map_helper))
    def known_obj_cb(self, msg: KnownObjectList):
        self.known_obj_list = msg

    def joint_state_cb(self, msg: JointState):
        for name, val in zip(msg.name, msg.position):
            self.js_map[name] = val
        # This way traj generationg don't need to do special case for arm
        self.js_map['wrist_extension'] = self.get_wrist_extension_js()

    def state_update(self, new_state: Optional[CmdStates]):
        if new_state is None:
            return
        if new_state == self.state:
            return

        self.get_logger().warn(f" ==== >>>>  State changing from {self.state} to {new_state}")
        self.state = new_state
        return

    def trigger_water(self , request, response:TriggerSrv.Response):
        # This is the one to trigger a watering action

        # Must be in idle state to start this.
        if self.state != self.CmdStates.IDLE:
            response.success = False
            response.message = f"Cannot start watering in {self.state}"
            return response

        self.ext_watering_event.set()
        response.success = True
        response.message = f"Trigger set"
        return response


    """####################### 
    State machines ! 
    ######################### 
    """

    async def main_timer(self):
        # await self.debug_state()

        if self.ext_watering_event.is_set():
            self.clear_marker(self.WATERED_PLANT_MARKER_ID)
            self.clear_marker(self.POI_MARKER_ID)
            self.clear_marker(self.NAV_PLANED_LOC_MARKER_ID)
            self.clear_marker(self.WAVE_FRONT_MARKER_ID)
            self.clear_marker(self.TEXT_INFO_ID)

            # Clears all cached state variables.
            self.next_planning_object_idx = 0
            self.successful_planned_pose = None
            self.current_chasing_plant = None
            self.current_watering_obj = None
            self.skipped_pot_objects.clear()
            self.watered_objects.clear()

            # Jump into the state of watering.
            self.state_update(self.CmdStates.PAUSE_BEFORE_NEXT)
            self.ext_watering_event.clear()


        # This is the main logic.
        if self.state == self.CmdStates.IDLE:
            # Do nothing in idle state.
            # Depends on what user service call is sent, then switch to the correct state.
            return

        elif self.state == self.CmdStates.EXPLORE_PLANNING:
            self.state_update(await self.explore_planning_state())

        elif self.state == self.CmdStates.EXPLORE:
            self.state_update(await self.explore_state())


        elif self.state == self.CmdStates.PAUSE_BEFORE_NEXT:
            self.state_update(await self.pause_before_next_state())

        elif self.state == self.CmdStates.PLANT_SELECTION:
            self.state_update(await self.plant_selection_state())

        elif self.state == self.CmdStates.NAV_PLANNING:
            self.state_update(await self.nav_planning_state())

        elif self.state == self.CmdStates.MOVING:
            self.state_update(await self.moving_state())

        elif self.state == self.CmdStates.WATERING_AREA_FINDING:
            self.state_update(await self.find_water_area_state())

        elif self.state == self.CmdStates.WATERING:
            self.state_update(await self.watering_state())
        elif self.state == self.CmdStates.RETURN_HOME:
            self.state_update(await self.homing_state())

        else:
            self.get_logger().error(f"Stuck in non-existing state: {self.state}")
            raise RuntimeError(f"Stuck in non-existing state: {self.state}")
        # Reached here cuz state is just wrong !

    def make_ComputePathToPose_goal(self, x, y, heading_z):
        pose_goal = ComputePathToPose.Goal()
        pose_goal.goal.header.frame_id = self.world_frame
        pose_goal.goal.header.stamp = self.get_clock().now().to_msg()

        # we should be able to leave stamp empty.
        pose_goal.goal.pose.position.x = x
        pose_goal.goal.pose.position.y = y

        q = tf_transformations.quaternion_from_euler(0, 0, heading_z)
        pose_goal.goal.pose.orientation.x = q[0]
        pose_goal.goal.pose.orientation.y = q[1]
        pose_goal.goal.pose.orientation.z = q[2]
        pose_goal.goal.pose.orientation.w = q[3]
        # Must fill in the planner id, or it won't print the action failed reason.
        pose_goal.planner_id = "GridBased"

        pose_goal.use_start = False
        return pose_goal

    async def ActionSendAwait(self,
                              goal_obj,
                              client: ActionClient,
                              feedback_cb=None) -> tuple[GoalStatus, any]:

        goal_handle: ClientGoalHandle = await client.send_goal_async(goal_obj,
                                                                     feedback_callback=feedback_cb)

        if not goal_handle.accepted:
            raise ValueError("goal rejected!")

        # for example res is a "ComputePathToPose_GetResult_Response" object.
        res = await goal_handle.get_result_async()
        # self.get_logger().warn(f"type of res is {type(res)}, itself is \n{res}")

        # if res.status != GoalStatus.STATUS_SUCCEEDED:
        return res.status, res.result

    def NavStartupMap(self) -> bool:
        if self.map_helper is None:
            self.get_logger().warn(f"Skipping cycle for missing map")
            return False
        if ("joint_lift" not in self.js_map) or ("joint_wrist_yaw" not in self.js_map):
            self.get_logger().warn(f"Skipping cycle for no joint states")
            return False
        return True

    def StartupCheck(self) -> bool:
        # effectively skip to next cycle if these are none
        if self.map_helper is None:
            self.get_logger().warn(f"Skipping cycle for missing map")
            return False
        if self.known_obj_list is None:
            self.get_logger().warn(f"Skipping cycle for missing known objects")
            return False
        if ("joint_lift" not in self.js_map) or ("joint_wrist_yaw" not in self.js_map):
            self.get_logger().warn(f"Skipping cycle for no joint states")
            return False
        return True

    async def debug_state(self):

        await self.pour_water_action()
        raise

    def refresh_object(self, object: KnownObject) -> KnownObject:

        self.info(f"refreshing object's info for {object}")

        o: KnownObject
        for o in self.known_obj_list.objects:
            if o.id == object.id:
                self.info(f"refreshed object info is {o}")
                return o

        self.error(f"Cannot find the same object with existing id, given {object.id}")
        raise

    async def explore_planning_state(self):
        if not self.NavStartupMap():
            return self.CmdStates.EXPLORE_PLANNING

        self.get_logger().info(f"switching to nav mode")
        ret = await self.switch_nav_service.call_async(TriggerSrv.Request())
        self.get_logger().info(f"ret from switch nav {ret} ")

        # Look around with camera
        await self.move_single_joint("joint_head_tilt", -0.6, velocity=1.0 , fail_able= True)
        if self.first_explore:
            # First look around, so most range are lid up.
            await self.camera_scan_around(pan_delta=3.5 , velocity=0.25 )
            
            # Then point camera to left, and spin for a bit to cover the corner behind lift.s
            self.first_explore = False
            await self.move_single_joint("joint_head_pan", 0.8, velocity=0.5 , fail_able= True)
            point_on_left = PointStamped()
            point_on_left.header.frame_id = self.robot_baseframe
            # Basically want robot to look into +y direction, where its own pole is that it can't scan using camera head.s
            point_on_left.point.y = 0.5
            point_on_left.point.x = -0.1

            await self.turn_info_plant_cmdvel(self.get_point_in_frame(point_on_left,self.world_frame) , vel_max_clamp= 0.3)
        else:
            await self.camera_scan_around(pan_delta=1.8 , velocity=0.25 )

        # Need to give time for map to update
        time.sleep(2.0)

        # Find a spot to frontier over.
        # Need to know where robot is.
        self.info(f"Getting current robot loc")
        robot_start_point: PointStamped
        for i in range(3):
            maybe_robot_point = self.try_get_robot_point(self.map_helper.map_frame)
            if maybe_robot_point is not None:
                robot_start_point = maybe_robot_point
                break
            time.sleep(0.5)
        else:
            raise RuntimeError(f"Can not get robot's base location in map frame!")
        self.info(f"Current robot location {robot_start_point}")
        with self.map_lock:

            # This way we set id from here.
            def pub_cb(m:Marker):
                m.id = self.WAVE_FRONT_MARKER_ID
                self.pub_marker(m)

            frontier_cells = self.map_helper.find_reachable_frontiers(robot_start_point , self.FRONTIER_REACHABLE_RADIUS , pub_cb)
            self.info(f"Got {len(frontier_cells)} frontier cells")
            goal_point = None
            # We skip all cells that's under robot.

        # Release map lock at this point. we are done searching
        for cell in frontier_cells:
            cell_in_world =self.map_helper.map_loc_to_world_point(cell)
            distance = point_point_distanec(cell_in_world , robot_start_point.point)
            if distance > self.PLAN_GOAL_CLEARANCE_RADIUS:
                goal_point = cell_in_world
                # We make a heading for this goal.
                # Point robot back to source + 90 deg
                diff_x = robot_start_point.point.x - goal_point.x
                diff_y = robot_start_point.point.y - goal_point.y
                # Don't look back at start, it doesn't work well.
                heading = normalize_angle( - math.atan2(diff_y, diff_x) )

                # Check if this goal is plan-able
                maybe_plan_goal = await self.plan_nav_to_pose(goal_point,heading)
                if maybe_plan_goal is not None:
                    self.successful_planned_pose = maybe_plan_goal
                    self.info(f"FIND reachable and plan-able frontier: {maybe_plan_goal}")
                    return self.CmdStates.EXPLORE
            time.sleep(0.1)

        if goal_point is None:
            self.info("|| =============== EXPLORE ALL FINISHED! ================= ||\n"
            "no more valid frontier")
            self.clear_marker(self.WAVE_FRONT_MARKER_ID)
            self.successful_planned_pose = None
            return self.CmdStates.RETURN_HOME



    async def explore_state(self):
        # First put robot in nav mode
        self.get_logger().info(f"switching to nav mode")
        ret = await self.switch_nav_service.call_async(TriggerSrv.Request())
        self.get_logger().info(f"ret from switch nav {ret} ")

        # To ensure camera is looking where robot is going.
        await self.ResetCamera()

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = self.successful_planned_pose
        nav_goal.pose.header.stamp = self.get_clock().now().to_msg()
        self.info(f"Navigating to {nav_goal.pose.pose}")
        for i in range(3):
            result: NavigateToPose.Result
            status, result = await self.ActionSendAwait(
                nav_goal,
                self.navigate_to_pose_client,
                # print_fb,
            )
            if status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().warn(f"Did not get successful goal result, got {status}")
            else:
                # This is the successful break condition.
                break
        else:
            self.get_logger().error(
                f"Repeated Failure trying to nav to {nav_goal.pose.pose.position}")
            self.error("Going back and select next plant.")
            self.successful_planned_pose = None
            return self.CmdStates.IDLE

        self.clear_marker(self.POI_MARKER_ID)
        self.clear_marker(self.NAV_PLANED_LOC_MARKER_ID)
        self.clear_marker(self.WAVE_FRONT_MARKER_ID)
        self.successful_planned_pose = None
        return self.CmdStates.EXPLORE_PLANNING



    async def pause_before_next_state(self):
        """
        This is a state of just a bit of camera movement. 
        """


        if len(self.js_map) < 7:
            # This is just launch, we at least want js to be updated
            return
        if self.pause_start_time is None:
            self.clear_marker(self.NAV_PLANED_LOC_MARKER_ID)
            self.clear_marker(self.POI_MARKER_ID)
            # We don't want to look down, so arm don't generated shadows.

            self.get_logger().info(f"switching to nav mode")
            ret = await self.switch_nav_service.call_async(TriggerSrv.Request())
            self.get_logger().info(f"ret from switch nav {ret} ")

            await self.move_single_joint("joint_head_tilt", -0.5)
            # Look behind itself.
            await self.move_single_joint("joint_head_pan", -3.14 , velocity=0.3)
            await self.move_single_joint("joint_head_tilt", -0.25)

            self.pause_start_time = time.time()
            return self.CmdStates.PAUSE_BEFORE_NEXT

        if (time.time() - self.pause_start_time) > self.PAUSE_DURATION:
            self.get_logger().info("Pause timed up")
            self.pause_start_time = None
            return self.CmdStates.PLANT_SELECTION
        return self.CmdStates.PAUSE_BEFORE_NEXT

    async def plant_selection_state(self):
        """Pick the object for next round of action to plan for.

        Returns:
        """
        if not self.StartupCheck():
            return self.CmdStates.PAUSE_BEFORE_NEXT

        obj: KnownObject
        if self.next_planning_object_idx < len(self.known_obj_list.objects):

            obj: KnownObject = self.known_obj_list.objects[self.next_planning_object_idx]
            # We've pick the plant, increment counter
            self.next_planning_object_idx += 1  # move on for next iteration
        elif len(self.skipped_pot_objects) > 0:
            self.warn(f"Out of new objects to check, Going back to previously skipped objects")
            obj = self.skipped_pot_objects.popleft()
        else:
            self.get_logger().warn(f"{len(self.known_obj_list.objects)} locations all visited!")
            self.next_planning_object_idx = 0
            self.current_watering_obj = None
            self.current_chasing_plant = None

            return self.CmdStates.RETURN_HOME

        if obj.object_class != self.PLANT_CLASS_ID:
            self.get_logger().warn(f"Skipping Object typeof type {obj.object_class} ")
            return self.CmdStates.PLANT_SELECTION

        self.warn(f"\n\n---------------------------------------- \n"
                  "Working on new plant"
                  f"id {obj.id}"
                  f"at {obj.space_loc}"
                  "= VV = VV = VV =")

        self.current_chasing_plant = obj

        return self.CmdStates.NAV_PLANNING

    async def nav_planning_state(self) -> CmdStates:
        if not self.StartupCheck():
            return self.CmdStates.NAV_PLANNING

        self.get_logger().warn(f"Handling Object at index {self.next_planning_object_idx}")

        maybe_goal_pose = await self.plan_nav_to_obj_point(self.current_chasing_plant.space_loc,
                                                   self.MIN_PLAN_OFFSET_RADIUS,
                                                   self.MAX_PLAN_OFFSET_RADIUS)

        if maybe_goal_pose is None:
            self.error(
                f"Cannot plan to current object id {self.current_chasing_plant.id} at {self.current_chasing_plant.space_loc}"
            )
            self.skipped_pot_objects.append(self.current_chasing_plant)
            return self.CmdStates.PAUSE_BEFORE_NEXT

        self.successful_planned_pose = maybe_goal_pose
        self.get_logger().info(f"Recording with pose {self.successful_planned_pose}")

        # After planning move to executing
        return self.CmdStates.MOVING

    async def moving_state(self) -> CmdStates:

        # First put robot in nav mode
        self.get_logger().info(f"switching to nav mode")
        ret = await self.switch_nav_service.call_async(TriggerSrv.Request())
        self.get_logger().info(f"ret from switch nav {ret} ")

        # To ensure camera is looking where robot is going.
        await self.ResetCamera()

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = self.successful_planned_pose
        nav_goal.pose.header.stamp = self.get_clock().now().to_msg()
        for i in range(3):
            result: NavigateToPose.Result
            status, result = await self.ActionSendAwait(
                nav_goal,
                self.navigate_to_pose_client,
                # print_fb,
            )
            if status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().warn(f"Did not get successful goal result, got {status}")
            else:
                # This is the successful break condition.
                break
        else:
            self.get_logger().error(
                f"Repeated Failure trying to nav to {nav_goal.pose.pose.position}")
            self.error("Going back and select next plant.")
            return self.CmdStates.PAUSE_BEFORE_NEXT

        return self.CmdStates.WATERING_AREA_FINDING

    async def find_water_area_state(self) -> CmdStates:

        # First look at the plant's location.
        self.get_logger().warn("switching to pos mode ")
        ret = await self.switch_pos_service.call_async(TriggerSrv.Request())
        self.get_logger().info(f"ret from switch pos {ret} ")

        # we will basically only work on this coordinate from this point on.
        loc_world = self.current_chasing_plant.space_loc

        await self.camera_look_at_object(loc_world)
        # await self.camera_scan_around()
        time.sleep(1.0)  # Give yolo time to update
        self.current_chasing_plant = self.refresh_object(self.current_chasing_plant)

        search_trials = 0
        plant_object = self.current_chasing_plant
        scan_velocity = 0.18

        while True:
            # Then find a close by watering area.
            potential_watering_obj: KnownObject
            for potential_watering_obj in self.known_obj_list.objects:
                if potential_watering_obj.object_class == self.WATERING_AREA_CLASS_ID:
                    # Watering object must be above the pot
                    z_diff = potential_watering_obj.space_loc.point.z - plant_object.space_loc.point.z
                    self.info(
                        f"Comparing potential {potential_watering_obj} \nto {plant_object}\n"
                        f"distance {point_point_distanec(potential_watering_obj.space_loc.point , plant_object.space_loc.point)}\n"
                        f"z_diff { z_diff}")
                    if z_diff > -0.01:
                        if point_point_distanec(
                                potential_watering_obj.space_loc.point,
                                plant_object.space_loc.point) < self.POT_TO_WATERING_DIS_THRESHOLD:
                            self.current_watering_obj = potential_watering_obj
                            self.warn(
                                f"Found matching watering area with id {potential_watering_obj.id} at {potential_watering_obj.space_loc.point}"
                            )
                            return self.CmdStates.WATERING

            # we have went through all current objects without finding anything.
            # Move head around and try more
            if search_trials > 3:
                self.error(f"Can not find a valid watering area to pot id {plant_object.id} at "
                           f"{plant_object.space_loc} \n"
                           "Re selecting plants")
                return self.CmdStates.PAUSE_BEFORE_NEXT

            self.warn(f"Did not match any watering area, moving camera around more")
            await self.camera_scan_around(velocity=scan_velocity)
            self.current_chasing_plant = self.refresh_object(self.current_chasing_plant)
            plant_object = self.current_chasing_plant
            search_trials += 1
            scan_velocity = scan_velocity * 0.75

        return self.CmdStates.WATERING_AREA_FINDING

    async def watering_state(self) -> CmdStates:
        # TODO find the watering area in frame.
        # Just use the plant location for now.
        self.get_logger().warn("switching to pos mode ")
        ret = await self.switch_pos_service.call_async(TriggerSrv.Request())
        self.get_logger().info(f"ret from switch pos {ret} ")

        # we will basically only work on this coordinate from this point on.
        watering_loc = self.current_watering_obj.space_loc

        await self.camera_look_at_object(watering_loc)
        time.sleep(1.0)  # Give yolo time to update
        self.current_watering_obj = self.refresh_object(self.current_watering_obj)
        watering_loc = self.current_watering_obj.space_loc

        # Make sure we centered the wrist
        self.info(f"re-centering the wrist")
        await self.move_multi_joint(["joint_wrist_yaw", "joint_wrist_pitch", "joint_wrist_roll"],
                                    [0, 0.1, 0],
                                    duration=1.0,
                                    fail_able=True)

        # And also retract the arm
        if (self.get_wrist_extension_js() > 0.):
            self.info(f"collapsing arm")
            await self.move_multi_joint(
                ["joint_arm_l3", "joint_arm_l2", "joint_arm_l1", "joint_arm_l0"], [0, 0, 0, 0],
                duration=2.5,
                fail_able=True)

        # We first do a arm lift
        # with the gripper flatten, rise the lift up to slightly above target height.
        self.info(f"Lifting arm")
        loc_in_ee = self.get_point_in_frame(watering_loc, self.ee_frame)
        self.get_logger().info(f"current object loc in ee frame {loc_in_ee}")
        # This does needs a little extra.
        lift_amount = loc_in_ee.point.z + 0.03
        await self.move_single_joint_delta("joint_lift",
                                           lift_amount,
                                           duration=lift_amount / self.LIFT_VELOCITY_MAX + 0.5)

        # MOVE camera with the base turning.
        await self.turn_info_plant_cmdvel(watering_loc, offset=math.pi / 2)
        self.get_logger().warn("switching to pos mode again")
        # ret = await self.switch_traj_service.call_async(TriggerSrv.Request())
        ret = await self.switch_pos_service.call_async(TriggerSrv.Request())
        self.get_logger().info(f"ret from switch pos {ret} ")

        await self.camera_look_at_object(watering_loc)
        time.sleep(1.0)  # Give yolo time to update
        self.current_watering_obj = self.refresh_object(self.current_watering_obj)
        watering_loc = self.current_watering_obj.space_loc

        # Now we extend the arm to the plant.
        self.info(f"Extending arm")
        loc_in_ee = self.get_point_in_frame(watering_loc, self.ee_frame)

        # Now we take out the x-component. as the arm delta.

        if (loc_in_ee.point.x < 0.1):
            self.info(f"Arm already close enough to target.")
        else:
            # We don't do it all the way, leave a tiny tiny bit.
            extension_delta = loc_in_ee.point.x - 0.08
            self.info(f"extending arm for {extension_delta} amount")
            if extension_delta > 0.52:
                self.warn("arm extension value is too large, maybe should re-do moving?")
            await self.move_single_joint_delta("wrist_extension", extension_delta, duration=2.0)

        # Final lineup using the wrist.
        # The angle from wrist joint, into the plant is what's needed.

        # link_wrist_yaw is pointing z down, and in urdf, the joint is     <axis xyz="0.0 0.0 -1.0"/>
        # Need to flip the angle comming out here.

        # loc_in_ee = self.get_point_in_ee(loc_world)
        self.info("aiming wrist to target")
        loc_in_wrist = self.get_point_in_frame(watering_loc, "link_wrist_yaw")

        # Flip direction
        wrist_yaw_angle = normalize_angle(-math.atan2(loc_in_wrist.point.y, loc_in_wrist.point.x) -
                                          math.pi / 2)
        self.info(
            f"Target in wrist frame at {loc_in_wrist} , calculated, flipped angle is {wrist_yaw_angle}"
        )
        await self.move_single_joint_delta("joint_wrist_yaw", wrist_yaw_angle)

        time.sleep(3.0)

        self.info("!! pouring water !! ")
        # Need two moves, one rise and pour, another one that goes back.
        await self.pour_water_action()

        # TODO mark the PLANT when it is watered!
        text_pos = watering_loc
        text_pos.point.z += 0.5

        self.watered_objects.append(self.current_chasing_plant )
        self.pub_marker(MakeTextMarker(self.TEXT_INFO_ID, "Watered!", text_pos))

        self.pub_marker(
            MakeSphereMaker(self.WATERED_PLANT_MARKER_ID,
                            watering_loc.point,
                            watering_loc.header,
                            ColorRGBA(r=1.0, g=0.1, b=0.1, a=0.9),
                            scale=0.4))


        time.sleep(2.0)
        self.warn("Retracting arm ! ")

        await self.move_single_joint("wrist_extension", 0.001, duration=2.0)

        self.info(f"Turning the base back and lowering the lift.")
        # Now turn the base back. to clear for lowering the arm
        await self.ResetCamera()
        await self.turn_info_plant_cmdvel(watering_loc, offset=0)
        await self.lower_arm_to_nav()

        self.warn(
            f"Watered plant with id {self.current_chasing_plant.id} at {self.current_chasing_plant.space_loc}"
        )
        return self.CmdStates.PAUSE_BEFORE_NEXT


    async def homing_state(self):
        self.get_logger().info(f"switching to nav mode")
        ret = await self.switch_nav_service.call_async(TriggerSrv.Request())
        self.get_logger().info(f"ret from switch nav {ret} ")

        # To ensure camera is looking where robot is going.
        await self.ResetCamera()

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose.header.frame_id = self.world_frame
        nav_goal.pose.header.stamp = self.get_clock().now().to_msg()
        nav_goal.pose.pose.position.x =0.0
        nav_goal.pose.pose.position.y =0.0
        nav_goal.pose.pose.position.z =0.0
        nav_goal.pose.pose.orientation.x = 0.0
        nav_goal.pose.pose.orientation.y = 0.0
        nav_goal.pose.pose.orientation.z = 0.0
        nav_goal.pose.pose.orientation.w = 1.0

        for i in range(3):
            result: NavigateToPose.Result
            status, result = await self.ActionSendAwait(
                nav_goal,
                self.navigate_to_pose_client,
                # print_fb,
            )
            if status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().warn(f"Did not get successful goal result, got {status}")
            else:
                # This is the successful break condition.
                break
        else:
            self.get_logger().error(
                f"Repeated Failure trying to nav to {nav_goal.pose.pose.position}")
            self.error("Going back and select next plant.")
            return self.CmdStates.IDLE

        return self.CmdStates.IDLE


    async def lower_arm_to_nav(self):
        await self.move_single_joint("joint_lift", 0.28, duration=2.0)

    async def plan_nav_to_pose(self,point: Point , z_heading:float ) -> Optional[PoseStamped]:

        pose_goal = self.make_ComputePathToPose_goal(point.x, point.y, z_heading)

        goal_marker = MakeCylinderMarker(id=self.POI_MARKER_ID,
                                            pos=pose_goal.goal.pose.position,
                                            header=pose_goal.goal.header,
                                            height=1.7)
        self.pub_marker(goal_marker)

        self.get_logger().info(f"Try nav planning to {pose_goal.goal.pose.position}")

        planning_result: ComputePathToPose.Result
        goal_status, planning_result = await self.ActionSendAwait(
            pose_goal, self.compute_path_client)
        if goal_status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(
                f"Did not get successful goal result, got {goal_status} with {planning_result}"
            )
            self.clear_marker(self.POI_MARKER_ID)
            return None
        else:
            # This is the successful break condition.
            # Get back the pose we have successfully planned for
            self.get_logger().info(
                f"Got a planned path to {point} , planning_time {planning_result.planning_time} "
            )
            return pose_goal.goal



    async def plan_nav_to_obj_point(self, goal_point: PointStamped, min_proximity,
                            max_proximity) -> Optional[PoseStamped]:

        if goal_point.header.frame_id != self.map_helper.map_frame:
            self.warn(
                f"Planning target have different frame then map ({goal_point.header.frame_id}),"
                " Converting")
            maybe_goal_point = self.try_transform(goal_point, self.map_helper.map_frame)
            if maybe_goal_point is None:
                self.warn(
                    f"Cannot convert goal {goal_point} into map frame {self.map_helper.map_frame}")
                return None
            goal_point = maybe_goal_point

        # Search go out as a circle.
        # every time the potential points are emptied, search radius increase and fill the list again.
        target_map_coord = self.map_helper.world_point_to_map(goal_point.point)

        check_cell_rad = math.ceil(self.PLAN_GOAL_CLEARANCE_RADIUS /
                                   self.map_helper.map_info.resolution)
        self.get_logger().info(f"Searching for plan-able grid-location with clearance of "
                               f"{self.PLAN_GOAL_CLEARANCE_RADIUS} m / {check_cell_rad} grids")

        min_search_ring_grid_rad = min_proximity / self.map_helper.meter_per_cell
        max_search_ring_grid_rad = max_proximity / self.map_helper.meter_per_cell

        for ring_grid_rad in range(int(min_search_ring_grid_rad),
                                   int(max_search_ring_grid_rad) + 1):
            for maybe_coord in self.map_helper.GetRing(target_map_coord, ring_grid_rad):
                maybe_valid,  checked_coords = self.map_helper.CheckEmptyCircle(
                    maybe_coord, check_cell_rad)
                # TODO this marker is not ID so won't be cleared.
                # debug_marker = self.map_helper.color_sphere_gen(checked_coords,
                #                                                 id=self.NAV_PLANED_LOC_MARKER_ID,
                #                                                 color=COLOR_MSG_LIST_RGBW[1])
                # self.pub_marker(debug_marker)

                if maybe_valid:
                    planning_xy = self.map_helper.map_loc_to_world(maybe_coord)
                    loc_x, loc_y = planning_xy
                    diff_x = goal_point.point.x - loc_x
                    diff_y = goal_point.point.y - loc_y
                    heading_into_plant = math.atan2(diff_y, diff_x)
                    pose_goal = self.make_ComputePathToPose_goal(loc_x, loc_y, heading_into_plant)
                    goal_marker = MakeCylinderMarker(id=self.POI_MARKER_ID,
                                                     pos=pose_goal.goal.pose.position,
                                                     header=pose_goal.goal.header,
                                                     height=goal_point.point.z * 2)
                    self.pub_marker(goal_marker)

                    self.get_logger().info(f"Try nav planning to {pose_goal.goal.pose.position}")

                    planning_result: ComputePathToPose.Result
                    goal_status, planning_result = await self.ActionSendAwait(
                        pose_goal, self.compute_path_client)
                    if goal_status != GoalStatus.STATUS_SUCCEEDED:
                        self.get_logger().warn(
                            f"Did not get successful goal result, got {goal_status} with {planning_result}"
                        )
                    else:
                        # This is the successful break condition.
                        # Get back the pose we have successfully planned for
                        self.get_logger().info(
                            f"Got a planned path to {planning_xy} , planning_time {planning_result.planning_time} "
                        )
                        return pose_goal.goal

                # Try again on next cell
        else:
            self.error(
                f"MAX Plan offset radius reached for {goal_point.point}\n"
                f"Last tried radius: cell: {ring_grid_rad} world: {ring_grid_rad * self.map_helper.meter_per_cell}\n"
            )
            return None

    async def turn_info_plant_cmdvel(self, target_point: PointStamped, offset=0 , vel_max_clamp = 0.8):

        self.get_logger().warn("switching to nav mode ")

        switch_mode_future = self.switch_nav_service.call_async(TriggerSrv.Request())

        self.info(f"Turning base to lineup arm")

        loc_base = self.get_point_in_frame(target_point, self.robot_baseframe)
        self.get_logger().info(f"Target's location in base frame is {loc_base}")
        self.pub_marker(
            MakeCylinderMarker(id=self.POI_MARKER_ID,
                               pos=loc_base.point,
                               header=loc_base.header,
                               diameter=0.08,
                               height=0.3,
                               alpha=0.3))

        # Then we compute the angle for turning.
        base_diff_angle = math.atan2(loc_base.point.y, loc_base.point.x)
        base_diff_offset_angle = normalize_angle(base_diff_angle + offset)
        # base_diff_offset_angle *= 0.1 # First round, we expect the base to stuck for the first round, so only turn a little, so don't waste much time.
        ret = await switch_mode_future
        self.get_logger().info(f"switch pos RET {ret} ")
        self.info(f"Initial angle offset: {base_diff_offset_angle}")

        ang_vel_gain = 0.5

        while abs(base_diff_offset_angle) > 0.05:

            # let's do min 0.1 max 1.0
            clamped_cmd_mag = max(min(abs(base_diff_offset_angle * ang_vel_gain), vel_max_clamp ), 0.08)

            cmd_twist = Twist()
            cmd_twist.angular.z = clamped_cmd_mag * sign(base_diff_offset_angle) 
            self.info(f"amount to turn {base_diff_offset_angle} , z vel {cmd_twist.angular.z}")

            self.cmd_vel_pub.publish(cmd_twist)

            time.sleep(0.02)

            loc_base = self.get_point_in_frame(target_point, self.robot_baseframe)

            base_diff_angle = math.atan2(loc_base.point.y, loc_base.point.x)
            base_diff_offset_angle = normalize_angle(base_diff_angle + offset)

        # Finish off with a 0 speed.
        self.cmd_vel_pub.publish(Twist())

    # async def turn_base_into_plant(self ,loc_world , offset = math.pi/2):
    #     self.get_logger().warn("switching to traj mode ")

    #     switch_mode_future =  self.switch_traj_service.call_async(TriggerSrv.Request())

    #     self.info(f"Turning base to lineup arm")

    #     loc_base = self.get_point_in_frame(loc_world, self.robot_baseframe)
    #     self.get_logger().info(f"Target's location in base frame is {loc_base}")
    #     self.marker_pub.publish(MakeCylinderMarker(id=101 , pos=loc_base.point , header= loc_base.header,diameter=0.08))

    #     # Then we compute the angle for turning.
    #     base_diff_angle = math.atan2(loc_base.point.y , loc_base.point.x)
    #     base_diff_angle_offsetted = normalize_angle(base_diff_angle + offset)
    #     # base_diff_angle_offsetted *= 0.1 # First round, we expect the base to stuck for the first round, so only turn a little, so don't waste much time.
    #     ret = await switch_mode_future
    #     self.get_logger().info(f"ret from switch pos {ret} ")

    #     for i in range(6):
    #         self.info(f"Trying for {i} th time")
    #         self.get_logger().info(f"Target angle is {base_diff_angle_offsetted}")

    #         # This is a special action sending, so not using the helpers here.
    #         traj_goal = FollowJointTrajectory.Goal()
    #         traj_goal.multi_dof_trajectory = self.build_rot_traj(base_diff_angle_offsetted)

    #         result : FollowJointTrajectory.Result
    #         status, result = await self.ActionSendAwait(
    #             traj_goal,
    #             self.trajectory_action_client,
    #         )
    #         if (result.error_code == FollowJointTrajectory.Result.SUCCESSFUL):
    #             self.get_logger().info("Successful! ")
    #         if status != GoalStatus.STATUS_SUCCEEDED:
    #             self.get_logger().warn(f"Did not get successful goal result, got {status}" )

    #         loc_base = self.get_point_in_frame(loc_world, self.robot_baseframe)
    #         self.get_logger().info(f"Target's location in base frame is {loc_base}")
    #         self.marker_pub.publish(MakeCylinderMarker(id=101 , pos=loc_base.point , header= Header(frame_id=self.robot_baseframe),diameter=0.08))

    #         # Then we compute the angle for turning.
    #         base_diff_angle = math.atan2(loc_base.point.y , loc_base.point.x)
    #         base_diff_angle_offsetted = normalize_angle(base_diff_angle + offset)
    #         if base_diff_angle_offsetted < 0.03:
    #             self.get_logger().info(f"Arm is pointing roughly towards the plants.")
    #             break

    #     else:
    #         self.get_logger().info(f"Failed to turn base into target heading.")
    #         raise RuntimeError("Cannot Turn the base into watering area.")

    async def pour_water_action(self):
        # joint_wrist_pitch 0 -> -0.486
        # Lift delta + 0.1581
        # Arm + 0.043801334055

        self.get_logger().warn("switching to pos mode ")
        # ret = await self.switch_traj_service.call_async(TriggerSrv.Request())
        ret = await self.switch_pos_service.call_async(TriggerSrv.Request())
        self.get_logger().info(f"ret from switch pos {ret} ")

        lift_init = self.js_map["joint_lift"]
        arm_init = self.get_wrist_extension_js()
        pitch_init = 0.09
        # pitch is abs mode.

        lift_delta = 0.1881
        arm_delta = 0.052801
        pitch_target = -0.69

        time_delta = 2.0
        current_time = 0.0
        traj_goal = FollowJointTrajectory.Goal()
        traj_goal.trajectory.joint_names = ["joint_lift", "wrist_extension", "joint_wrist_pitch"]

        current_time += 0.2
        # Add current location in
        # Don't give it the init move, some how the dynamixal is really bad at tiny moves
        j_p = JointTrajectoryPoint()
        j_p.positions = [lift_init, arm_init, self.js_map["joint_wrist_pitch"]]
        # j_p.velocities = [0.0, 0.0, 0.1 / 0.2]
        j_p.time_from_start = self.build_duration(current_time)
        traj_goal.trajectory.points.append(j_p)

        current_time += time_delta
        j_p = JointTrajectoryPoint()
        j_p.positions = [lift_init + lift_delta, arm_init + arm_delta, pitch_target]
        j_p.velocities = [
            lift_delta / time_delta,
            arm_delta / time_delta,
            pitch_target / time_delta,
        ]
        j_p.time_from_start = self.build_duration(current_time)
        traj_goal.trajectory.points.append(j_p)

        # Pause a little
        # current_time += 0.5
        # j_p.time_from_start = self.build_duration(current_time)
        # traj_goal.trajectory.points.append(j_p)

        # j_p = JointTrajectoryPoint()
        # j_p.positions = [lift_init + lift_delta, arm_init + arm_delta, pitch_target-0.05]
        # j_p.velocities = [0.1, 0.1, -0.05 / 0.5]
        # j_p.time_from_start = self.build_duration(current_time)
        # traj_goal.trajectory.points.append(j_p)

        current_time += time_delta - 0.1
        # Add current location in
        j_p = JointTrajectoryPoint()
        j_p.positions = [lift_init, arm_init, pitch_init]
        j_p.velocities = [
            -lift_delta / time_delta,
            -arm_delta / time_delta,
            -pitch_target / time_delta,
        ]
        j_p.time_from_start = self.build_duration(current_time)
        traj_goal.trajectory.points.append(j_p)

        result: FollowJointTrajectory.Result
        status, result = await self.ActionSendAwait(
            traj_goal,
            self.trajectory_action_client,
        )
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.warn(f"Did not get successful goal status, got {status}")

        if (result.error_code == FollowJointTrajectory.Result.SUCCESSFUL):
            self.info("Successful! ")
        else:
            self.error(f"FAILED! {result} ")
            raise
        return True

    def get_point_in_frame(self, target_point, frame_name):
        # Re compute the target location in robot's frame.
        for i in range(5):
            loc_in_ee = self.try_transform(target_point, frame_name)
            time.sleep(0.5)
            if loc_in_ee is not None:
                return loc_in_ee
        else:
            self.get_logger().info(f"Time out getting transform")
            raise RuntimeError(f"Timeout getting point {target_point} into frame {frame_name}")

    async def ResetCamera(self):
        self.info("Resetting camera ")
        await self.move_multi_joint(["joint_head_pan", "joint_head_tilt"], [0.0, -0.65])

    async def camera_look_at_object(self, loc_world):
        # link_head_pan
        loc_pan = self.get_point_in_frame(loc_world, "link_head_pan")
        pan_angle = get_z_angle_to_point(loc_pan.point)
        self.info(
            f"Turning camera to look at relative point {loc_pan} , computed angle {pan_angle}")
        await self.move_single_joint_delta("joint_head_pan", pan_angle)

        loc_tilt = self.get_point_in_frame(loc_world, "link_head_tilt")
        tilt_angle = get_z_angle_to_point(loc_tilt.point)
        self.info(
            f"Turning camera to look at relative point {loc_tilt} , computed angle {tilt_angle}")
        await self.move_single_joint_delta("joint_head_tilt", tilt_angle)

        # link_head_tilt

        self.move_multi_joint(["joint_head_pan", "joint_head_tilt"], [0.0, -0.5])

    async def camera_scan_around(self, pan_delta=0.5, tilt_delta=0.4, velocity=0.18):
        # Scan camera around current pointed location.
        # Pan scan
        self.info(f"Scanning camera around with pan detla {pan_delta} , tilt delta {tilt_delta}")
        current_pan = self.js_map["joint_head_pan"]
        current_tilt = self.js_map["joint_head_tilt"]
        # Pan range -4.04 1.73
        pan_left = min(current_pan + pan_delta, 1.6)
        pan_right = max(current_pan - pan_delta, -4.04)
        await self.move_single_joint("joint_head_pan", pan_left, velocity=velocity,fail_able=True)
        await self.move_single_joint("joint_head_pan", pan_right, velocity=velocity,fail_able=True)
        await self.move_single_joint("joint_head_pan", current_pan, velocity=velocity,fail_able=True)

        if tilt_delta < .05:
            return
        time.sleep(0.5)
        # tilt range 0.49 -2.02
        tilt_up = min(current_tilt + tilt_delta, 0.45)
        tilt_down = max(current_tilt - tilt_delta, -1.8)
        await self.move_single_joint("joint_head_tilt", tilt_up, velocity=velocity,fail_able=True)
        await self.move_single_joint("joint_head_tilt", tilt_down, velocity=velocity,fail_able=True)
        await self.move_single_joint("joint_head_tilt", current_tilt, velocity=velocity,fail_able=True)

    def build_rot_traj(self,
                       target_angle: float,
                       angular_vel=0.2,
                       total_duration=None) -> MultiDOFJointTrajectory:

        traj = MultiDOFJointTrajectory()
        traj.joint_names.append("position")

        if total_duration is None:
            total_duration = abs(target_angle) / angular_vel

        # self.get_logger().info(f"Current angle {current_angle}")
        traj_point = MultiDOFJointTrajectoryPoint()
        traj_point.transforms.append(self.build_turning_tf(0.0))
        traj_point.time_from_start = self.build_duration(0)
        traj.points.append(traj_point)

        # Add a final dot that is the final dest angle.
        traj_point = MultiDOFJointTrajectoryPoint()
        traj_point.transforms.append(self.build_turning_tf(target_angle))
        traj_point.time_from_start = self.build_duration(total_duration + 0.05)
        traj.points.append(traj_point)
        # Might need to find the base frame's angle in wrt odom as the starting offset angle.

        self.get_logger().info(f"traj length {len(traj.points)}")
        return traj

    def build_duration(self, sec: float) -> DurationMsg:
        return DurationMsg(sec=int(sec), nanosec=int((sec % 1) * 1e9))

    def build_turning_tf(self, angle) -> Transform:
        goal_tf = Transform()
        q = tf_transformations.quaternion_from_euler(0, 0, angle)
        goal_tf.rotation.x = q[0]
        goal_tf.rotation.y = q[1]
        goal_tf.rotation.z = q[2]
        goal_tf.rotation.w = q[3]
        return goal_tf

    async def move_single_joint_delta(self,
                                      joint_name: str,
                                      delta: float,
                                      duration=1.0,
                                      fail_able=False) -> bool:
        actual_target = self.js_map[joint_name] + delta
        return await self.move_single_joint(joint_name,
                                            actual_target,
                                            duration,
                                            fail_able=fail_able)

    async def move_single_joint(self,
                                joint_name: str,
                                target: float,
                                duration=1.0,
                                velocity=None,
                                fail_able=False) -> bool:
        traj_goal = FollowJointTrajectory.Goal()

        # Add current location in
        traj_goal.trajectory.joint_names.append(joint_name)

        if velocity is None:
            velocity = (target - self.js_map[joint_name]) / duration
        else:
            duration = abs((target - self.js_map[joint_name]) / velocity)

        if sign(velocity) != sign(target - self.js_map[joint_name]):
            velocity = -velocity

        self.info(f"moving {joint_name} to {target} over {duration}s")
        j_p = JointTrajectoryPoint()
        j_p.positions.append(self.js_map[joint_name])
        j_p.time_from_start = self.build_duration(0.0)
        traj_goal.trajectory.points.append(j_p)

        j_p = JointTrajectoryPoint()
        j_p.positions.append(target)
        j_p.velocities.append(velocity)
        j_p.time_from_start = self.build_duration(duration)
        traj_goal.trajectory.points.append(j_p)

        result: FollowJointTrajectory.Result
        status, result = await self.ActionSendAwait(
            traj_goal,
            self.trajectory_action_client,
        )
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.warn(f"Did not get successful goal status, got {status}")

        if (result.error_code == FollowJointTrajectory.Result.SUCCESSFUL):
            self.info("Successful! ")
        else:

            if not fail_able:
                self.error(f"FAILED! {result} ")
                raise RuntimeError(f"Joint motion failed with {result} ")
            self.warn(f"FAILED! {result} ")
            return False
        return True

    async def move_multi_joint_delta(self,
                                     joint_names: str,
                                     delta_targets: float,
                                     duration=1.0,
                                     fail_able=False) -> bool:
        target_list = []
        for name, delta in zip(joint_names, delta_targets):
            target_list.append(self.js_map[name] + delta)
        return await self.move_multi_joint(joint_names, target_list, duration, fail_able)

    async def move_multi_joint(self,
                               joint_names: str,
                               targets: float,
                               duration=1.0,
                               fail_able=False) -> bool:
        self.info(f"moving \n{joint_names} to \n{targets} over {duration}s")

        traj_goal = FollowJointTrajectory.Goal()

        # Add current location in
        j_p = JointTrajectoryPoint()
        for n in joint_names:
            traj_goal.trajectory.joint_names.append(n)
            j_p.positions.append(self.js_map[n])
        j_p.time_from_start = self.build_duration(0.0)
        traj_goal.trajectory.points.append(j_p)

        j_p = JointTrajectoryPoint()
        for t in targets:
            j_p.positions.append(t)
        j_p.time_from_start = self.build_duration(duration)
        traj_goal.trajectory.points.append(j_p)

        result: FollowJointTrajectory.Result
        status, result = await self.ActionSendAwait(
            traj_goal,
            self.trajectory_action_client,
        )
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.warn(f"Did not get successful goal status, got {status}")

        if (result.error_code == FollowJointTrajectory.Result.SUCCESSFUL):
            self.info("Successful! ")
        else:

            if not fail_able:
                self.error(f"FAILED! {result} ")
                raise RuntimeError(f"Failed to execute trajectory! ")
            self.warn(f"FAILED! {result} ")
            return False
        return True

    def apply_transform(self, point: PointStamped, transform: TransformStamped):
        # Extract the translation from the Transform message
        translation = np.array([
            transform.transform.translation.x, transform.transform.translation.y,
            transform.transform.translation.z
        ])

        # Extract the rotation (quaternion) from the Transform message
        rotation = [
            transform.transform.rotation.x, transform.transform.rotation.y,
            transform.transform.rotation.z, transform.transform.rotation.w
        ]

        # Convert the point to a numpy array
        point_vector = np.array([point.point.x, point.point.y, point.point.z])

        # Rotate the point using the quaternion
        rotated_point = tf_transformations.quaternion_matrix(rotation)[:3, :3].dot(point_vector)

        # Apply the translation
        transformed_point = rotated_point + translation

        # Create a new Point object with the transformed coordinates
        transformed_point_msg = PointStamped()
        transformed_point_msg.header = transform.header
        transformed_point_msg.point.x = transformed_point[0]
        transformed_point_msg.point.y = transformed_point[1]
        transformed_point_msg.point.z = transformed_point[2]

        return transformed_point_msg

    def try_transform_tf(self, point_stamp: PointStamped, target_frame):
        try:
            return self._tf_buffer.transform(point_stamp, target_frame)
        except Exception as ex:
            self.get_logger().warn(f"failed to get transform, {ex}")
            return None

    def try_get_robot_point(self , source_frame:str):
        try:
            tf = self._tf_buffer.lookup_transform(target_frame=source_frame,
                                                  source_frame=self.robot_baseframe,
                                                  time=rclpy.time.Time())
            p = PointStamped()
            p.point.x = tf.transform.translation.x
            p.point.y = tf.transform.translation.y
            p.point.z = tf.transform.translation.z
            p.header = tf.header
            return p
        except Exception as ex:
            self.get_logger().warn(f"failed to get transform, {ex}")
            return None


    def try_transform(self, point_stamp: PointStamped, target_frame) -> Optional[PointStamped]:

        try:
            tf = self._tf_buffer.lookup_transform(target_frame=target_frame,
                                                  source_frame=point_stamp.header.frame_id,
                                                  time=rclpy.time.Time())
            return self.apply_transform(point_stamp, tf)
        except Exception as ex:
            self.get_logger().warn(f"failed to get transform, {ex}")
            return None

    def get_wrist_extension_js(self):
        return self.js_map["joint_arm_l3"] + self.js_map["joint_arm_l2"] + self.js_map[
            "joint_arm_l1"] + self.js_map["joint_arm_l0"]

    def clear_marker(self, marker_id):
        m = Marker()
        m.ns = self.MARKER_NAMESPACE
        m.id = marker_id
        m.action = Marker.DELETEALL
        self.pub_marker(m)


def main(args=None):
    rclpy.init(args=args)

    try:
        mode_swtich_node = ModeSwitchNode()
        node = GoalMover(mode_swtich_node)
        exec = MultiThreadedExecutor(5)
        exec.add_node(mode_swtich_node)
        exec.add_node(node)
        exec.spin()
        # rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('interrupt received, so shutting down')

    rclpy.shutdown()


if __name__ == "__main__":
    main()
