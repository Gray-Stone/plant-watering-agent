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
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration as DurationMsg
from builtin_interfaces.msg import Time as RosTime
from geometry_msgs.msg import Point, PointStamped, TransformStamped, Transform, Vector3 , PoseStamped, Pose
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

from stretch_mover.msg import (KnownObject, KnownObjectList, YoloDetection,
                               YoloDetectionList)
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup , ReentrantCallbackGroup
from collections import deque

from trajectory_msgs.msg import MultiDOFJointTrajectory , MultiDOFJointTrajectoryPoint, JointTrajectory, JointTrajectoryPoint


from stretch_mover_utils.grid_utils import OccupancyGridHelper , COLOR_MSG_LIST_RGBW

from control_msgs.action import FollowJointTrajectory
# COLOR_MSG_LIST_RGBW = [
#     ColorRGBA(r=1.0,b=0.0,g=0.0,a=1.0),
#     ColorRGBA(r=0.0,b=0.0,g=1.0,a=1.0),
#     ColorRGBA(r=0.0,b=1.0,g=0.0,a=1.0),
#     ColorRGBA(r=1.0,b=1.0,g=1.0,a=1.0),
# ]

def normaliz_angle (angle: float) -> float:


    #   return rad - (std::ceil((rad + PI) / (2.0 * PI)) - 1.0) * 2.0 * PI;

    return angle - (math.ceil((angle  + math.pi) / (2.0 * math.pi)) - 1.0) * 2.0 * math.pi;

def map_marker_check(map_helper: OccupancyGridHelper)->Marker:
    # Map data: -1 unknown, 0 free, 100 occupied.
    locs = []
    for ix in range(map_helper.map_info.width):
        for iy in range(map_helper.map_info.height):
            loc = (ix,iy)
            if not map_helper.ValidLoc(loc):
                raise ValueError(f"ixy {loc} is out of bound! ")
            locs.append(loc)

    return map_helper.color_sphere_gen(locs , scale = 0.03)


def MakeSphereMaker(id , pos: Point , header , color : ColorRGBA = ColorRGBA(r=1.0,a=1.0)) -> Marker:
    m = Marker()
    m.header = header
    m.type = Marker.SPHERE
    m.id = id
    m.pose.position = pos
    m.color = color
    m.scale.x = 0.1
    m.scale.y = 0.1
    m.scale.z = 0.1
    return m


def MakeCylinderMarker(id,
                       pos: Point,
                       header,
                       color=COLOR_MSG_LIST_RGBW[1],
                       diameter=0.02,
                       height=1.8) -> Marker:
    m = Marker()
    m.header = header
    m.type = Marker.CYLINDER
    m.id = id
    m.pose.position = pos
    m.color = color
    m.scale.x = diameter
    m.scale.y = diameter
    m.scale.z = height
    return m

class GoalMover(Node):

    PAUSE_DURATION = 2.0

    class CmdStates(Enum):
        IDLE = enum.auto()

        PLANT_SELECTION = enum.auto()

        PLANNING = enum.auto()
        MOVING = enum.auto()

        WATERING_AREA_FINDING = enum.auto()

        ARM_TURNING = enum.auto()
        PAUSE = enum.auto()


    def pub_marker(self , m:Marker):
        self.marker_pub.publish(m)

    def __init__(self):
        super().__init__("move_to_plants")

        #### Parameters
        self.declare_parameter("pot_class_id" , int(1))
        # world frame will also be the map's frame.
        self.declare_parameter("world_frame" , 'map')
        self.declare_parameter("robot_baseframe" , "base_footprint")
        self.declare_parameter("mast_frame" , "link_mast")
        self.declare_parameter("ee_frame" , "link_grasp_center")

        self.declare_parameter("known_object_topic", "yolo_ros/known_objects")

        self.pot_class_id = self.get_parameter("pot_class_id").get_parameter_value().integer_value
        self.world_frame = self.get_parameter("world_frame").get_parameter_value().string_value
        self.robot_baseframe = self.get_parameter("robot_baseframe").get_parameter_value().string_value
        self.mast_frame = self.get_parameter("mast_frame").get_parameter_value().string_value
        self.ee_frame = self.get_parameter("ee_frame").get_parameter_value().string_value
        known_object_topic = self.get_parameter("known_object_topic").get_parameter_value().string_value


        #### Member Vars
        self.map_helper :OccupancyGridHelper = None
        self.known_obj_list: KnownObjectList = None
        self.js_map: dict[str,float] = {}
        self.next_planning_object_idx = 0
        self.current_chasing_obj : KnownObject = None

        self.successful_planned_pose: Pose = None

        self.plan_dest_clearance_radius = 0.2
        self.max_plan_offset_radius = 0.7 # arm length is 0.52

        self.state = self.CmdStates.IDLE
        self.state_update(self.CmdStates.PLANT_SELECTION)


        # Time keeper for the pause state.
        self.pause_start_time = time.time()

        #### Publisher
        self.state_change_publisher = self.create_publisher(StringMsg , "move_plant_state_change" , 2)
        self.marker_pub = self.create_publisher(Marker, "VisualizationMarker" , 1)

        #### Service client
        # /switch_to_navigation_mode [std_srvs/srv/Trigger]
        # /switch_to_position_mode [std_srvs/srv/Trigger]
        # /switch_to_trajectory_mode [std_srvs/srv/Trigger]
        self.service_client_cb_group = ReentrantCallbackGroup()

        self.switch_nav_service = self.create_client(TriggerSrv,"switch_to_navigation_mode" ,callback_group=self.service_client_cb_group)
        self.switch_pos_service = self.create_client(TriggerSrv,"switch_to_position_mode" ,callback_group=self.service_client_cb_group)
        self.switch_traj_service = self.create_client(TriggerSrv,"switch_to_trajectory_mode" ,callback_group=self.service_client_cb_group)

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

        self.map_subs = self.create_subscription(OccupancyGrid, "/map" , self.global_map_cb , 1)
        # self.map_subs = self.create_subscription(OccupancyGrid, "/local_costmap/costmap" , self.global_map_cb , 1)
        self.known_obj_subs  = self.create_subscription(  KnownObjectList , known_object_topic , self.known_obj_cb , 1)
        self.js_sub = self.create_subscription(JointState , "joint_states" , self.joint_state_cb ,2)
        ## main timer
        self.create_timer(0.5,self.main_timer)
        self.get_logger().info("Node configured, timer created!")



    def global_map_cb(self,map_msg : OccupancyGrid):
        # print(f"\n\n ================== ")
        # print(f"Got map , header {map_msg.header} , info {map_msg.info}")

        self.map_helper = OccupancyGridHelper(map_msg)
        # Map data: -1 unknown, 0 free, 100 occupied.
        # For debug purpose.
        # self.pub_marker(map_marker_check(self.map_helper))
    def known_obj_cb(self, msg: KnownObjectList):
        self.known_obj_list = msg

    def joint_state_cb(self,msg:JointState):

        for name,val in zip(msg.name , msg.position):
            self.js_map[name] = val

    def state_update(self , new_state:'CmdStates'):
        if new_state == self.state:
            return

        self.get_logger().warn(f"State changing from {self.state} to {new_state}")
        self.state = new_state
        return

    async def main_timer(self):
        if not self.StartupCheck():
            return

        # This is the main logic.
        if self.state == self.CmdStates.IDLE:
            # Do nothing in idle state.
            return

        elif self.state == self.CmdStates.PAUSE:
            if self.pause_start_time is None:
                self.pause_start_time = time.time()
                return
            if time.time() - self.pause_start_time > self.PAUSE_DURATION:
                self.get_logger().info("Pause timed up")
                self.state_update(self.CmdStates.PLANT_SELECTION)
                self.pause_start_time = None
            return

        elif self.state == self.CmdStates.PLANT_SELECTION:
            self.state_update(await self.plant_selection_state())


        elif self.state == self.CmdStates.PLANNING:
            self.state_update(await self.planning_state())

        elif self.state == self.CmdStates.MOVING:
            self.state_update(await self.moving_state())
        elif self.state == self.CmdStates.ARM_TURNING:
            self.state_update(await self.arm_pointing_state())

        else:
            self.get_logger().error(f"Stuck in non-existing state: {self.state}")
            raise RuntimeError(f"Stuck in non-existing state: {self.state}")
        # Reached here cuz state is just wrong !

    def make_ComputePathToPose_goal(self , x,y):
        pose_goal = ComputePathToPose.Goal()
        pose_goal.goal.header.frame_id = self.world_frame
        pose_goal.goal.header.stamp = self.get_clock().now().to_msg()

        # we should be able to leave stamp empty.
        pose_goal.goal.pose.position.x = x
        pose_goal.goal.pose.position.y = y
        # Must fill in the planner id, or it won't print the action failed reason.
        pose_goal.planner_id = "GridBased"

        # TODO add orientation
        pose_goal.use_start = False
        return pose_goal

    async def ActionSendAwait(self,goal_obj , client : ActionClient , feedback_cb = None )-> tuple [GoalStatus,any]:

        goal_handle: ClientGoalHandle =  await client.send_goal_async(goal_obj , feedback_callback=feedback_cb)

        if not goal_handle.accepted:
            raise ValueError("goal rejected!")

        # for example res is a "ComputePathToPose_GetResult_Response" object.
        res = await goal_handle.get_result_async()
        # self.get_logger().warn(f"type of res is {type(res)}, itself is \n{res}")

        # if res.status != GoalStatus.STATUS_SUCCEEDED:
        return res.status,  res.result


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

    async def plant_selection_state(self):
        """Pick the object for next round of action to plan for.

        Returns:
            
        """
        if self.next_planning_object_idx >= len(self.known_obj_list.objects):
            self.get_logger().warn(f"{len(self.known_obj_list.objects)} locations all visited!")
            return self.CmdStates.IDLE

        self.get_logger().warn(f"Handling Object at index {self.next_planning_object_idx}")
        obj :KnownObject = self.known_obj_list.objects[self.next_planning_object_idx]
        # We've pick the plant, increment counter
        self.next_planning_object_idx +=1 # Move on to the next one.


        if obj.object_class != 0:
            self.get_logger().warn(f"Skipping Object typeof type {obj.object_class} ")
            return self.CmdStates.PLANT_SELECTION

        self.current_chasing_obj = obj

        # TODO We skip ARM_TURNING for now.
        self.get_logger().warn(f"Skil to arm turning")

        return self.CmdStates.ARM_TURNING


    async def planning_state(self) -> CmdStates:
        if not self.StartupCheck():
            return self.CmdStates.PLANNING




        object_world_loc = self.current_chasing_obj.space_loc
        if object_world_loc.header.frame_id != self.map_helper.map.header.frame_id:
            # TODO we just don't handle this at all
            self.get_logger().error(
                f"Target Loc frame {object_world_loc.header.frame_id} not in same frame as map {self.map_helper.map.header.frame_id}! "
            )
            raise ValueError(
                f"Target Loc frame {object_world_loc.header.frame_id} not in same frame as map {self.map_helper.map.header.frame_id}! "
            )


        # Search go out as a circle.
        # every time the potential points are emptied, search radius increase and fill the list again.
        object_map_coord = self.map_helper.world_point_to_map(object_world_loc.point)
        # planning_xy = (object_world_loc.point.x, object_world_loc.point.y)
        potential_loc_xy = deque()
        last_search_radius = 0 # integer unit in map grid number
        while True:
            # Need to add more search-able locations. We add a "ring" at a time.
            if len(potential_loc_xy) ==0:
                self.get_logger().warn(f"At search radius {last_search_radius} around {object_world_loc.point} Did not get valid goal")
                last_search_radius += 1

                # A failure termination condition
                if (last_search_radius * self.map_helper.map_info.resolution) > self.max_plan_offset_radius:
                    self.get_logger().error(
                        f"MAX Plan offset radius reached for {object_world_loc.point}\n"
                        f"Last tried radius: cell: {last_search_radius} world: {last_search_radius * self.map_helper.map_info.resolution}")
                    return self.CmdStates.PLANT_SELECTION

                maybe_map_coords = self.map_helper.GetRing(
                    object_map_coord, last_search_radius)

                check_cell_rad = math.ceil(self.plan_dest_clearance_radius / self.map_helper.map_info.resolution)
                self.get_logger().error(f"Clearance cell number {check_cell_rad}")

                for coord in maybe_map_coords:

                    maybe_valid , checked_coords = self.map_helper.CheckEmptyCircle(coord, check_cell_rad)
                    # debug_marker = self.map_helper.color_sphere_gen(checked_coords ,id = 5 , color = COLOR_MSG_LIST_RGBW[1] )
                    # print(f"checked_coords size {len(checked_coords)}")
                    # self.pub_marker(debug_marker)
                    if maybe_valid:
                        potential_loc_xy.append( self.map_helper.map_loc_to_world(coord) )

                self.get_logger().warn(f"Found {len(potential_loc_xy)} cells to check at ring of radius {last_search_radius}")
                continue # Just in case we didn't add any valid cell
            # Try to see if this location is plan-able by nav2.
            planning_xy = potential_loc_xy.popleft()


            # The loop is put into this order so planning_xy is pre-defined and fetch-able by outside.
            loc_x, loc_y = planning_xy
            pose_goal = self.make_ComputePathToPose_goal(loc_x, loc_y)
            # pose_goal.goal.pose.position.z = loc.point.z

            goal_marker = MakeCylinderMarker(id = 10, pos= pose_goal.goal.pose.position , header=pose_goal.goal.header, )
            self.pub_marker(goal_marker)

            self.get_logger().info(f"Sending goal at {pose_goal.goal.pose.position}")

            result : ComputePathToPose.Result
            status , result = await self.ActionSendAwait(pose_goal , self.compute_path_client)
            if status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().warn(f"Did not get successful goal result, got {status}" )
            else:
                # This is the successful break condition.
                break


        # End of loop

        near_end_pose_stamped:  PoseStamped = result.path.poses[-2]
        self.successful_planned_pose = near_end_pose_stamped.pose
        self.successful_planned_pose.position.x = planning_xy[0]
        self.successful_planned_pose.position.y = planning_xy[1]

        self.get_logger().info(f"Got a planned path to {planning_xy} , planning_time {result.planning_time} ")
        self.get_logger().info(f"Recording with pose {self.successful_planned_pose}")

        # TODO record something for MOVING state to do.
        # We actually just record this successful point and ask nav2 to replay.

        # After planning move to executing
        return self.CmdStates.MOVING

    async def moving_state(self) -> CmdStates:

        # First put robot in nav mode
        self.get_logger().info(f"switching to nav mode")
        ret = await self.switch_nav_service.call_async(TriggerSrv.Request())
        self.get_logger().info(f"ret from switch nav {ret} ")



        nav_goal = NavigateToPose.Goal()
        nav_goal.pose.header.frame_id = self.world_frame
        nav_goal.pose.header.stamp = self.get_clock().now().to_msg()
        nav_goal.pose.pose = self.successful_planned_pose
        nav_goal.pose.pose = self.successful_planned_pose
        for i in range(3):
            result : NavigateToPose.Result
            status, result = await self.ActionSendAwait(
                nav_goal,
                self.navigate_to_pose_client,
                # print_fb,
            )
            if status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().warn(f"Did not get successful goal result, got {status}" )
            else:
                # This is the successful break condition.
                break
        else:
            self.get_logger().error(f"Repeated Failure trying to nav to {nav_goal.pose.pose.position}")

        return self.CmdStates.PAUSE

    async def arm_pointing_state(self) -> CmdStates:
        # TODO find the watering area in frame.
        # Just use the plant location for now.
        self.get_logger().warn("switching to pos mode ")
        # ret = await self.switch_traj_service.call_async(TriggerSrv.Request())
        ret = await self.switch_pos_service.call_async(TriggerSrv.Request())
        self.get_logger().info(f"ret from switch pos {ret} ")


        loc_world = self.current_chasing_obj.space_loc


        # Make sure we centered the wrist

        status, result = await self.move_multi_joint(
            ["joint_wrist_yaw", "joint_wrist_pitch", "joint_wrist_roll"], [0, 0, 0], duration=2.5)
        if (result.error_code == FollowJointTrajectory.Result.SUCCESSFUL):
            self.get_logger().info("Successful! ")
        else:
            self.get_logger().error(f"FAILED! {result} ")

        # And also retract the arm

        # We first do a arm lift
        status, result = await self.move_multi_joint(
            ["joint_arm_l3", "joint_arm_l2", "joint_arm_l1" ,"joint_arm_l0"], [0, 0,0,0], duration=2.5)

        
        # status, result = await self.move_single_joint(
        #     "wrist_extension",0.05, duration=2.5)
        if (result.error_code == FollowJointTrajectory.Result.SUCCESSFUL):
            self.get_logger().info("Successful! ")
        else:
            self.get_logger().error(f"FAILED! {result} ")
            raise

        # with the grap flatten, rise the lift up to slightly above target height.
        
        for i in range(5):
            loc_in_ee = self.try_transform(loc_world, self.ee_frame)
            time.sleep(0.5)
            if loc_in_ee is not None:
                break
        else:
            self.get_logger().info(f"Time out getting transform")
            raise
        self.get_logger().info(f"Converted loc into ee frame {loc_in_ee}")

        status, result = await self.move_single_joint_delta( "joint_lift" , loc_in_ee.point.z + 0.1 , duration = 2.0)

        if (result.error_code == FollowJointTrajectory.Result.SUCCESSFUL):
            self.get_logger().info("Successful! ")
        else:
            self.get_logger().error(f"FAILED! {result} ")
            raise






        

        # T_world_base = self._tf_buffer.lookup_transform(self.robot_baseframe, self.world_frame,timeout= Duration(sec=1.0) )
        # Ros tf bug?
        #   File "/opt/ros/humble/lib/python3.10/site-packages/tf2_ros/buffer.py", line 220, in can_transform
        #     while (clock.now() < start_time + timeout and
        # TypeError: unsupported operand type(s) for +: 'Time' and 'Duration'

        # loc_base = self._tf_buffer.transform(loc_world, self.world_frame,timeout= Duration(sec=1))




        # # Dummy move to set motor in right state
        # traj_goal = FollowJointTrajectory.Goal()
        # traj_goal.multi_dof_trajectory = self.build_rot_traj(0.2 , total_duration= 0.7)
        # result : FollowJointTrajectory.Result
        # status, result = await self.ActionSendAwait(
        #     traj_goal,
        #     self.trajectory_action_client,
        # )

        # loc_base = self._tf_buffer.transform(loc_world, self.robot_baseframe)
        for i in range(5):
            loc_in_ee = self.try_transform_tf(loc_world, self.ee_frame)
            time.sleep(0.5)
            if loc_in_ee is not None:
                break
        else:
            self.get_logger().info(f"Time out getting transform")
            raise
        self.get_logger().info(f"Converted loc into ee frame {loc_in_ee}")


        self.get_logger().info(f"Target's location in base frame is {loc_base}")
        self.marker_pub.publish(MakeCylinderMarker(id=101 , pos=loc_base.point , header= Header(frame_id=self.robot_baseframe),diameter=0.08))

        # Then we compute the angle for turning.
        base_diff_angle = math.atan2(loc_base.point.y , loc_base.point.x)
        base_diff_angle_90 = normaliz_angle(base_diff_angle + math.pi/2)

        for i in range(5):


            self.get_logger().info(f"Base turning angle is {base_diff_angle}")

            traj_goal = FollowJointTrajectory.Goal()
            traj_goal.multi_dof_trajectory = self.build_rot_traj(base_diff_angle_90)

            result : FollowJointTrajectory.Result
            status, result = await self.ActionSendAwait(
                traj_goal,
                self.trajectory_action_client,
            )
            self.get_logger().info(f"Result code: {result.error_code} , Result string: {result.error_string}")
            if (result.error_code == FollowJointTrajectory.Result.SUCCESSFUL):
                self.get_logger().info("Successful! ")

            if status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().warn(f"Did not get successful goal result, got {status}" )

            loc_base = self._tf_buffer.transform(loc_world, self.world_frame)
            self.get_logger().info(f"Target's location in base frame is {loc_base}")
            self.marker_pub.publish(MakeCylinderMarker(id=101 , pos=loc_base.point , header= Header(frame_id=self.robot_baseframe),diameter=0.08))


            # Then we compute the angle for turning.
            base_diff_angle = math.atan2(loc_base.point.y , loc_base.point.x)
            base_diff_angle_90 = normaliz_angle(base_diff_angle + math.pi/2)

            if base_diff_angle_90 < 0.01:
                self.get_logger().info(f"Arm is pointing roughly towards the plants.")
                break
        else:
            self.get_logger().info(f"Failed to turn base into target heading.")
            raise





        return self.CmdStates.PLANT_SELECTION

    def build_rot_traj(self ,target_angle : float , angular_vel = 0.2 , total_duration = None ) -> MultiDOFJointTrajectory:

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

    def build_duration(self,sec:float) ->DurationMsg:
        return DurationMsg(sec = int(sec) ,nanosec = int((sec%1) * 1e9 ) )
    def build_turning_tf(self,angle)->Transform:
        goal_tf = Transform()
        q = tf_transformations.quaternion_from_euler(0,0,angle)
        goal_tf.rotation.x = q[0]
        goal_tf.rotation.y = q[1]
        goal_tf.rotation.z = q[2]
        goal_tf.rotation.w = q[3]
        return goal_tf

    async def move_single_joint_delta(self , joint_name :str , delta : float , duration = 1.0):
        actual_target = self.js_map[joint_name] + delta
        return await self.move_single_joint(joint_name , actual_target, duration = 1.0)


    async def move_single_joint(self,joint_name :str , target : float , duration = 1.0):

        traj_goal = FollowJointTrajectory.Goal()

        # Add current location in
        traj_goal.trajectory.joint_names.append(joint_name)

        # if joint_name == "wrist_extension":
        #     current_joint_value = self.get_wrist_extension_js()



        j_p = JointTrajectoryPoint()
        j_p.positions.append(self.js_map[joint_name])
        j_p.time_from_start = self.build_duration(0.0)
        traj_goal.trajectory.points.append(j_p)

        j_p = JointTrajectoryPoint()
        j_p.positions.append(target)
        j_p.time_from_start = self.build_duration(duration)
        traj_goal.trajectory.points.append(j_p)


        result: FollowJointTrajectory.Result
        status, result = await self.ActionSendAwait(
            traj_goal,
            self.trajectory_action_client,
        )
        self.get_logger().info(
            f"Result code: {result.error_code} , Result string: {result.error_string}")
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f"Did not get successful goal result, got {status}")

        return status,result

    async def move_multi_joint(self,joint_names :str , targets : float , duration = 1.0):

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
        self.get_logger().info(
            f"Result code: {result.error_code} , Result string: {result.error_string}")
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f"Did not get successful goal result, got {status}")

        return status,result

    def apply_transform(self,point:PointStamped, transform:TransformStamped):
        # Extract the translation from the Transform message
        translation = np.array([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z])
        
        # Extract the rotation (quaternion) from the Transform message
        rotation = [transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w]
        
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

    def try_transform_tf(self,point_stamp : PointStamped , target_frame):
        try:
            return self._tf_buffer.transform(point_stamp , target_frame)
        except () as ex:
            self.get_logger().warn(f"failed to get transform, {ex}")
            return None


    def try_transform(self,point_stamp : PointStamped , target_frame):
        
        
        try:
            tf = self._tf_buffer.lookup_transform(
                target_frame=target_frame,
                source_frame=point_stamp.header.frame_id,
                time=rclpy.time.Time())
            return self.apply_transform(point_stamp , tf)
        except () as ex:
            self.get_logger().warn(f"failed to get transform, {ex}")
            return None


    def get_wrist_extension_js(self):
        return self.js_map["joint_arm_l3"]+self.js_map["joint_arm_l2"]+self.js_map["joint_arm_l1"]+self.js_map["joint_arm_l0"]

def main(args=None):
    rclpy.init(args=args)

    try:
        node = GoalMover()
        exec = MultiThreadedExecutor(5)
        exec.add_node(node)
        exec.spin()
        # rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('interrupt received, so shutting down')

    rclpy.shutdown()



if __name__ == "__main__":
    main()
