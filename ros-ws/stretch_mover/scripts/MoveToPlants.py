#! /usr/bin/env python3

import copy
import dataclasses
import enum
import time
from enum import Enum
from typing import Optional
from action_msgs.msg import GoalStatus

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from builtin_interfaces.msg import Time as RosTime
from geometry_msgs.msg import Point, PointStamped, TransformStamped, Vector3 , PoseStamped, Pose
from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose
from nav_msgs.msg import MapMetaData, OccupancyGrid
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

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from collections import deque

COLOR_MSG_LIST_RGBW = [
    ColorRGBA(r=1.0,b=0.0,g=0.0,a=1.0),
    ColorRGBA(r=0.0,b=0.0,g=1.0,a=1.0),
    ColorRGBA(r=0.0,b=1.0,g=0.0,a=1.0),
    ColorRGBA(r=1.0,b=1.0,g=1.0,a=1.0),
]


class OccupancyGridHelper():
    def __init__(self , map_msg: OccupancyGrid ):
        self.map = map_msg
        self.map_info = map_msg.info

    def world_point_to_map(self , pose: Point):
        return self.world_loc_to_map( (pose.x,pose.y) ,)

    def world_loc_to_map(self , world_loc: tuple[float,float]):
        # According to doc, origin is lower left corner.
        # The map frame is x to right, y up.
        x,y = world_loc
        x_off = x - self.map_info.origin.position.x
        y_off = y - self.map_info.origin.position.y

        index_x = int(x_off / self.map_info.resolution)
        index_y = int(y_off / self.map_info.resolution)

        return index_x , index_y


    def map_loc_to_world(self,map_loc : tuple[int,int])->tuple[float,float]:
        mx , my = map_loc

        point_x = mx * self.map_info.resolution + self.map_info.origin.position.x
        point_y = my * self.map_info.resolution + self.map_info.origin.position.y
        # This will give it's lower left corner. so need to shift it by half a cell size
        point_x += self.map_info.resolution/2
        point_y += self.map_info.resolution/2
        return ( point_x , point_y)
    def map_loc_to_world_point(self,map_loc : tuple[int,int]) -> Point:
        wx,wy = self.map_loc_to_world(map_loc)
        return (Point(x=wx,y=wy))

    def ValidLoc(self,map_loc: tuple[int,int]):
        x,y = map_loc
        if x <0 or x>= self.map_info.width:
            return False
        if y<0 or y>= self.map_info.height:
            return False
        return True

    def Get4Neighbor(self, map_loc: tuple[int, int]) -> list[tuple[int, int]]:
        # TODO change this by add each after boundary check
        x, y = map_loc
        valid_nbr = []
        for loc in [
            (x + 1, y),
            (x - 1, y),
            (x, y + 1),
            (x, y - 1),
        ]:
            if self.ValidLoc(loc):
                valid_nbr.append(loc)

        return valid_nbr

    def GetRing(self,center_loc:tuple[int,int] , ring_radius:int) -> list[tuple[int,int]]:
        cx,cy = center_loc
        ring_list = []
        for dx in range (-ring_radius , ring_radius +1):
            for dy in range (-ring_radius , ring_radius +1):
                if int(round(np.sqrt(dx**2 + dy**2))) == ring_radius:
                    potential_xy = [cx+dx ,cy+dy]
                    if self.ValidLoc(potential_xy):
                        ring_list.append(potential_xy)
        return ring_list



    def get_data(self, map_loc: tuple[int,int])-> Optional[int]:
        if not self.ValidLoc(map_loc):
            return None
        x,y = map_loc
        index = y * self.map_info.width + x
        return self.map.data[index]

    def color_sphere_gen(self,map_locs:list[tuple[int,int]] , scale = 0.1)->Marker:
        m = Marker()
        m.scale = Vector3(x=scale,y=scale,z=scale)
        m.type = Marker.SPHERE_LIST
        m.header = self.map.header
        for loc in map_locs:
            data = self.get_data(loc)
            m.points.append(self.map_loc_to_world_point(loc))

            if data == -1:
                m.colors.append(COLOR_MSG_LIST_RGBW[2])
            elif data ==0 :
                m.colors.append(COLOR_MSG_LIST_RGBW[3])
            elif data>99:
                m.colors.append(COLOR_MSG_LIST_RGBW[0])
            else:
                c = COLOR_MSG_LIST_RGBW[2]
                c.r = data / 100
                m.colors.append(c)
        return m


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

    PAUSE_DURATION = 3.0

    class CmdStates(Enum):
        IDLE = enum.auto()
        PLANNING = enum.auto()
        MOVING = enum.auto()
        PAUSE = enum.auto()


    def pub_marker(self , m:Marker):
        self.marker_pub.publish(m)

    def __init__(self):
        super().__init__("move_to_plants")

        #### Parameters
        self.declare_parameter("pot_class_id" , int(1))
        # world frame will also be the map's frame.
        self.declare_parameter("world_frame" , 'map')
        self.declare_parameter("known_object_topic", "yolo_ros/known_objects")

        self.pot_class_id = self.get_parameter("pot_class_id").get_parameter_value().integer_value
        self.world_frame = self.get_parameter("world_frame").get_parameter_value().string_value
        known_object_topic = self.get_parameter("known_object_topic").get_parameter_value().string_value


        #### Member Vars
        self.map_helper :OccupancyGridHelper = None
        self.known_obj_list: KnownObjectList = None
        self.next_planning_object_idx = 0
        self.current_chasing_obj : KnownObject = None
        self.successful_planned_pose: Pose = None
        self.max_plan_offset_radius = 0.48 # arm length is 0.52

        self.state = self.CmdStates.IDLE

        # Time keeper for the pause state.
        self.pause_start_time = time.time()

        #### Publisher
        self.state_change_publisher = self.create_publisher(StringMsg , "move_plant_state_change" , 2)
        self.marker_pub = self.create_publisher(Marker, "VisualizationMarker" , 1)

        #### Service client
        # /switch_to_navigation_mode [std_srvs/srv/Trigger]
        # /switch_to_position_mode [std_srvs/srv/Trigger]
        # /switch_to_trajectory_mode [std_srvs/srv/Trigger]
        service_client_cb_group = MutuallyExclusiveCallbackGroup()

        self.switch_nav_service = self.create_client(TriggerSrv,"switch_to_navigation_mode" ,callback_group=service_client_cb_group)

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
        # Subscribers
        self.map_subs = self.create_subscription(OccupancyGrid, "/map" , self.global_map_cb , 1)
        # self.map_subs = self.create_subscription(OccupancyGrid, "/local_costmap/costmap" , self.global_map_cb , 1)
        self.known_obj_subs  = self.create_subscription(  KnownObjectList , known_object_topic , self.known_obj_cb , 1)

        ## main timer
        self.create_timer(0.5,self.main_timer)
        self.get_logger().info("Node configured, timer created!")

        self.state_update(self.CmdStates.PLANNING)


    def global_map_cb(self,map_msg : OccupancyGrid):
        # print(f"\n\n ================== ")
        # print(f"Got map , header {map_msg.header} , info {map_msg.info}")

        self.map_helper = OccupancyGridHelper(map_msg)
        # Map data: -1 unknown, 0 free, 100 occupied.
        # For debug purpose.
        # self.pub_marker(map_marker_check(self.map_helper))
    def known_obj_cb(self, msg: KnownObjectList):
        self.known_obj_list = msg

    def state_update(self , new_state:'CmdStates'):
        if new_state == self.state:
            return

        self.get_logger().warn(f"State changing from {self.state} to {new_state}")
        self.state = new_state
        return

    async def main_timer(self):
        # This is the main logic.
        if self.state == self.CmdStates.IDLE:
            # Do nothing in idle state.
            return

        elif self.state == self.CmdStates.PAUSE:
            if time.time() - self.pause_start_time > self.PAUSE_DURATION:
                # self.get_logger().info("Pause timed up")
                self.state_update(self.CmdStates.PLANNING)
            return

        elif self.state == self.CmdStates.PLANNING:
            self.state_update(await self.planning_state())

        elif self.state == self.CmdStates.MOVING:
            self.state_update(await self.moving_state())

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

    async def planning_state(self) -> CmdStates:
        # effectively skip to next cycle if these are none
        if self.map_helper is None:
            self.get_logger().warn(f"Skipping cycle for missing map")
            return self.CmdStates.PLANNING
        if self.known_obj_list is None:
            self.get_logger().warn(f"Skipping cycle for missing known objects")
            return self.CmdStates.PLANNING

        if self.next_planning_object_idx >= len(self.known_obj_list.objects):
            self.get_logger().warn(f"{len(self.known_obj_list.objects)} locations all visited!")
            return self.CmdStates.IDLE

        obj :KnownObject = self.known_obj_list.objects[self.next_planning_object_idx]

        self.get_logger().warn(f"Processing Object at index {self.next_planning_object_idx}")

        if obj.object_class != 0  :
            self.get_logger().warn(f"Skipping Object typeof type {obj.object_class} ")
            self.next_planning_object_idx +=1 # Move on to the next one.
            return self.CmdStates.PLANNING

        self.current_chasing_obj = obj
        object_world_loc = obj.space_loc
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
        planning_xy = (object_world_loc.point.x, object_world_loc.point.y)
        potential_loc_xy = deque()
        last_search_radius = 0 # integer unit in map grid number
        while True:
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

            # Need to add more search-able locations. We add a "ring" at a time.
            if len(potential_loc_xy) ==0:
                self.get_logger().warn(f"At search radius {last_search_radius} around {object_world_loc.point} Did not get valid goal")
                last_search_radius += 1

                # A failure termination condition
                if last_search_radius / self.map_helper.map_info.resolution > self.max_plan_offset_radius:
                    self.get_logger().error(
                        f"MAX Plan offset radius reached for {object_world_loc.point}"
                        f" Skipping object at index {self.next_planning_object_idx}")
                    self.next_planning_object_idx +=1 # Move on to the next one.
                    return self.CmdStates.PLANNING

                maybe_map_coords = self.map_helper.GetRing(
                    object_map_coord, last_search_radius)
                for coord in maybe_map_coords:
                    # Only checking cell is free here. Actually reach-ability is only check-able by planner.
                    if self.map_helper.get_data(coord) < 95 :
                        potential_loc_xy.append( self.map_helper.map_loc_to_world(coord) )

                self.get_logger().warn(f"Found {len(potential_loc_xy)} cells to check at ring of radius {last_search_radius}")

            # Try to see if this location is plan-able by nav2.
            planning_xy = potential_loc_xy.popleft()

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
        self.next_planning_object_idx +=1 # increment planning counter
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
            def print_fb(fb_msg):
                self.get_logger().info(f"Getting feedback {fb_msg}")

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

def main(args=None):
    rclpy.init(args=args)

    try:
        node = GoalMover()
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('interrupt received, so shutting down')

    rclpy.shutdown()



if __name__ == "__main__":
    main()
