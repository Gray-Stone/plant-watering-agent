#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import copy
import dataclasses
import enum
import time
from enum import Enum
from typing import Optional
from action_msgs.msg import GoalStatus

import message_filters
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from builtin_interfaces.msg import Time as RosTime
from geometry_msgs.msg import Point, PointStamped, TransformStamped, Vector3
from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
# from sensor_msgs.msg import CameraInfo, Image as ImageMsg
from std_msgs.msg import ColorRGBA, Header
from std_msgs.msg import String as StringMsg
from stretch_mover.msg import (KnownObject, KnownObjectList, YoloDetection,
                               YoloDetectionList)
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker, MarkerArray
import asyncio

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class GoalMover(Node):


    class CmdStates(Enum):
        IDLE = enum.auto()
        PLANNING = enum.auto()
        MOVING = enum.auto()
        PAUSE = enum.auto()


    def pub_marker(self , m:Marker) :
        self.marker_pub.publish(m)
    def gen_pose_goal(self):
        pose_goal = ComputePathToPose.Goal()

        pose_goal.goal.header.frame_id = 'map'
        pose_goal.goal.pose.position.x = 1.0
        pose_goal.goal.pose.position.y = 1.0
        # Must fill in the planner id, or it won't print the action failed reason.
        pose_goal.planner_id = "GridBased"
        # TODO add orientation
        pose_goal.use_start = False
        return pose_goal

    def __init__(self):
        super().__init__("move_to_plants")


        self.state = self.CmdStates.IDLE

        action_cb_group = MutuallyExclusiveCallbackGroup()
        #### Action client
        self.compute_path_client = ActionClient(self,ComputePathToPose , "compute_path_to_pose",callback_group=action_cb_group)
        self.navigate_to_pose_client = ActionClient(self,ComputePathToPose , "/navigate_to_pose")

        self.waiting_planning = False


        ## main timer
        self.create_timer(0.1,self.main_timer)
        self.get_logger().info("Node configured, timer created!")

        self.state_update(self.CmdStates.PLANNING)


    def state_update(self , new_state:'CmdStates'):
        if new_state == self.state:
            return

        self.get_logger().warn(f"State changing from {self.state} to {new_state}")
        self.state = new_state
        return

    async def main_timer(self):
        planning_outcome  = await self.planning_state_await()
        self.state_update(planning_outcome)



    def get_result_callback(self, future):
        res = future.result()

        if res.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Did not get successful goal result, got {res.status}" )
        result : ComputePathToPose.Result = res.result
        self.get_logger().info(f"Planning result {result}")
        self.planning_result = result

    def goal_response_callback(self, future):
        self.get_logger().error("In CB! ")
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)


    async def planning_state_await(self) -> CmdStates : 

        pose_goal = self.gen_pose_goal()
        self.get_logger().info(f"About to Sending goal")

        goal_handle = await self.compute_path_client.send_goal_async(
                pose_goal)
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        return self.CmdStates.MOVING



    async def planning_state_cb_style(self) -> CmdStates:
        # effectively skip to next cycle if these are none



        if not self.waiting_planning:
            pose_goal = self.gen_pose_goal()
            self.get_logger().info(f"Sending goal {pose_goal}")





            self.goal_future = self.compute_path_client.send_goal_async(
                pose_goal)
            self.goal_future.add_done_callback(self.goal_response_callback)
            self.waiting_planning = True
            self.planning_result = None
            return self.CmdStates.PLANNING

        if self.planning_result is None:
            return self.CmdStates.PLANNING

        self.get_logger().info(f"Planning result {self.planning_result}")


        
        # while not self.goal_future.done():
        #     self.get_logger().warn(f"Waiting for future {self.goal_future}")
        #     return self.CmdStates.PLANNING


        # goal_handle: ClientGoalHandle =  self.goal_future.result()
        # self.get_logger().warn(f"Goal handle returned")
        # self.get_logger().warn(f"type of goal_handle is {type(goal_handle)}, itself is \n{goal_handle}")
        # self.waiting_planning = False



        # if not goal_handle.accepted:
        #     raise ValueError("Not accepting goal!")

        # res :ComputePathToPose.Result = await goal_handle.get_result_async()
        
        # self.get_logger().warn(f"type of res is {type(res)}, itself is \n{res}")

        # if res.status != GoalStatus.STATUS_SUCCEEDED:
        #     raise RuntimeError(f"Did not get successful goal result, got {res.status}" )
        # result : ComputePathToPose.Result = res.result
        # self.get_logger().info(f"Planning result {result}")

        # # TODO record something for MOVING state to do.

        # # After planning move to executing

        return self.CmdStates.MOVING



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
