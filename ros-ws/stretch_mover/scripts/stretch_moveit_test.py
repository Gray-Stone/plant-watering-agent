
import copy
import dataclasses
import enum
import time
from enum import Enum
from typing import Optional
from action_msgs.msg import GoalStatus

import math

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

from stretch_mover_utils.grid_utils import OccupancyGridHelper , COLOR_MSG_LIST_RGBW

from octomap_msgs.msg import OctomapWithPose , Octomap
from moveit_msgs.msg import PlanningSceneWorld


class OctoMapRelay(Node):
    def __init__(self):

        super().__init__("Octomap_scene_relay")

        self.declare_parameter("world_frame" , 'map')

        self.world_frame = self.get_parameter("world_frame").get_parameter_value().string_value

        self.planning_scene_world_pub =  self.create_publisher(PlanningSceneWorld , "planning_scene_world" , 1)
                
        # /octomap_binary [octomap_msgs/msg/Octomap] to /planning_scene_world

        # self.create_subscription(Octomap , "octomap_binary" , self.octomap_cb ,1 )
        self.create_subscription(Octomap , "octomap_full" , self.octomap_cb ,1 )




    def octomap_cb(self , octomap_msg : Octomap):
        print(f"header {octomap_msg.header }")
        print(f"binary {octomap_msg.binary }")
        print(f"id {octomap_msg.id  }")
        print(f"resolution {octomap_msg.resolution }")

        scene_world = PlanningSceneWorld()
        scene_world.octomap.header.frame_id = self.world_frame
        scene_world.octomap.header.stamp = self.get_clock().now().to_msg()
        scene_world.octomap.octomap = octomap_msg
        self.planning_scene_world_pub.publish(scene_world)

        

if __name__ == "__main__":
    
    rclpy.init()
    node = OctoMapRelay()
    rclpy.spin(node)
