
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

from 




if __name__ == "__main__":
    rclpy.init()
    logger = rclpy.logging.get_logger("moveit_py.pose_goal")
    mp = MoveItPy(node_name="moveit_py")

