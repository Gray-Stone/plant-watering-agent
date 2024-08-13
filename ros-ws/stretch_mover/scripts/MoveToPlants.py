#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import copy
import dataclasses

from typing import Optional

import cv2
import cv_bridge
import message_filters
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from builtin_interfaces.msg import  Time as RosTime

from geometry_msgs.msg import Point, PointStamped, TransformStamped , Vector3
from image_geometry import PinholeCameraModel
from message_filters import ApproximateTimeSynchronizer
from rclpy.node import Node
# from sensor_msgs.msg import CameraInfo, Image as ImageMsg
from std_msgs.msg import (String as StringMsg , ColorRGBA, Header)

from stretch_mover.msg import YoloDetection, YoloDetectionList , KnownObject , KnownObjectList
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from ultralytics import YOLO
from vision_msgs.msg import (Detection2D, Detection2DArray,
                             ObjectHypothesisWithPose)
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.action import ActionClient

from nav2_msgs.action import ComputePathToPose, FollowPath , NavigateToPose
from nav_msgs.msg import OccupancyGrid , MapMetaData

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

class GoalMover(Node):

    def pub_marker(self , m:Marker) : 
        self.marker_pub.publish(m)


    def __init__(self):
        super().__init__("move_to_plants")

        #### Parameters
        self.declare_parameter("known_object_topic" , "yolo_known_objects")
        self.declare_parameter("pot_class_id" , int(1))

        self.known_obj_topic = self.get_parameter("known_object_topic").get_parameter_value().string_value
        self.pot_class_id = self.get_parameter("pot_class_id").get_parameter_value().integer_value

        #### Publisher
        self.state_change_publisher = self.create_publisher(StringMsg , "move_plant_state_change" , 2)
        self.marker_pub = self.create_publisher(Marker, "VisualizationMarker" , 1)

        #### Action client
        self.compute_path_client = ActionClient(self,ComputePathToPose , "compute_path_through_poses")

        # Subscribers
        self.map_subs = self.create_subscription(OccupancyGrid, "/map" , self.global_map_cb , 1)
        # self.map_subs = self.create_subscription(OccupancyGrid, "/local_costmap/costmap" , self.global_map_cb , 1)


    def global_map_cb(self,map_msg : OccupancyGrid):
        print(f"\n\n ================== ")
        print(f"Got map , header {map_msg.header} , info {map_msg.info}")
        self.map_helper = OccupancyGridHelper(map_msg)

        self.oc_map = map_msg

        # Map data: -1 unknown, 0 free, 100 occupied.

        self.pub_marker(map_marker_check(self.map_helper))







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
