#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ultralytics_ros
# Copyright (C) 2023-2024  Alpaca-zip
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Affero General Public License for more details.
#
# You should have received a copy of the GNU Affero General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
from typing import Optional
import cv2
import cv_bridge
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import Image , CameraInfo
from ultralytics import YOLO
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from stretch_mover.msg import YoloDetection , YoloDetectionList
from std_msgs.msg import Header , ColorRGBA
from builtin_interfaces.msg import Duration
from message_filters import ApproximateTimeSynchronizer
import message_filters
from visualization_msgs.msg import MarkerArray, Marker

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_py
from geometry_msgs.msg import TransformStamped , Point
from image_geometry import PinholeCameraModel



class DepthProcessor(Node):

    LIVE_DETECTION_MARKER_ID = 10
    CLASS_COLORS = [(0.1,0.1,0.9),
                    (0.1,0.9,0.1)]
    def __init__(self):
        super().__init__("basic_depth_process")

        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("aligned_depth_topic", "/camera/aligned_depth_to_color/image_raw")

        self.declare_parameter("yolo_result_topic", "yolo_ros/detections")
        self.declare_parameter("min_depth_range", 0.3)
        self.declare_parameter("max_depth_range", 3.0)

        self.declare_parameter("world_frame", "base_link")

        self.declare_parameter("classes", list(range(2)))
        self.declare_parameter("tracker", "bytetrack.yaml")

        self.declare_parameter("debug", False)
        self.declare_parameter("verbose", False)


        self.bridge = cv_bridge.CvBridge()

        camera_info_topic = (self.get_parameter("camera_info_topic").get_parameter_value().string_value)
        aligned_depth_topic = (self.get_parameter("aligned_depth_topic").get_parameter_value().string_value)
        yolo_result_topic = (self.get_parameter("yolo_result_topic").get_parameter_value().string_value)
        self.min_depth_range = ( self.get_parameter("min_depth_range").get_parameter_value().double_value)
        self.max_depth_range = ( self.get_parameter("max_depth_range").get_parameter_value().double_value)

        self.world_frame = (self.get_parameter("world_frame").get_parameter_value().string_value)

        self.debug = (self.get_parameter("debug").get_parameter_value().bool_value)
        self.verbose = (self.get_parameter("verbose").get_parameter_value().bool_value)

        self.get_logger().warn(f"yolo_result_topic: {yolo_result_topic}")
        self.get_logger().warn(f"depth_topic: {aligned_depth_topic}")

        self.bridge = cv_bridge.CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.marker_array_pub = self.create_publisher(MarkerArray, "yolo_depth_markers", 2)

        if self.debug:
            # self.mask_debug_pub = self.create_publisher(Image,"/camera/color/combined_mask_debug" , 1)
            cv2.namedWindow("basic_mask_debug" , cv2.WINDOW_NORMAL)

        # Make message filter and cb

        camera_info_sub = message_filters.Subscriber(self,CameraInfo, camera_info_topic, qos_profile= 3)
        depth_sub = message_filters.Subscriber(self,Image, aligned_depth_topic, qos_profile= 3)
        yolo_detect_sub = message_filters.Subscriber( self,YoloDetectionList, yolo_result_topic ,qos_profile = 5)

        self.depth_sync = ApproximateTimeSynchronizer([camera_info_sub, depth_sub, yolo_detect_sub],
                                                      queue_size=10,slop=0.05)
        self.depth_sync.registerCallback(self.depth_process_cb)

    def GetTF(self,target_frame:str, source_frame:str , time) -> Optional[TransformStamped] :
        try:
            t = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                time)
            return t
        except TransformException as ex:
            self.get_logger().warn(
                f'Could not transform {target_frame} to {source_frame}: {ex}')
            return None

    def AddPointToMarker(self  , sphere_list_marker:Marker , space_xyz ,color ):
        # TODO maybe check the point?
        sphere_list_marker.points.append(Point(x=space_xyz[0], y=space_xyz[1], z=space_xyz[2]))
        sphere_list_marker.colors.append(ColorRGBA(r=color[0], g=color[1], b=color[2], a=1.0))


    def depth_process_cb(self,camera_info_msg:CameraInfo , depth_msg: Image ,yolo_dec_list: YoloDetectionList):

        depth_world_tf =self.GetTF(self.world_frame , depth_msg.header.frame_id , depth_msg.header.stamp)
        if depth_world_tf is None:
            return
        if len(yolo_dec_list.detections) ==0:
            self.get_logger().error("Received Empty detection !")
            return

        camera_model = PinholeCameraModel()
        camera_model.fromCameraInfo(camera_info_msg)

        depth_image = self.bridge.imgmsg_to_cv2(depth_msg , desired_encoding=depth_msg.encoding)

        live_sphere_list_marker = Marker()
        live_sphere_list_marker.type = Marker.SPHERE_LIST


        if self.debug:
            fixed_masks = []
            centroid_xys = []
        if self.verbose:
            space_xyzs = []
        dec : YoloDetection
        for dec in yolo_dec_list.detections:
            mask_img = self.bridge.imgmsg_to_cv2(dec.mask , desired_encoding="mono8")

            # Do some basic process to help with image quality.
            kernel = np.ones((3, 3), np.uint8)
            opened_mask = cv2.morphologyEx(mask_img, cv2.MORPH_OPEN, kernel)
            closed_mask = cv2.morphologyEx(opened_mask, cv2.MORPH_CLOSE, kernel)
            dilated_mask = cv2.dilate(closed_mask,kernel,iterations = 1)

            m = cv2.moments(dilated_mask , binaryImage=True)

            px = int(m['m10'] / m['m00'])
            py = int(m['m01'] / m['m00'])

            if self.debug:
                fixed_masks.append(dilated_mask)
                centroid_xys.append((px,py))

            depth_at_centroid = depth_image[py,px] / 1000.0

            if depth_at_centroid < self.min_depth_range or depth_at_centroid>self.max_depth_range:
                if self.verbose:
                    self.get_logger().info(f"Drop centroid at uv {(px,py)} with depth {depth_at_centroid}")
                continue

            unit_xyz = camera_model.projectPixelTo3dRay((px,py))
            space_xyz = [ v * depth_at_centroid for v in unit_xyz ]
            # This is the spot in the space.

            self.AddPointToMarker(live_sphere_list_marker,space_xyz, self.CLASS_COLORS[dec.class_id])
            
            if self.verbose:
                space_xyzs.append(space_xyz)


            # Now we publish the point as visual message.

        live_sphere_list_marker.id = self.LIVE_DETECTION_MARKER_ID
        live_sphere_list_marker.header = depth_msg.header
        live_sphere_list_marker.scale.x = 0.03
        live_sphere_list_marker.scale.y = 0.03
        live_sphere_list_marker.scale.z = 0.03   
        live_sphere_list_marker.lifetime = Duration(sec=2)
        marker_array_msg = MarkerArray()
        marker_array_msg.markers.append(live_sphere_list_marker)

        self.marker_array_pub.publish(marker_array_msg)

        if self.verbose : 
            self.get_logger().info(f"Total of {len(space_xyzs)} space points")

        if self.debug and (len(fixed_masks) !=0):
            if self.verbose:     
                self.get_logger().info(f"Total of {len(centroid_xys)} centroid points")
            debug_mask = np.zeros(fixed_masks[0].shape , dtype = fixed_masks[0].dtype)
            for m in fixed_masks:
                debug_mask = cv2.bitwise_or(debug_mask , m)
            for c in centroid_xys:
                cv2.circle(debug_mask, c, 8, 120, thickness = -1)
            cv2.imshow("basic_mask_debug" , debug_mask)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = DepthProcessor()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
