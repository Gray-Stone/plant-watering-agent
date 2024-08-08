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
from sensor_msgs.msg import Image
from ultralytics import YOLO
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from ultralytics_ros.msg import YoloResult
from stretch_mover.msg import YoloDetection , YoloDetectionList
from std_msgs.msg import Header



class TrackerNode(Node):
    def __init__(self):
        super().__init__("tracker_node")
        self.declare_parameter("yolo_model", "yolov8n.pt")
        self.declare_parameter("input_topic", "image_raw")
        self.declare_parameter("yolo_result_topic", "yolo_ros/detect_result")
        self.declare_parameter("result_image_topic", "yolo_ros/result_image")
        self.declare_parameter("conf_thres", 0.25)
        self.declare_parameter("iou_thres", 0.45)
        self.declare_parameter("max_det", 300)
        self.declare_parameter("classes", list(range(2)))
        self.declare_parameter("tracker", "bytetrack.yaml")
        self.declare_parameter("device", "cpu")
        self.declare_parameter("result_conf", True)
        self.declare_parameter("result_line_width", 1)
        self.declare_parameter("result_font_size", 1)
        self.declare_parameter("result_font", "Arial.ttf")
        self.declare_parameter("result_labels", True)
        self.declare_parameter("result_boxes", True)

        self.declare_parameter("debug", False)
        self.declare_parameter("verbose", False)
        
        self.last_seq = 0

        path = get_package_share_directory("ultralytics_ros")
        yolo_model = self.get_parameter("yolo_model").get_parameter_value().string_value
        # self.model = YOLO(f"{path}/models/{yolo_model}")
        self.model = YOLO(f"{yolo_model}")
        self.model.fuse()

        self.bridge = cv_bridge.CvBridge()

        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        yolo_result_topic = (
            self.get_parameter("yolo_result_topic").get_parameter_value().string_value
        )
        result_image_topic = (
            self.get_parameter("result_image_topic").get_parameter_value().string_value
        )
        self.debug = (
            self.get_parameter("debug").get_parameter_value().bool_value
        )
        self.verbose = (
            self.get_parameter("verbose").get_parameter_value().bool_value
        )
        self.get_logger().warn(f"yolo_result_topic: {yolo_result_topic}")

        self.results_pub = self.create_publisher(YoloDetectionList, yolo_result_topic, 1)
        self.result_image_pub = self.create_publisher(Image, result_image_topic, 1)

        if self.debug: 
            self.mask_debug_pub = self.create_publisher(Image,"/camera/color/combined_mask_debug" , 1)

        self.create_subscription(Image, input_topic, self.image_callback, 1)

    def image_callback(self, msg:Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        rot_cv_image = cv2.rotate(cv_image,cv2.ROTATE_90_CLOCKWISE)

        conf_thres = self.get_parameter("conf_thres").get_parameter_value().double_value
        iou_thres = self.get_parameter("iou_thres").get_parameter_value().double_value
        max_det = self.get_parameter("max_det").get_parameter_value().integer_value
        classes = (
            self.get_parameter("classes").get_parameter_value().integer_array_value
        )
        tracker = self.get_parameter("tracker").get_parameter_value().string_value
        device = self.get_parameter("device").get_parameter_value().string_value or None
        results = self.model.track(
            source=rot_cv_image,
            conf=conf_thres,
            iou=iou_thres,
            max_det=max_det,
            classes=classes,
            tracker=tracker,
            device=device,
            verbose=False,
            retina_masks=True,
        )

        if results is not None:
            detection_msg = self.create_detections_array(results , msg.header , cv2.ROTATE_90_COUNTERCLOCKWISE)
            if detection_msg is None:
                if self.verbose: 
                    self.get_logger().info(f"Detection got no mask, detected bbox number: {len(results[0].boxes)}" , throttle_duration_sec=3)
                return
            detection_msg.header =msg.header

            yolo_result_image_msg = self.create_result_image(results , cv2.ROTATE_90_COUNTERCLOCKWISE)
            yolo_result_image_msg.header = msg.header

            self.results_pub.publish(detection_msg)
            self.result_image_pub.publish(yolo_result_image_msg)

    def create_detections_array(self, results , mask_header: Header, rotate =None) -> Optional[YoloDetectionList]:
        """Put yolo result into YoloDetectionList message type

        Args:
            results (yolo.result): result object returned by yolo tracking
            mask_header (std_msgs.msg.header): header of source image, will be applied to mask as well
            rotate (cv.rotateCode, optional): CV rotation option, to rotate the mask back. Defaults to None.

        Returns:
            Optional[YoloDetectionList]: YoloDetectionList object from result, None if no mask or classes are detected.
        """
        classes = results[0].boxes.cls
        confidence_score = results[0].boxes.conf

        detection_list = YoloDetectionList()
        total_mask_list = []
        if results[0].masks is None:
            return None
        for cls, conf, mask_tensor in zip(classes, confidence_score, results[0].masks):
            det = YoloDetection()
            mask_numpy = (np.squeeze(mask_tensor.data.to("cpu").detach().numpy()).astype(np.uint8) *
                          255)
            if rotate is not None:
                mask_numpy = cv2.rotate(mask_numpy,rotateCode=rotate)
            if self.debug:
                total_mask_list.append(mask_numpy)
            mask_image_msg = self.bridge.cv2_to_imgmsg(mask_numpy, encoding="mono8")
            mask_image_msg.header = mask_header
            det.class_id = int(cls)
            det.score = float(conf)
            det.mask = mask_image_msg
            det.header = mask_header
            detection_list.detections.append(det)
        detection_list.header = mask_header

        if self.verbose:
            self.get_logger().info(f"detected {len(detection_list.detections)} classes", throttle_duration_sec=3)
        if self.debug: 
            # Create a combined mask for debugging.
            combined_mask = np.zeros(total_mask_list[0].shape[:2] , dtype = np.uint8)
            
            for msk in total_mask_list:
                combined_mask =np.bitwise_or(combined_mask ,msk)

            combined_mask_msg = self.bridge.cv2_to_imgmsg(combined_mask , encoding = "mono8")
            combined_mask_msg.header = mask_header
            self.mask_debug_pub.publish(combined_mask_msg)


        return detection_list 

    def create_result_image(self, results,rotate =None):
        result_conf = self.get_parameter("result_conf").get_parameter_value().bool_value
        result_line_width = (
            self.get_parameter("result_line_width").get_parameter_value().integer_value
        )
        result_font_size = (
            self.get_parameter("result_font_size").get_parameter_value().integer_value
        )
        result_font = (
            self.get_parameter("result_font").get_parameter_value().string_value
        )
        result_labels = (
            self.get_parameter("result_labels").get_parameter_value().bool_value
        )
        result_boxes = (
            self.get_parameter("result_boxes").get_parameter_value().bool_value
        )
        plotted_image = results[0].plot(
            conf=result_conf,
            line_width=result_line_width,
            font_size=result_font_size,
            font=result_font,
            labels=result_labels,
            boxes=result_boxes,
        )                
        plotted_image = cv2.rotate(plotted_image,rotateCode=rotate)

        result_image_msg = self.bridge.cv2_to_imgmsg(plotted_image, encoding="bgr8")
        return result_image_msg

def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
