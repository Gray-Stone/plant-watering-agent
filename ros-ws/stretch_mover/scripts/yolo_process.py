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
from stretch_mover.msg import YoloDetections
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
        self.declare_parameter("classes", list(range(80)))
        self.declare_parameter("tracker", "bytetrack.yaml")
        self.declare_parameter("device", "cpu")
        self.declare_parameter("result_conf", True)
        self.declare_parameter("result_line_width", 1)
        self.declare_parameter("result_font_size", 1)
        self.declare_parameter("result_font", "Arial.ttf")
        self.declare_parameter("result_labels", True)
        self.declare_parameter("result_boxes", True)

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
        self.create_subscription(Image, input_topic, self.image_callback, 1)
        self.results_pub = self.create_publisher(YoloDetections, yolo_result_topic, 1)
        self.result_image_pub = self.create_publisher(Image, result_image_topic, 1)

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
            yolo_result_image_msg = Image()
            yolo_result_image_msg.header = msg.header

            detection_msgs = self.create_detections_array(results , msg.header , cv2.ROTATE_90_CLOCKWISE)
            if detection_msgs is None:
                return
            detection_msgs.header =msg.header

            yolo_result_image_msg = self.create_result_image(results)
            self.results_pub.publish(detection_msgs)
            self.result_image_pub.publish(yolo_result_image_msg)

    def create_detections_array(self, results , mask_header: Header, rotate =None) -> Optional[YoloDetections]:
        classes = results[0].boxes.cls
        confidence_score = results[0].boxes.conf

        detections = YoloDetections()
        if results[0].masks is None:
            # self.get_logger().warn(f"Detection got no mask, detected classes: {classes} ")
            return None
        for cls, conf, mask_tensor in zip(classes, confidence_score, results[0].masks):
            mask_numpy = (np.squeeze(mask_tensor.data.to("cpu").detach().numpy()).astype(np.uint8) *
                          255)
            if rotate is not None:
                mask_numpy = cv2.rotate(mask_numpy,cv2.ROTATE_90_CLOCKWISE)

            mask_image_msg = self.bridge.cv2_to_imgmsg(mask_numpy, encoding="mono8")
            mask_image_msg.header = mask_header
            detections.class_ids.append(int(cls))
            detections.score.append(float(conf))
            detections.masks.append(mask_image_msg)
        self.get_logger().info(f"detected {len(detections.class_ids)} classes")
        return detections

    def create_result_image(self, results):
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
        result_image_msg = self.bridge.cv2_to_imgmsg(plotted_image, encoding="bgr8")
        return result_image_msg

def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
