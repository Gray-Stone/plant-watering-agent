#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vision_msgs/msg/detection3_d.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <stretch_mover/msg/yolo_detections.hpp>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

using ApproximateSyncPolicyTyped = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::CameraInfo, sensor_msgs::msg::Image, stretch_mover::msg::YoloDetections>;

class DepthMaskProecss : public rclcpp::Node {

private:
  // member variables

  std::string world_frame_; // The frame we assume detected object will be static with.
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_image_sub_;
  message_filters::Subscriber<stretch_mover::msg::YoloDetections> yolo_detection_subs_;

  std::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicyTyped>> sync_;

  // Callbacks

  void syncCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &camera_info_ptr,
                    const sensor_msgs::msg::Image::ConstSharedPtr &depth_image_ptr,
                    const stretch_mover::msg::YoloDetections::ConstSharedPtr &yolo_result_ptr) {

    RCLCPP_ERROR(get_logger(), "In sync call! ");
    stretch_mover::msg::YoloDetections detecs = *yolo_result_ptr;
    detecs.masks = std::vector<sensor_msgs::msg::Image>{};
    RCLCPP_ERROR_STREAM(get_logger(), "yolo result: " << stretch_mover::msg::to_yaml(detecs));
    RCLCPP_ERROR_STREAM(get_logger(),
                        "info stamp: " << std_msgs::msg::to_yaml(camera_info_ptr->header));
    RCLCPP_ERROR_STREAM(get_logger(),
                        "depth stamp: " << std_msgs::msg::to_yaml(depth_image_ptr->header));
  }

public:
  DepthMaskProecss() : rclcpp::Node("depth_mask_process") {
    this->declare_parameter<std::string>("camera_info_topic", "/camera_info");
    this->declare_parameter<std::string>("yolo_result_topic", "/yolo_result/");
    this->declare_parameter<std::string>("aligned_depth_topic",
                                         "/camera/aligned_depth_to_color/image_raw");
    this->declare_parameter<std::string>("world_frame", "/map");

    std::string camera_info_topic;
    this->get_parameter("camera_info_topic", camera_info_topic);
    std::string yolo_result_topic;
    this->get_parameter("yolo_result_topic", yolo_result_topic);
    std::string aligned_depth_topic;
    this->get_parameter("aligned_depth_topic", aligned_depth_topic);
    this->get_parameter("world_frame", world_frame_);

    camera_info_sub_.subscribe(this, camera_info_topic);
    depth_image_sub_.subscribe(this, yolo_result_topic);
    yolo_detection_subs_.subscribe(this, aligned_depth_topic);

    sync_ = std::make_shared<message_filters::Synchronizer<ApproximateSyncPolicyTyped>>(
        10, camera_info_sub_, depth_image_sub_, yolo_detection_subs_);
    sync_->registerCallback(std::bind(&DepthMaskProecss::syncCallback, this, std::placeholders::_1,
                                      std::placeholders::_2, std::placeholders::_3));
    RCLCPP_ERROR(get_logger() ,"message filter should have been configured ");
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DepthMaskProecss>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}