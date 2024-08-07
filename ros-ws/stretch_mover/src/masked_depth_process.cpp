#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <builtin_interfaces/msg/time.hpp>

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

#include <stretch_mover/msg/yolo_detection.hpp>
#include <stretch_mover/msg/yolo_detection_list.hpp>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

using ApproximateSyncPolicyTyped = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::CameraInfo, sensor_msgs::msg::Image, stretch_mover::msg::YoloDetectionList>;

class DepthMaskProecss : public rclcpp::Node {

private:
  // member variables

  // ROS IPC stuff
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_image_sub_;
  message_filters::Subscriber<stretch_mover::msg::YoloDetectionList> yolo_detection_subs_;

  std::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicyTyped>> sync_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_cloud_pub_;
  // Calculation members.

  std::string world_frame_; // The frame we assume detected object will be static with.
  image_geometry::PinholeCameraModel cam_model_;
  double min_depth_range_;
  double max_depth_range_;

  bool debug_;
  bool verbose_;

  // Helpers

  std::optional<geometry_msgs::msg::TransformStamped> GetTF(const std::string &target_frame,
                                                            const std::string &sourec_frame, 
                                                            builtin_interfaces::msg::Time stamp) {
    try {
      auto map_base_tf = tf_buffer_->lookupTransform( target_frame , sourec_frame,stamp );
      return map_base_tf;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_STREAM(get_logger(), "Could not transform map to wx200/base_link: " << ex.what());
      // TODO ignore it if can't find a good transform
      return std::nullopt;
    }
  }

  // Callbacks

  void syncCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &camera_info_ptr,
               const sensor_msgs::msg::Image::ConstSharedPtr &depth_image_ptr,
               const stretch_mover::msg::YoloDetectionList::ConstSharedPtr &yolo_detection_list) {
    if (debug_ && verbose_) {

      RCLCPP_WARN(get_logger(), "In sync call! ");
      // stretch_mover::msg::YoloDetectionList detecs = *yolo_detection_list;
      // detecs.masks = std::vector<sensor_msgs::msg::Image>{};
      RCLCPP_WARN_STREAM(get_logger(),
                          "info stamp: " << std_msgs::msg::to_yaml(camera_info_ptr->header));
      RCLCPP_WARN_STREAM(get_logger(),
                          "depth stamp: " << std_msgs::msg::to_yaml(depth_image_ptr->header));
      RCLCPP_WARN_STREAM(get_logger(),
                          "yolo header: " << std_msgs::msg::to_yaml(yolo_detection_list->header));
    }
    // Loop through each detection

    cam_model_.fromCameraInfo(camera_info_ptr);
    cv_bridge::CvImagePtr depth_cv_ptr;

    try {
      depth_cv_ptr = cv_bridge::toCvCopy(depth_image_ptr, sensor_msgs::image_encodings::TYPE_16UC1);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "%s", e.what());
      return;
    }

    pcl::PointCloud<pcl::PointXYZL> debug_combined_cloud;

    // Assume mask, depth all use the same frame.
    auto maybe_tf = GetTF( world_frame_, depth_image_ptr->header.frame_id, depth_image_ptr->header.stamp );
    if (! maybe_tf.has_value()) {
      RCLCPP_WARN(get_logger(), "TF look up error, skipping this cycle");
      return ;
    }
    geometry_msgs::msg::TransformStamped depth_world_tf = maybe_tf.value();
    // tf_buffer_->lookupTransform(world_frame_, depth_image_ptr->header.frame_id, depth_image_ptr->header.stamp);
    Eigen::Affine3f depth_world_eigen_tf = tf2::transformToEigen(depth_world_tf.transform).cast<float>();

    for (size_t i = 0; i < yolo_detection_list->detections.size(); ++i) {
      stretch_mover::msg::YoloDetection det = yolo_detection_list->detections[i];

      int id = det.class_id;

      cv_bridge::CvImagePtr cv_ptr;

      try {
        cv_ptr = cv_bridge::toCvCopy(det.mask, sensor_msgs::image_encodings::MONO8);
      } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(get_logger(), "%s", e.what());
        return;
      }

      std::vector<cv::Point2i> object_points;
      cv::findNonZero(cv_ptr->image, object_points);

      pcl::PointCloud<pcl::PointXYZL> new_cloud;

      for (const cv::Point2i &loc : object_points) {

        double depth_value = depth_cv_ptr->image.at<uint16_t>(loc);
        depth_value /= 1000.0;
        // Need to double check for out of range depth value
        if (depth_value < min_depth_range_ or depth_value > max_depth_range_) {
          // Skipping this value as it's out of range.
          continue;
        }

        cv::Point3f space_ray = cam_model_.projectPixelTo3dRay(loc);
        cv::Point3f space_loc = space_ray * depth_value;
        new_cloud.push_back(pcl::PointXYZL{space_loc.x, space_loc.y, space_loc.z, uint32_t(id)});
      }
      if (verbose_) {
        RCLCPP_INFO_STREAM(get_logger(), "get new cloud " << new_cloud.width);
      }

      pcl::PointCloud<pcl::PointXYZL> new_cloud_world;
      pcl::transformPointCloud(new_cloud , new_cloud_world , depth_world_eigen_tf);


      if (debug_) {
        debug_combined_cloud += new_cloud_world;
      }



    }
    if (debug_) {
      sensor_msgs::msg::PointCloud2 debug_cloud_msg;
      pcl::toROSMsg(debug_combined_cloud, debug_cloud_msg);
      // debug_cloud_msg.header = yolo_detection_list->header;
      debug_cloud_msg.header = yolo_detection_list->header;
      debug_cloud_msg.header.frame_id = world_frame_;
      debug_cloud_pub_->publish(debug_cloud_msg);
    }
  }

public:
  DepthMaskProecss() : rclcpp::Node("depth_mask_process") {
    this->declare_parameter<std::string>("camera_info_topic", "/camera_info");
    this->declare_parameter<std::string>("aligned_depth_topic",
                                         "/camera/aligned_depth_to_color/image_raw");
    this->declare_parameter<std::string>("yolo_result_topic", "/yolo_result/");

    this->declare_parameter<std::string>("world_frame", "/map");
    this->declare_parameter<double>("min_depth_range", 0.3);
    this->declare_parameter<double>("max_depth_range", 3.0);

    this->declare_parameter<bool>("debug", false);
    this->declare_parameter<bool>("verbose", false);

    std::string camera_info_topic;
    this->get_parameter("camera_info_topic", camera_info_topic);
    std::string aligned_depth_topic;
    this->get_parameter("aligned_depth_topic", aligned_depth_topic);
    std::string yolo_result_topic;
    this->get_parameter("yolo_result_topic", yolo_result_topic);

    this->get_parameter("world_frame", world_frame_);

    this->get_parameter("min_depth_range", min_depth_range_);
    this->get_parameter("max_depth_range", max_depth_range_);

    this->get_parameter("debug", debug_);
    this->get_parameter("verbose", verbose_);

    debug_cloud_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("detected_cloud_debug", 1);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    camera_info_sub_.subscribe(this, camera_info_topic);
    depth_image_sub_.subscribe(this, aligned_depth_topic);
    yolo_detection_subs_.subscribe(this, yolo_result_topic);

    sync_ = std::make_shared<message_filters::Synchronizer<ApproximateSyncPolicyTyped>>(
        10, camera_info_sub_, depth_image_sub_, yolo_detection_subs_);
    sync_->registerCallback(std::bind(&DepthMaskProecss::syncCallback, this, std::placeholders::_1,
                                      std::placeholders::_2, std::placeholders::_3));
    RCLCPP_INFO(get_logger(), "Message filter configured, ready to process");
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DepthMaskProecss>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}