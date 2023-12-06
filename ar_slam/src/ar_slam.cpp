/*
Copyright (c) 2023 Derek King

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/


// Standard Libaray
#include <fstream>
#include <map>
#include <memory>
#include <string>

// ArSlam
#include "ar_slam/ar_slam_util.hpp"

// ROS2
#include "rclcpp/rclcpp.hpp"

// ROS2 Interfaces
#include "ar_slam_interfaces/msg/detections.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// Components
#include <rclcpp_components/register_node_macro.hpp>

// TF2
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

namespace ar_slam
{

class ArSlam : public rclcpp::Node
{
public:
  ArSlam(const rclcpp::NodeOptions & options)
  : Node("ar_slam", options)
  {
    {
      rcl_interfaces::msg::ParameterDescriptor desc{};
      desc.name = "output_map_fn";
      desc.read_only = true;
      desc.description =
        "Output filename for saved YAML map.  If empty map will not be saved";
      output_map_fn_ = this->declare_parameter(desc.name, "", desc);
    }

    {
      rcl_interfaces::msg::ParameterDescriptor desc{};
      desc.name = "display_debug";
      desc.read_only = true;
      desc.description =
        "If true debugging images will be display whie processing each capture";
      solver_.display_debug() = this->declare_parameter(desc.name, false, desc);
    }

    {
      rcl_interfaces::msg::ParameterDescriptor desc{};
      desc.name = "display_wait_duration";
      desc.read_only = true;
      desc.description =
        "How long to wait for key press while display debuging image.  Use 0 for forever.";
      solver_.display_wait_duration() = this->declare_parameter(desc.name, 0.1, desc);
    }

    // TODO size params for different detection types?
    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/ar_slam/image", 10);
    camera_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/ar_slam/camera_info", 10);
    viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/ar_slam/viz", 10);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_;
    rclcpp::QoS qos(10);
    using std::placeholders::_1;
    subscription_ = this->create_subscription<ar_slam_interfaces::msg::Detections>(
      "merged_detections", qos, std::bind(&ArSlam::detection_callback, this, _1), sub_options);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  ~ArSlam()
  {
    if (!output_map_fn_.empty()) {
      const std::string & fn = output_map_fn_;
      std::cout << "Saving map to " << fn << std::endl;
      std::ofstream file(fn);
      solver_.saveYaml(file);
    }
  }

protected:
  void detection_callback(ar_slam_interfaces::msg::Detections::UniquePtr detections)
  {
    RCLCPP_INFO_STREAM(
      this->get_logger(), "Got " << detections->detections.size()
                                 << " detections for " << detections->capture_uid
                                 << " from " << detections->image_path);

    // Capture handle should always be valid if image has detections
    std::optional<CaptureHandle> capture_handle = solver_.addDetections(*detections);
    if (!capture_handle.has_value()) {
      RCLCPP_WARN_STREAM(
        this->get_logger(),
        "No failed adding capture " << detections->capture_uid);
      return;
    }

    solver_.solveIncremental();

    rclcpp::Time stamp = this->get_clock()->now();
    tf_broadcaster_->sendTransform(solver_.getTransforms(stamp));

    if (detections->image.data.size()) {
      auto image_msg = std::make_unique<sensor_msgs::msg::Image>(detections->image);
      image_msg->header.frame_id = std::to_string(solver_.at(capture_handle.value()).uid);
      image_msg->header.stamp = stamp;
      image_pub_->publish(std::move(image_msg));
    } else {
      RCLCPP_WARN(get_logger(), "Empty image in detections message");
    }

    {
      auto camera_msg = std::make_unique<sensor_msgs::msg::CameraInfo>(solver_.getCameraInfo());
      camera_msg->header.frame_id = std::to_string(solver_.at(capture_handle.value()).uid);
      camera_msg->header.stamp = stamp;
      camera_msg->height = detections->image_height;
      camera_msg->width = detections->image_width;
      camera_pub_->publish(std::move(camera_msg));
    }

    auto viz_msg = std::make_unique<visualization_msgs::msg::MarkerArray>();
    solver_.appendArucoMarkers(viz_msg->markers, stamp);
    viz_pub_->publish(std::move(viz_msg));
  }

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<ar_slam_interfaces::msg::Detections>::SharedPtr subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;

  ArSlamSolver solver_;

  std::string output_map_fn_;
};

}  // namespace ar_slam

RCLCPP_COMPONENTS_REGISTER_NODE(ar_slam::ArSlam)
