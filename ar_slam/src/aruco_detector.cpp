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
#include <map>
#include <memory>
#include <string>

// ROS2
#include "rclcpp/rclcpp.hpp"

// ROS2 Interfaces
#include "ar_slam_interfaces/msg/detections.hpp"
#include "ar_slam_interfaces/msg/capture.hpp"
//#include "sensor_msgs/msg/image.hpp"

// ROS2 Other
#include "cv_bridge/cv_bridge.hpp"
#include "sensor_msgs/image_encodings.hpp"

// OpenCV
#include "opencv2/aruco.hpp"
#include "opencv2/opencv.hpp"

// Components
#include <rclcpp_components/register_node_macro.hpp>

// ArSlam
#include "ar_slam/ar_slam_util.hpp"

namespace ar_slam
{

class ArucoDetector : public rclcpp::Node
{
public:
  ArucoDetector(const rclcpp::NodeOptions & options)
  : Node("aruco_detect", options),
    aruco_params_{new cv::aruco::DetectorParameters}
  {
    {
      rcl_interfaces::msg::ParameterDescriptor desc{};
      desc.name = "aruco_dict";
      desc.read_only = true;
      desc.description = "Aruco Dictionary to use";
      std::ostringstream options;
      for (const auto & kv : aruco_dict_lookup_) {
        options << kv.first << " ";
      }
      desc.additional_constraints = options.str();
      std::string dict_name = this->declare_parameter(desc.name, "4X4_50", desc);
      detector_name_ = std::string("aruco_") + dict_name;
      auto itr = aruco_dict_lookup_.find(dict_name);
      if (itr == aruco_dict_lookup_.end()) {
        RCLCPP_INFO(get_logger(), "Invalid aruco_dict %s", dict_name.c_str());
        throw std::runtime_error("invalid aruco_dict");
      }
      RCLCPP_ERROR(get_logger(), "Using aruco_dict %s", dict_name.c_str());
      aruco_dict_ = cv::aruco::getPredefinedDictionary(itr->second);
    }

    {
      rcl_interfaces::msg::ParameterDescriptor desc{};
      desc.name = "id_offset";
      desc.read_only = true;
      desc.description =
        "offset to add to detected Arudo ID.  Allows different types of tags to be used with overlapping ID ranges";
      desc.integer_range.emplace_back();
      desc.integer_range.at(0).from_value = 0;
      desc.integer_range.at(0).to_value = 1000 * 1000 * 1000;
      id_offset_ = this->declare_parameter(desc.name, 0, desc);
    }

    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_;
    rclcpp::QoS qos(10);
    using std::placeholders::_1;
    subscription_ = this->create_subscription<ar_slam_interfaces::msg::Capture>(
      "images", qos, std::bind(&ArucoDetector::image_callback, this, _1), sub_options);

    publisher_ = this->create_publisher<ar_slam_interfaces::msg::Detections>("detections", 10);
  }

protected:
  void image_callback(std::shared_ptr<const ar_slam_interfaces::msg::Capture> capture) const
  {
    const sensor_msgs::msg::Image image = capture->image;
    RCLCPP_INFO(
      this->get_logger(), "I got image %d x %d id %s ptr %p",
      image.width, image.height,
      capture->capture_uid.c_str(),
      (void *)capture.get());

    cv_bridge::CvImagePtr cv_img = cv_bridge::toCvCopy(image);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> rects, rejected;
    cv::aruco::detectMarkers(cv_img->image, aruco_dict_, rects, ids, aruco_params_, rejected);

    if (ids.empty()) {
      RCLCPP_WARN(get_logger(), "No AR tags detected");
    } else {
      RCLCPP_INFO(get_logger(), "Detected %d tags in image", static_cast<int>(ids.size()));
    }

    cv::Size img_size = cv_img->image.size();
    auto msg = std::make_unique<ar_slam_interfaces::msg::Detections>();
    msg->capture_uid = capture->capture_uid;
    msg->image_width = capture->image.width;
    msg->image_height = capture->image.height;
    msg->image_path = capture->image_path;
    msg->detectors.emplace_back(detector_name_);
    msg->detections.resize(ids.size());
    for (unsigned idx = 0; idx < ids.size(); ++idx) {
      auto & detection = msg->detections.at(idx);
      detection.id = ids[idx] + id_offset_;
      const auto & rect = rects.at(idx);
      for (unsigned ii = 0; ii < 4; ++ii) {
        Point point = from_cv_img(rect.at(ii), img_size);
        auto & corner = detection.corners[ii];
        corner.x = point.x;
        corner.y = point.y;
      }
    }

    publisher_->publish(std::move(msg));
  }

  std::string detector_name_;
  cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
  cv::Ptr<cv::aruco::DetectorParameters> aruco_params_;
  unsigned id_offset_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<ar_slam_interfaces::msg::Capture>::SharedPtr subscription_;
  rclcpp::Publisher<ar_slam_interfaces::msg::Detections>::SharedPtr publisher_;

  std::map<std::string, int> aruco_dict_lookup_ = {
    {"4X4_50", cv::aruco::DICT_4X4_50},
    {"5X5_100", cv::aruco::DICT_5X5_100},
    {"6X6_250", cv::aruco::DICT_6X6_250},
  };
};

}  // namespace ar_slam

RCLCPP_COMPONENTS_REGISTER_NODE(ar_slam::ArucoDetector)
