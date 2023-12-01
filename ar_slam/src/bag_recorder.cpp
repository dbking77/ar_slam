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

// ROS2 Other
#include "rosbag2_cpp/writer.hpp"

// OpenCV
#include "opencv2/aruco.hpp"
#include "opencv2/opencv.hpp"

// Components
#include <rclcpp_components/register_node_macro.hpp>

// ArSlam
#include "ar_slam/ar_slam_util.hpp"

namespace ar_slam
{

class BagRecorder : public rclcpp::Node
{
public:
  BagRecorder(const rclcpp::NodeOptions & options)
  : Node("bag_recorder", options)
  {
    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open("/tmp/ar_slam.bag");

    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_;
    rclcpp::QoS qos(10);
    using std::placeholders::_1;
    capture_sub_ = this->create_subscription<ar_slam_interfaces::msg::Capture>(
      "captures", qos, std::bind(&BagRecorder::capture_callback, this, _1), sub_options);
    detect_sub_ = this->create_subscription<ar_slam_interfaces::msg::Detections>(
      "merged_detections", qos, std::bind(&BagRecorder::detection_callback, this, _1), sub_options);
  }

protected:
  void capture_callback(std::shared_ptr<rclcpp::SerializedMessage> msg)
  {
    rclcpp::Time time_stamp = this->now();
    writer_->write(msg, "captures", "ar_slam_interfaces/msg/Capture", time_stamp);
  }

  void detection_callback(std::shared_ptr<rclcpp::SerializedMessage> msg)
  {
    rclcpp::Time time_stamp = this->now();
    writer_->write(msg, "merged_detections", "ar_slam_interfaces/msg/Detections", time_stamp);
  }

  std::unique_ptr<rosbag2_cpp::Writer> writer_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<ar_slam_interfaces::msg::Capture>::SharedPtr capture_sub_;
  rclcpp::Subscription<ar_slam_interfaces::msg::Detections>::SharedPtr detect_sub_;
};

}  // namespace ar_slam

RCLCPP_COMPONENTS_REGISTER_NODE(ar_slam::BagRecorder)
