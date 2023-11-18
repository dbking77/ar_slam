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

// Components
#include <rclcpp_components/register_node_macro.hpp>

namespace ar_slam
{

class MergeDetections : public rclcpp::Node
{
public:
  MergeDetections(const rclcpp::NodeOptions & options)
  : Node("merge_detections", options)
  {
    {
      rcl_interfaces::msg::ParameterDescriptor desc{};
      desc.name = "expected_detectors";
      desc.read_only = true;
      desc.description =
        "detectors names to expect before when merging messages, if empty messages is immediately passed through";
      std::vector<std::string> default_detectors = {"4X4_50", "5X5_100"};
      std::vector<std::string> detectors = this->declare_parameter(
        desc.name, default_detectors,
        desc);

      // Convert vector into set
      std::copy(
        detectors.begin(), detectors.end(),
        std::inserter(expected_detectors_, expected_detectors_.end()));
    }

    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_;
    rclcpp::QoS qos(10);
    using std::placeholders::_1;
    subscription_ = this->create_subscription<ar_slam_interfaces::msg::Detections>(
      "detections", qos, std::bind(&MergeDetections::detection_callback, this, _1), sub_options);

    publisher_ =
      this->create_publisher<ar_slam_interfaces::msg::Detections>("merged_detections", 10);
  }

protected:
  void detection_callback(ar_slam_interfaces::msg::Detections::UniquePtr detections)
  {
    // find matching  in merge_queue_
    auto itr = std::find_if(
      merge_queue_.begin(), merge_queue_.end(),
      [&](const MergedDetections & merged)
      {
        return merged.get_id() == detections->capture_uid;
      });

    if (itr == merge_queue_.end()) {
      merge_queue_.emplace_back(std::move(detections));
      itr = merge_queue_.end() - 1;
    } else {
      if (!itr->add(std::move(detections))) {
        // This might happend if there are more detectors running that list in expected set
        RCLCPP_WARN_STREAM(
          get_logger(), "Got more dections for "
            << detections->capture_uid << " after result was published");
      }
    }

    if (itr->has_all(expected_detectors_)) {
      RCLCPP_INFO_STREAM(get_logger(), "Publishing merged detections for " << itr->get_id());
      publisher_->publish(itr->publish());
    }

    while (merge_queue_.size() > 1) {
      auto & front = merge_queue_.front();
      if (!front.is_published()) {
        std::ostringstream ss;
        for (const auto & detector : front.get_detectors()) {
          ss << ' ' << detector;
        }
        RCLCPP_WARN_STREAM(
          get_logger(), "Dropping incomplete merge for " << front.get_id()
                                                         << " with" << ss.str());
      }
      merge_queue_.pop_front();
    }
  }

  struct MergedDetections
  {
    MergedDetections(ar_slam_interfaces::msg::Detections::UniquePtr detections)
    : capture_uid_{detections->capture_uid},
      merged_detections_{std::move(detections)}
    {
      std::move(
        merged_detections_->detectors.begin(),
        merged_detections_->detectors.end(),
        std::inserter(detectors_, detectors_.end()));
    }

    bool add(ar_slam_interfaces::msg::Detections::UniquePtr detections)
    {
      if (is_published()) {
        return false;
      }
      std::move(
        detections->detectors.begin(),
        detections->detectors.end(),
        std::inserter(detectors_, detectors_.end()));
      std::move(
        detections->detections.begin(),
        detections->detections.end(),
        std::back_inserter(merged_detections_->detections));
      return true;
    }

    bool is_published() const
    {
      return !static_cast<bool>(merged_detections_);
    }

    ar_slam_interfaces::msg::Detections::UniquePtr publish()
    {
      std::move(
        detectors_.begin(), detectors_.end(),
        std::back_inserter(merged_detections_->detectors));
      return std::move(merged_detections_);
    }

    bool has_all(const std::set<std::string> & expected_detectors)
    {
      return std::includes(
        detectors_.begin(), detectors_.end(),
        expected_detectors.begin(), expected_detectors.end());
    }

    const std::string & get_id() const
    {
      return capture_uid_;
    }

    const std::set<std::string> & get_detectors() const
    {
      return detectors_;
    }

protected:
    const std::string capture_uid_;
    std::set<std::string> detectors_;
    ar_slam_interfaces::msg::Detections::UniquePtr merged_detections_;
  };

  std::deque<MergedDetections> merge_queue_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<ar_slam_interfaces::msg::Detections>::SharedPtr subscription_;
  rclcpp::Publisher<ar_slam_interfaces::msg::Detections>::SharedPtr publisher_;

  std::set<std::string> expected_detectors_;
};

}  // namespace ar_slam

RCLCPP_COMPONENTS_REGISTER_NODE(ar_slam::MergeDetections)
