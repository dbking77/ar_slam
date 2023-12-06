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
#include "ar_slam_interfaces/msg/capture.hpp"
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
      desc.name = "expected_detector_types";
      desc.read_only = true;
      desc.description =
        "detectors names to expect before when merging messages, if empty messages is immediately passed through";
      std::vector<std::string> default_detector_types = {"4X4_50", "5X5_100"};
      std::vector<std::string> detector_types = this->declare_parameter(
        desc.name, default_detector_types,
        desc);

      // Convert vector into set
      std::copy(
        detector_types.begin(), detector_types.end(),
        std::inserter(expected_detector_types_, expected_detector_types_.end()));
    }

    {
      rcl_interfaces::msg::ParameterDescriptor desc{};
      desc.name = "include_image";
      desc.read_only = true;
      bool default_include_image = true;
      desc.description =
        "If true, top-level capture image will be included in merged detections";
      include_image_ = this->declare_parameter(
        desc.name, default_include_image, desc);
    }

    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    publisher_ =
      this->create_publisher<ar_slam_interfaces::msg::Detections>("merged_detections", 10);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_;
    rclcpp::QoS qos(10);
    using std::placeholders::_1;
    detection_sub_ = this->create_subscription<ar_slam_interfaces::msg::Detections>(
      "detections", qos, std::bind(&MergeDetections::detectionCallback, this, _1), sub_options);

    if (include_image_) {
      capture_sub_ = this->create_subscription<ar_slam_interfaces::msg::Capture>(
        "captures", qos, std::bind(&MergeDetections::captureCallback, this, _1), sub_options);
    }
  }

protected:
  struct MergedDetections;

  MergedDetections * findOrAdd(const std::string & capture_uid)
  {
    // find matching uid in merge_queue_
    auto itr = std::find_if(
      merge_queue_.begin(), merge_queue_.end(),
      [&](const MergedDetections & merged)
      {
        return merged.getId() == capture_uid;
      });

    if (itr == merge_queue_.end()) {
      merge_queue_.emplace_back(capture_uid);
      return &merge_queue_.back();
    } else if (itr->isPublished()) {
      RCLCPP_WARN_STREAM(
        get_logger(), "Got more dections for "
          << capture_uid << " after result was published");
      return nullptr;
    } else {
      return &(*itr);
    }
  }

  void postCallback(MergedDetections * merged)
  {
    if (merged->hasAll(expected_detector_types_, include_image_)) {
      RCLCPP_INFO_STREAM(get_logger(), "Publishing merged detections for " << merged->getId());
      publisher_->publish(merged->publish());
    }

    // TODO parameterize merge queue size
    while (merge_queue_.size() > 2) {
      auto & front = merge_queue_.front();
      if (!front.isPublished()) {
        std::ostringstream ss;
        for (const auto & detector : front.getDetectorTypes()) {
          ss << ' ' << detector;
        }
        RCLCPP_WARN_STREAM(
          get_logger(), "Dropping incomplete merge for " << front.getId()
                                                         << " with" << ss.str());
      }
      merge_queue_.pop_front();
    }
  }

  void detectionCallback(ar_slam_interfaces::msg::Detections::UniquePtr detections)
  {
    MergedDetections * merged = findOrAdd(detections->capture_uid);
    if (merged != nullptr) {
      merged->add(std::move(detections));
      postCallback(merged);
    }
  }

  void captureCallback(ar_slam_interfaces::msg::Capture::ConstSharedPtr capture)
  {
    MergedDetections * merged = findOrAdd(capture->capture_uid);
    if (merged != nullptr) {
      merged->add(capture);
      postCallback(merged);
    }
  }

  struct MergedDetections
  {
    MergedDetections(std::string capture_uid)
    : capture_uid_{std::move(capture_uid)}
    {
    }

    void add(ar_slam_interfaces::msg::Detections::UniquePtr detections)
    {
      if (static_cast<bool>(merged_detections_)) {
        // steal detections and detector_types types from incoming message
        std::move(
          detections->detections.begin(),
          detections->detections.end(),
          std::back_inserter(merged_detections_->detections));
        std::move(
          detections->detector_types.begin(),
          detections->detector_types.end(),
          std::inserter(detector_types_, detector_types_.end()));
      } else {
        // steal entire message
        merged_detections_ = std::move(detections);
        // steal detections from internal message
        std::move(
          merged_detections_->detector_types.begin(),
          merged_detections_->detector_types.end(),
          std::inserter(detector_types_, detector_types_.end()));
      }
    }

    void add(ar_slam_interfaces::msg::Capture::ConstSharedPtr capture)
    {
      capture_ = std::move(capture);
    }

    bool isPublished() const
    {
      return is_published_;
    }

    ar_slam_interfaces::msg::Detections::UniquePtr publish()
    {
      std::move(
        detector_types_.begin(), detector_types_.end(),
        std::back_inserter(merged_detections_->detector_types));
      if (static_cast<bool>(capture_)) {
        merged_detections_->image = capture_->image;
      }
      is_published_ = true;
      return std::move(merged_detections_);
    }

    bool hasAll(const std::set<std::string> & expected_detector_types, bool include_image)
    {
      if (include_image and !static_cast<bool>(capture_)) {
        return false;
      }
      return std::includes(
        detector_types_.begin(), detector_types_.end(),
        expected_detector_types.begin(), expected_detector_types.end());
    }

    const std::string & getId() const
    {
      return capture_uid_;
    }

    const std::set<std::string> & getDetectorTypes() const
    {
      return detector_types_;
    }

protected:
    const std::string capture_uid_;
    std::set<std::string> detector_types_;
    bool is_published_ = false;
    ar_slam_interfaces::msg::Detections::UniquePtr merged_detections_;
    ar_slam_interfaces::msg::Capture::ConstSharedPtr capture_;
  };

  std::deque<MergedDetections> merge_queue_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<ar_slam_interfaces::msg::Detections>::SharedPtr detection_sub_;
  rclcpp::Subscription<ar_slam_interfaces::msg::Capture>::SharedPtr capture_sub_;
  rclcpp::Publisher<ar_slam_interfaces::msg::Detections>::SharedPtr publisher_;

  std::set<std::string> expected_detector_types_;
  bool include_image_ = true;
};

}  // namespace ar_slam

RCLCPP_COMPONENTS_REGISTER_NODE(ar_slam::MergeDetections)
