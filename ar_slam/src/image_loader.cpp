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
#include <chrono>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <thread>

// OpenCV
#include "opencv2/opencv.hpp"

// ROS2
#include "rclcpp/rclcpp.hpp"

// ROS2 Interfaces
//#include "sensor_msgs/msg/image.hpp"
#include "ar_slam_interfaces/msg/capture.hpp"
#include "ar_slam_interfaces/srv/load_images.hpp"

// ROS2 Other
#include "cv_bridge/cv_bridge.hpp"
#include "sensor_msgs/image_encodings.hpp"

// Components
#include <rclcpp_components/register_node_macro.hpp>

#include "ar_slam/ar_slam_util.hpp"

using namespace std::chrono_literals;

namespace ar_slam
{

class ImageLoader : public rclcpp::Node
{
public:
  ImageLoader(const rclcpp::NodeOptions & options)
  : Node("img_loader", options)
  {
    using std::placeholders::_1;
    using std::placeholders::_2;

    {
      rcl_interfaces::msg::ParameterDescriptor desc{};
      desc.name = "img_fn";
      desc.read_only = true;
      desc.description = "image filename to load at startup (if non-empty)";
      std::string img_fn = this->declare_parameter(desc.name, "", desc);
      if (!img_fn.empty()) {
        RCLCPP_INFO(this->get_logger(), "Using initial img_fn %s", img_fn.c_str());
        img_fns_.emplace_back(img_fn);
      }
    }

    {
      rcl_interfaces::msg::ParameterDescriptor desc{};
      desc.name = "img_fns";
      desc.read_only = true;
      desc.description = "list of image filenames to load at startup (if non-empty)";
      std::vector<std::string> default_img_fns = {}; // don't load any files by default
      std::vector<std::string> img_fns = this->declare_parameter(desc.name, default_img_fns, desc);
      for (const std::string & img_fn : img_fns) {
        if (!img_fn.empty()) {
          RCLCPP_INFO(this->get_logger(), "Loading initial fn %s", img_fn.c_str());
          img_fns_.emplace_back(img_fn);
        }
      }
    }

    double pub_period = 0.0;
    {
      rcl_interfaces::msg::ParameterDescriptor desc{};
      desc.name = "pub_period";
      desc.read_only = true;
      desc.description = "image publishing period";
      desc.floating_point_range.emplace_back();
      desc.floating_point_range.at(0).from_value = 0.0;
      desc.floating_point_range.at(0).to_value = 10.0;
      pub_period = this->declare_parameter(desc.name, 0.5, desc);
    }

    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::PublisherOptions pub_options;
    // pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
    publisher_ =
      this->create_publisher<ar_slam_interfaces::msg::Capture>("captures", 10, pub_options);

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(pub_period),
      std::bind(&ImageLoader::timerCallback, this),
      callback_group_);

    service_ = this->create_service<ar_slam_interfaces::srv::LoadImages>(
      "load_images",
      std::bind(&ImageLoader::loadImagesServiceCb, this, _1, _2),
      rclcpp::ServicesQoS{},
      callback_group_
    );
  }

private:
  void loadImagesServiceCb(
    const std::shared_ptr<ar_slam_interfaces::srv::LoadImages::Request> request,
    std::shared_ptr<ar_slam_interfaces::srv::LoadImages::Response> response)
  {
    std::lock_guard lock(mutex_);
    std::copy(request->img_fns.begin(), request->img_fns.end(), std::back_inserter(img_fns_));
    response->msg = "good";
    response->success = true;
  }

  cv::Mat checkAndFixImageSize(cv::Mat img)
  {
    if (img_size_.has_value()) {
      cv::Size img_size = img_size_.value();

      // Auto-rotate image if needed.  Phone cameras will anoyingly rotate image
      if ((img.size().width == img_size.height) and
        (img.size().height == img_size.width))
      {
        RCLCPP_WARN(
          this->get_logger(),
          "Image rotated relative to previous, fixing by rotating 90 degrees");
        cv::rotate(img, img, cv::ROTATE_90_CLOCKWISE);
      }

      if (img.size() != img_size) {
        std::ostringstream ss;
        ss << "Loaded images should all be same size : "
           << " expected " << img_size
           << " got " << img.size();
        RCLCPP_ERROR(this->get_logger(), ss.str().c_str());
        img = cv::Mat();
      }
    } else {
      img_size_ = img.size();
    }
    return img;
  }

  void timerCallback()
  {
    std::lock_guard lock(mutex_);
    if (img_fns_.empty()) {
      return;
    }
    std::string img_fn = img_fns_.front();
    img_fns_.pop_front();
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", img_fn.c_str());

    cv::Mat img = cv::imread(img_fn);
    if (img.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Error loading image: '%s'", img_fn.c_str());
      return;
    }

    img = checkAndFixImageSize(img);
    if (img.empty()) {
      return;
    }

    if (img.type() != CV_8UC3) {
      RCLCPP_ERROR(this->get_logger(), "Unexpected image type: '%d'", img.type());
      return;
    }

    cv_bridge::CvImage cv_img(std_msgs::msg::Header{}, sensor_msgs::image_encodings::BGR8, img);
    auto msg = std::make_unique<ar_slam_interfaces::msg::Capture>();
    msg->image_path = img_fn;
    cv_img.toImageMsg(msg->image);

    std::string unique_name = gen_unique_name(img_fn);
    unique_names_.insert(unique_name);
    msg->capture_uid = std::move(unique_name);

    RCLCPP_INFO(
      this->get_logger(), "Image size %d x %d id %s ptr %p",
      msg->image.width, msg->image.height,
      msg->capture_uid.c_str(),
      (void *)msg.get());

    publisher_->publish(std::move(msg));
  }

  std::string gen_unique_name(const std::string & img_fn)
  {
    std::string base_name = filename_no_ext(img_fn);
    if (unique_names_.count(base_name) == 0) {
      return base_name;
    }

    // base name is already used
    for (int ii = 0; ii < 1000; ++ii) {
      std::string new_name = base_name + ":" + std::to_string(ii);
      if (unique_names_.count(new_name) == 0) {
        return new_name;
      }
    }

    throw std::runtime_error("Could not find unique name for " + base_name);
  }

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<ar_slam_interfaces::msg::Capture>::SharedPtr publisher_;
  rclcpp::Service<ar_slam_interfaces::srv::LoadImages>::SharedPtr service_;
  std::deque<std::string> img_fns_;
  std::optional<cv::Size> img_size_;
  std::mutex mutex_;

  // Track names of images so each output message can be given a unique id based on image name
  std::set<std::string> unique_names_;
};

}  // namespace ar_slam

RCLCPP_COMPONENTS_REGISTER_NODE(ar_slam::ImageLoader)
