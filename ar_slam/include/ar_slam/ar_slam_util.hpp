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

#ifndef AR_SLAM_AR_SLAM_UTIL
#define AR_SLAM_AR_SLAM_UTIL

// C++ stdlib
#include <array>
#include <cmath>
#include <iostream>
#include <fstream>
#include <optional>
#include <string>
#include <vector>

// ROS2
#include "rclcpp/rclcpp.hpp"

//OpenCV
#include "opencv2/opencv.hpp"

// Ceres
#include "ceres/ceres.h"
#include "ceres/rotation.h"

// ROS2 Interfaces
#include "ar_slam_interfaces/msg/detections.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

// Yaml
#include "yaml-cpp/yaml.h"

struct Point
{
  Point() = default;
  Point(double _x, double _y)
  : x{_x}, y{_y} {}
  double x = 0.0;
  double y = 0.0;
};


struct CameraParams
{
  CameraParams()
  {
    std::fill(params.begin(), params.end(), 0.0);
    params[0] = 3000.0; // need to start with non-zero focal length
  }
  // 3 focal,l1,l2
  std::array<double, 3> params;

  // expected size of image
  std::optional<cv::Size> size;
};


struct PoseParams
{
  PoseParams()
  {
    std::fill(params.begin(), params.end(), 0.0);
  }

  /**
   * first 3 are translation
   * second 3 are axis-angle rotation
   * for axis angle, magnitude of vector is magnitude of rotation
   */
  std::array<double, 6> params;
};

struct CaptureHandle
{
  //CaptureHandle() = default;
  explicit CaptureHandle(unsigned _idx)
  : idx{_idx} {}
  bool operator==(const CaptureHandle & other) const {return other.idx == this->idx;}
  const unsigned idx = ~0;
};

struct ArucoHandle
{
  //ArucoHandle() = default;
  explicit ArucoHandle(unsigned _idx)
  : idx{_idx} {}
  const unsigned idx = ~0;
};

struct BlockHandle
{
  //BlockHandle() = default;
  explicit BlockHandle(unsigned _idx)
  : idx{_idx} {}
  unsigned idx = ~0;
};

struct CaptureUid
{
  explicit CaptureUid(unsigned _uid)
  : uid{_uid} {}
  bool operator==(const CaptureUid & other) const {return other.uid == this->uid;}
  const unsigned uid = 0;
};

struct ArucoId
{
  explicit ArucoId(unsigned _id)
  : id{_id} {}
  bool operator==(const ArucoId & other) const {return other.id == this->id;}
  const unsigned id = 0;
};


namespace std
{

template<>
struct hash<CaptureHandle>
{
  size_t operator()(const CaptureHandle & cap_handle) const
  {
    return cap_handle.idx;
  }
};

template<>
struct hash<CaptureUid>
{
  size_t operator()(const CaptureUid & cap_uid) const
  {
    return cap_uid.uid;
  }
};

inline std::string to_string(const CaptureUid & cap_uid)
{
  return std::string("CaptureUid:") + std::to_string(cap_uid.uid);
}

template<>
struct hash<ArucoId>
{
  size_t operator()(const ArucoId & ar_id) const
  {
    return ar_id.id;
  }
};

inline std::string to_string(const ArucoId & ar_id)
{
  return std::string("ArucoId:") + std::to_string(ar_id.id);
}

} // namespace std


inline std::ostream & operator<<(std::ostream & os, const ArucoId & ar_id)
{
  os << "ArucoId:" << ar_id.id;
  return os;
}

inline std::ostream & operator<<(std::ostream & os, const CaptureUid & cap_uid)
{
  os << "CaptureUid:" << cap_uid.uid;
  return os;
}

/// Represents a single capture from a camera
struct Capture
{
  Capture(CaptureUid _uid, CaptureHandle _handle, std::string _fn)
  : uid{_uid}, handle{_handle}, img_fn{std::move(_fn)}
  {}
  const CaptureUid uid;
  const CaptureHandle handle;
  const std::string img_fn;
  cv::Mat img;
  std::vector<BlockHandle> blocks;
  std::optional<BlockHandle> init_block;

  /**
   * inverse transform of pose
   * invert is used by optimize more often when optimizing
   */
  PoseParams inv_pose;

  double * data() {return inv_pose.params.data();}
  const double * data() const {return inv_pose.params.data();}

  cv::Mat loadImg()
  {
    if (img.empty()) {
      img = cv::imread(img_fn);
      if (img.empty()) {
        std::ostringstream ss;
        ss << "error loading image " << img_fn;
        throw std::runtime_error(ss.str());
      }
    }
    return img;
  }
};


/// Represents a unique AR tag
struct Aruco
{
  Aruco(ArucoId _id, ArucoHandle _handle)
  : id{_id}, handle{_handle} {}
  const ArucoId id;
  const ArucoHandle handle;
  bool initialized = false;
  std::vector<BlockHandle> blocks;  // is this needed?
  PoseParams pose;
  double * data() {return pose.params.data();}
  const double * data() const {return pose.params.data();}
};


/**
 * Convert point in capture x,y space to OpenCV coordinate system.
 * For this problem, 0,0 is at center of image, and x,y is normal cordinate system (+y is up)
 * OpenCV uses for images where y-axis increased downward and top-left is corner is 0,0
 * Also optionally scale down coordinates to scaled image
 */
inline cv::Point to_cv_img(double x, double y, const cv::Size & scaled_img_size, float scale = 1.0)
{
  const float xc = 0.5 * scaled_img_size.width;
  const float yc = 0.5 * scaled_img_size.height;
  return cv::Point((scale * x + xc), (-scale * y + yc));
}


inline Point from_cv_img(const cv::Point2f & point, const cv::Size & img_size)
{
  return Point {
    +(point.x - 0.5 * img_size.width),
    -(point.y - 0.5 * img_size.height)
  };
}


struct ArucoRect
{
  ArucoRect() = default;

  /**
   * Create rectangle from OpenCV ar_tag dectection.
   * Note conversion from OpenCV coordinate system to one used here
   */
  ArucoRect(const std::vector<cv::Point2f> & ar_detect, cv::Size img_size)
  {
    if (ar_detect.size() != corners.size()) {
      throw std::runtime_error("incorrect number of points");
    }
    for (unsigned idx = 0; idx < corners.size(); ++idx) {
      corners[idx] = from_cv_img(ar_detect[idx], img_size);
    }
  }

  ArucoRect(const ar_slam_interfaces::msg::Detection & detection)
  {
    for (unsigned idx = 0; idx < corners.size(); ++idx) {
      corners[idx].x = detection.corners[idx].x;
      corners[idx].y = detection.corners[idx].y;
    }
  }

  std::array<Point, 4> corners;
};


struct Block
{
  Block(
    BlockHandle _handle,
    const ArucoRect & _aruco_rect,
    CaptureHandle _capture,
    ArucoHandle _aruco)
  : handle{_handle},
    aruco_rect{_aruco_rect},
    capture{_capture},
    aruco{_aruco}
  {
  }

  BlockHandle handle;
  ArucoRect aruco_rect;
  CaptureHandle capture;
  ArucoHandle aruco;
  bool added = false;
};


/// 2.5 inches (parameterize value)
static constexpr double aruco_size = 0.0635;

/**
 * Point ordering from OpenCV Aruco tag detection
 * top left, top right, bottom right, bottom left
 * -x,+y,  +x,+y, +x,-y, -x,-y
 */
static const std::array<Point, 4> ARUCO_DIRECTIONS = {
  Point{-1, +1},
  Point{+1, +1},
  Point{+1, -1},
  Point{-1, -1}
};

inline double normalize_angle(double angle)
{
  return std::fmod(std::fmod(angle, 2 * M_PI) + 3 * M_PI, 2 * M_PI) - M_PI;
}


bool endswith(const std::string & str, const std::string & suffix);

/**
 * Take file path and returns just files basename with no extension
 *   file.jpg          -> file
 *   /path/to/file.jpg -> file
 *   ../../file.jpg    -> file
 *   ../file           -> file
 *   ../file.1.jpg     -> file.1
 */
std::string filename_no_ext(std::string filepath);


class ArSlamSolver
{
public:
  ArSlamSolver() = default;

  void loadImages(const std::vector<std::string> & img_fns);

  void loadYaml(const std::string & fn);

  void saveYaml(std::ostream & output) const;

  void displayDebug(const Capture & capture);

  void printCameras() const;

  void compareProjections() const;

  void solve();

  void solveIncremental();

  unsigned getNextCaptureIndex() const;

  void localizeMany(unsigned first_loc_cap_idx);

  void addDetections(const ar_slam_interfaces::msg::Detections & detections);

  std::vector<geometry_msgs::msg::TransformStamped>
  getTransforms(const rclcpp::Time & stamp) const;

  bool & display_debug() {return display_debug_;}
  double & display_wait_duration() {return display_wait_duration_;}

protected:
  Capture & addCapture(CaptureUid cap_uid, std::string fn)
  {
    CaptureHandle cap_handle(captures_.size());
    auto [itr, added] = capture_map_.try_emplace(cap_uid, cap_handle);
    if (!added) {
      throw std::runtime_error("Capture with uid already added");
    }
    captures_.emplace_back(std::move(cap_uid), cap_handle, std::move(fn));
    return captures_.back();
  }

  Aruco & addAruco(ArucoId ar_id)
  {
    ArucoHandle ar_handle(arucos_.size());
    aruco_map_.try_emplace(ar_id, ar_handle);
    arucos_.emplace_back(std::move(ar_id), ar_handle);
    return arucos_.back();
  }

  Aruco & getOrAddAruco(const ArucoId & ar_id)
  {
    auto aruco_itr = aruco_map_.find(ar_id);
    if (aruco_itr == aruco_map_.end()) {
      return addAruco(ar_id);
    }
    return at(aruco_itr->second);
  }

  Block & addBlock(
    const ArucoRect aruco_rect,
    CaptureHandle capture_handle,
    ArucoHandle aruco_handle)
  {
    BlockHandle block_handle(blocks_.size());
    blocks_.emplace_back(block_handle, std::move(aruco_rect), capture_handle, aruco_handle);
    at(capture_handle).blocks.emplace_back(block_handle);
    at(aruco_handle).blocks.emplace_back(block_handle);
    return blocks_.back();
  }

  Capture & at(CaptureHandle handle) {return captures_[handle.idx];}
  const Capture & at(CaptureHandle handle) const {return captures_[handle.idx];}

  Aruco & at(ArucoHandle handle) {return arucos_[handle.idx];}
  const Aruco & at(ArucoHandle handle) const {return arucos_[handle.idx];}

  Block & at(BlockHandle handle) {return blocks_[handle.idx];}
  const Block & at(BlockHandle handle) const {return blocks_[handle.idx];}

  void localizeOne(Capture & capture, unsigned first_loc_cap_idx);

  void addConnectedCaptures(
    const Capture & base_capture,
    std::deque<CaptureHandle> & open_captures);

  void solveCapture(Capture & capture, std::optional<BlockHandle> init_block_handle);

  void optimize(const Capture & capture);

  void resetProblem();

  cv::Mat checkAndFixImageSize(cv::Mat img);

  ceres::Problem problem_;

  CameraParams camera_;

  // Data store
  std::deque<Capture> captures_;
  std::deque<Aruco> arucos_;
  std::vector<Block> blocks_;

  // Capture index and ar_tags ID might not start from 0
  // have a lookup from ar_id and cap_idx into array index in data store
  std::unordered_map<CaptureUid, CaptureHandle> capture_map_;
  std::unordered_map<ArucoId, ArucoHandle> aruco_map_;

  /**
   * When solving incremental it is not always possible to solve a new
   * capture if it is not connected to the rest of captures through a common
   * ar tag.
   */
  std::unordered_set<CaptureHandle> unsolved_captures_;

  bool display_debug_ = true;
  bool display_debug_show_all_ar_ = false;
  double display_wait_duration_ = 0.0;
};

inline YAML::Emitter & operator<<(YAML::Emitter & out, const CaptureUid & uid)
{
  out << uid.uid;
  return out;
}

inline YAML::Emitter & operator<<(YAML::Emitter & out, const ArucoId & id)
{
  out << id.id;
  return out;
}


#endif  // AR_SLAM_AR_SLAM_UTIL
