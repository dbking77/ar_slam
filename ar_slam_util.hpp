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

//OpenCV
#include "opencv2/opencv.hpp"

// Ceres
#include "ceres/ceres.h"
#include "ceres/rotation.h"


struct Point
{
  Point() = default;
  Point(double _x, double _y) : x{_x}, y{_y} {}
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


/// Represents a single capture from a camera
struct Capture
{
  Capture(std::string _fn, unsigned _idx) : img_fn{std::move(_fn)}, idx{_idx} { }
  std::string img_fn;
  cv::Mat img;
  unsigned idx = 0;
  std::vector<unsigned> ar_ids;
  std::vector<unsigned> block_idxs;
  std::optional<unsigned> init_block_idx;

  /**
   * inverse transform of pose
   * invert is used by optimize more often when optimizing
   */
  PoseParams inv_pose;

  double* data() {return inv_pose.params.data();}
  const double* data() const {return inv_pose.params.data();}

  cv::Mat loadImg()
  {
    if (img.empty())
    {
      img = cv::imread(img_fn);
      if (img.empty())
      {
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
  Aruco(unsigned _id) : id{_id} { }
  unsigned id = 0;
  bool initialized = false;
  std::vector<unsigned> cap_idxs;
  std::vector<unsigned> block_idxs;
  PoseParams pose;
  double* data() {return pose.params.data();}
  const double* data() const {return pose.params.data();}
};


/**
 * Convert point in capture x,y space to OpenCV coordinate system.
 * For this problem, 0,0 is at center of image, and x,y is normal cordinate system (+y is up)
 * OpenCV uses for images where y-axis increased downward and top-left is corner is 0,0
 * Also optionally scale down coordinates to scaled image
 */
inline cv::Point to_cv_img(double x, double y, const cv::Size& scaled_img_size, float scale=1.0)
{
  const float xc = 0.5 * scaled_img_size.width;
  const float yc = 0.5 * scaled_img_size.height;
  return cv::Point((scale*x + xc), (-scale*y + yc));
}


inline Point from_cv_img(const cv::Point2f& point, const cv::Size& img_size)
{
  return Point
  {
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
  ArucoRect(const std::vector<cv::Point2f>& ar_detect, cv::Size img_size)
  {
    if (ar_detect.size() != corners.size())
    {
      throw std::runtime_error("incorrect number of points");
    }
    for (unsigned idx = 0; idx < corners.size(); ++idx)
    {
      corners[idx] = from_cv_img(ar_detect[idx], img_size);
    }
  }

  std::array<Point, 4> corners;
};


struct Block
{
  Block(unsigned _block_idx,
        ArucoRect& _aruco_rect,
        Capture& _capture,
        Aruco& _aruco) :
    block_idx{_block_idx},
    aruco_rect{_aruco_rect},
    capture{_capture},
    aruco{_aruco}
  {
    connect();
  }

  void connect()
  {
    capture.ar_ids.emplace_back(aruco.id);
    capture.block_idxs.emplace_back(block_idx);
    aruco.cap_idxs.emplace_back(capture.idx);
    aruco.block_idxs.emplace_back(block_idx);
  }

  unsigned block_idx = 0;
  ArucoRect aruco_rect;
  Capture& capture;
  Aruco& aruco;
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
    Point{-1,+1},
    Point{+1,+1},
    Point{+1,-1},
    Point{-1,-1}
};


inline double normalize_angle(double angle)
{
  return std::fmod(std::fmod(angle, 2*M_PI) + 3*M_PI, 2*M_PI) - M_PI;
}


inline bool endswith(const std::string& str, const std::string& suffix)
{
  return (str.size() >= suffix.size()) and (str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0);
}


class ArSlamSolver
{
public:
  ArSlamSolver() = default;

  void loadImages(const std::vector<std::string>& img_fns);

  void loadYaml(const std::string& fn);

  void saveYaml(std::ostream& output) const;

  void displayDebug(const Capture& capture);

  void printCameras() const;

  void compareProjections() const;

  void solve();

  unsigned getNextCaptureIndex() const;

  void localize(unsigned first_loc_cap_idx);

protected:
  void addConnectedCaptures(const Capture& base_capture, std::deque<unsigned>& open_captures);

  void optimize(const Capture& capture);

  void resetProblem();

  cv::Mat checkAndFixImageSize(cv::Mat img);

  ceres::Problem problem_;

  CameraParams camera_;

  std::deque<Capture> captures_;
  std::deque<Aruco> arucos_;
  std::deque<Block> blocks_;

  std::unordered_map<unsigned, Capture&> capture_map_;
  std::unordered_map<unsigned, Aruco&> aruco_map_;

  bool display_debug_ = true;
  bool display_debug_show_all_ar_ = false;
};


#endif  // AR_SLAM_AR_SLAM_UTIL
