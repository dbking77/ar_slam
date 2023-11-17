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


// 9 points
struct CameraParams
{
  CameraParams()
  {
    std::fill(params.begin(), params.end(), 0.0);
    params[0] = 3000.0; // non-zero focal length
  }
  // 3 f,l1,l2
  std::array<double, 3> params;
};

struct PoseParams
{
  PoseParams()
  {
    std::fill(params.begin(), params.end(), 0.0);
  }
  // 3 x,y,z, 3 axis-angle,
  std::array<double, 6> params;
};

/// Represents a single capture from a camera
struct Capture
{
  //Capture() = default;
  Capture(std::string _fn, unsigned _idx) : img_fn{std::move(_fn)}, idx{_idx} { }
  std::string img_fn;
  cv::Mat img;
  unsigned idx = 0;
  std::vector<unsigned> ar_ids;
  std::vector<unsigned> block_idxs;
  std::optional<unsigned> init_block_idx;
  // inverse transform of pose
  PoseParams inv_pose;
  double* data() {return inv_pose.params.data();}

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
};


struct Point
{
  Point() = default;
  Point(double _x, double _y) : x{_x}, y{_y} {}
  double x = 0.0;
  double y = 0.0;
};


inline cv::Point to_cv_img(double x, double y, const cv::Size& scaled_img_size, float scale=1.0)
{
  const float xc = 0.5 * scaled_img_size.width;
  const float yc = 0.5 * scaled_img_size.height;
  return cv::Point((scale*x + xc), (-scale*y + yc));
}

struct ArucoRect
{
  ArucoRect() = default;

  ArucoRect(const std::vector<cv::Point2f>& ar_detect, cv::Size img_size)
  {
    if (ar_detect.size() != corners.size())
    {
      throw std::runtime_error("incorrect number of points");
    }
    const float xc = 0.5 * img_size.width;
    const float yc = 0.5 * img_size.height;
    for (unsigned idx = 0; idx < corners.size(); ++idx)
    {
      corners[idx].x = +(ar_detect[idx].x - xc);
      corners[idx].y = -(ar_detect[idx].y - yc);
    }
  }

  void load(std::istream& input)
  {
    for (auto& corner : corners)
    {
      input >> corner.x >> corner.y;
      std::cout << "  corner " << corner.x << ',' << corner.y << std::endl;
    }
    if (input.bad())
    {
      throw std::runtime_error("error loading aruco_rect data");
    }
  }

  std::array<Point, 4> corners;
};

struct ArucoReprojectionError;

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
  ArucoReprojectionError* cost_functor = nullptr;
};


// 2.5 inches
static constexpr double aruco_size = 0.0635;

// point ordering
// top left, top right, bottom right, bottom left
// -x,+y,  +x,+y, +x,-y, -x,-y
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
  return (str.size() >= suffix.size()) and (0 == str.compare(str.size()-suffix.size(), suffix.size(), suffix));
}


class BalProblem
{
public:
  BalProblem() = default;

  void loadImages(const std::vector<std::string>& img_fns);

  void loadYaml(const std::string& fn);

  void saveYaml(std::ostream& output) const;

  void loadCsv(std::ifstream& input);

  void displayDebug(const Capture& capture);

  void printCameras() const;

  void compareProjections() const;

  void solve();

  void resetProblem();

protected:
  void addConnectedCaptures(const Capture& base_capture, std::deque<unsigned>& open_captures);

  void optimize(const Capture& capture);

  ceres::Problem problem_;

  CameraParams camera_;

  std::deque<Capture> captures_;
  std::deque<Aruco> arucos_;
  std::deque<Block> blocks_;

  std::unordered_map<unsigned, Capture&> capture_map_;
  std::unordered_map<unsigned, Aruco&> aruco_map_;

  bool display_debug_ = true;
};
