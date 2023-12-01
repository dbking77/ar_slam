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

// C++ stdlib
#include <cmath>
#include <deque>
#include <fstream>
#include <unordered_map>
#include <unordered_set>
#include <optional>
#include <vector>

#include "ar_slam/ar_slam_util.hpp"

// OpenCV
#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"

void composeAxisAngle(const double * rot1, const double * rot2, double * out)
{
  double q1[4];
  ceres::AngleAxisToQuaternion(rot1, q1);
  double q2[4];
  ceres::AngleAxisToQuaternion(rot2, q2);
  double q3[4];
  ceres::QuaternionProduct(q1, q2, q3);
  ceres::QuaternionToAngleAxis(q3, out);
}

std::tuple<double, double, double, double>
calcInitValues(const ArucoRect & aruco_rect, double focal)
{
  // guess distance by taking max distance between all corners
  double max_dist_sq = 0.0;
  double avg_x = 0.0;
  double avg_y = 0.0;
  for (unsigned idx = 0; idx < 4; ++idx) {
    const Point & p1 = aruco_rect.corners[idx];
    const Point & p2 = aruco_rect.corners[(idx + 1) & 0x3];
    const double dist_sq = std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2);
    max_dist_sq = std::max(dist_sq, max_dist_sq);
    avg_x += p1.x;
    avg_y += p1.y;
  }
  avg_x *= 0.25;
  avg_y *= 0.25;

  // TODO estimate/guess angle of marker around other x,y axis of rotation

  double avg_angle = 0.0;
  for (unsigned idx = 0; idx < 4; ++idx) {
    const Point & point = aruco_rect.corners[idx];
    const Point & direction = ARUCO_DIRECTIONS[idx];
    const double expected_corner_angle = std::atan2(direction.y, direction.x);
    const double actual_corner_angle = std::atan2(point.y - avg_y, point.x - avg_x);
    const double delta = normalize_angle(actual_corner_angle - expected_corner_angle);
    avg_angle += normalize_angle(delta - avg_angle) / (idx + 1);
  }

  // guess of distance of ar tag in camera frame
  const double local_z = focal * aruco_size / std::sqrt(max_dist_sq);
  const double local_x = avg_x * local_z / focal;
  const double local_y = avg_y * local_z / focal;

  return std::tuple(local_x, local_y, local_z, avg_angle);
}


void initCapturePose(
  const ArucoRect & aruco_rect,
  const double * const camera,
  const double * const ar_pose,
  double * const inv_cap_pose)
{
  const double focal = camera[0];
  auto [local_x, local_y, local_z, local_rot_z] = calcInitValues(aruco_rect, focal);
  double local_position[3] = {local_x, local_y, local_z};
  double local_rot[3] = {0.0, 0.0, local_rot_z};
  double inv_ar_rot[3] = {-ar_pose[3], -ar_pose[4], -ar_pose[5]};
  composeAxisAngle(local_rot, inv_ar_rot, &inv_cap_pose[3]);
  double cap_rotation[3] = {-inv_cap_pose[3], -inv_cap_pose[4], -inv_cap_pose[5]};
  ceres::AngleAxisRotatePoint(cap_rotation, local_position, inv_cap_pose);
  inv_cap_pose[0] -= ar_pose[0];
  inv_cap_pose[1] -= ar_pose[1];
  inv_cap_pose[2] -= ar_pose[2];
}


void initArPose(
  const ArucoRect & aruco_rect,
  const double * const camera,
  const double * const inv_cap_pose,
  double * const ar_pose)
{
  const double focal = camera[0];
  auto [local_x, local_y, local_z, local_rot_z] = calcInitValues(aruco_rect, focal);
  double local_position[3] = {local_x, local_y, local_z};
  double cap_rotation[3] = {-inv_cap_pose[3], -inv_cap_pose[4], -inv_cap_pose[5]};
  ceres::AngleAxisRotatePoint(cap_rotation, local_position, ar_pose);
  ar_pose[0] -= inv_cap_pose[0];
  ar_pose[1] -= inv_cap_pose[1];
  ar_pose[2] -= inv_cap_pose[2];
  double local_rot[3] = {0.0, 0.0, local_rot_z};
  double cap_rot[3] = {-inv_cap_pose[3], -inv_cap_pose[4], -inv_cap_pose[5]};
  composeAxisAngle(cap_rot, local_rot, &ar_pose[3]);
}


template<typename T>
void projectCorner(
  const T * const camera, const T * const inv_cap_pose, const T * const ar_pose,
  const unsigned idx, T * const projected_point)
{
  const auto & direction = ARUCO_DIRECTIONS[idx];

  // +z axis of AR tag is pointed out
  T corner[3];
  corner[0] = T(0.5 * aruco_size * direction.x);
  corner[1] = T(0.5 * aruco_size * direction.y);
  corner[2] = T(0.0);

  T corner_tx[3];
  ceres::AngleAxisRotatePoint(&ar_pose[3], corner, corner_tx);
  corner_tx[0] += ar_pose[0];
  corner_tx[1] += ar_pose[1];
  corner_tx[2] += ar_pose[2];

  // pose 0-2 x,y,z center of ARx tag
  T corner_cam_tx[3];
  corner_tx[0] += inv_cap_pose[0];
  corner_tx[1] += inv_cap_pose[1];
  corner_tx[2] += inv_cap_pose[2];
  ceres::AngleAxisRotatePoint(&inv_cap_pose[3], corner_tx, corner_cam_tx);

  const T corner_cam_x = corner_cam_tx[0] / corner_cam_tx[2];
  const T corner_cam_y = corner_cam_tx[1] / corner_cam_tx[2];

  const T & focal = camera[0];
  projected_point[0] = focal * corner_cam_x;
  projected_point[1] = focal * corner_cam_y;

  /* TODO try modelling camera radial distortion
    const T r2 = ceres::pow(xp,2) + ceres::pow(yp,2);
    const T& l1 = camera[7];
    const T& l2 = camera[8];
    const T distortion = r2 * (l1 + l2*r2) + 1.0;
    const T predicted_x = focal * distortion * xp;
    const T predicted_y = focal * distortion * yp;
  */
}


void compareProjection(
  const ArucoRect & rect, const double * const camera,
  const double * const cap_pose, const double * const ar_pose)
{
  using std::cout;
  using std::endl;
  for (unsigned idx = 0; idx < 4; ++idx) {
    double projected_point[2];
    const auto & corner = rect.corners[idx];
    projectCorner<double>(camera, cap_pose, ar_pose, idx, projected_point);
    cout << "  point " << idx << endl
         << "    x " << corner.x << " dx " << projected_point[0] - corner.x << endl
         << "    y " << corner.y << " dy " << projected_point[1] - corner.y << endl;
  }
}


struct ArucoReprojectionError
{
  ArucoReprojectionError(const ArucoRect & aruco_rect)
  : aruco_rect_{aruco_rect}
  {}

  template<typename T>
  bool operator()(
    const T * const camera, const T * const cap_pose, const T * const ar_pose,
    T * residuals) const
  {
    for (unsigned idx = 0; idx < 4; ++idx) {
      T projected_point[2];
      projectCorner(camera, cap_pose, ar_pose, idx, projected_point);
      const auto & corner = aruco_rect_.corners[idx];
      residuals[idx * 2 + 0] = projected_point[0] - corner.x;
      residuals[idx * 2 + 1] = projected_point[1] - corner.y;
    }
    return true;
  }

private:
  // Observations for a sample.
  ArucoRect aruco_rect_;
};


cv::Mat ArSlamSolver::checkAndFixImageSize(cv::Mat img)
{
  if (camera_.size.has_value()) {
    cv::Size cam_size = camera_.size.value();

    // Auto-rotate image if needed.  Phone cameras will anoyingly rotate image
    if ((img.size().width == cam_size.height) and
      (img.size().height == cam_size.width))
    {
      std::cerr <<
        "WARNING : some images are rotated relative to others fixing by rotating 90 degrees" <<
        std::endl;
      cv::rotate(img, img, cv::ROTATE_90_CLOCKWISE);
    }

    if (img.size() != cam_size) {
      std::ostringstream ss;
      ss << "Loaded images should all be same size : "
         << " expected " << cam_size
         << " got " << img.size();
      throw std::runtime_error(ss.str());
    }
  } else {
    camera_.size = img.size();
  }
  return img;
}


void ArSlamSolver::loadImages(const std::vector<std::string> & img_fns)
{
  cv::Ptr<cv::aruco::DetectorParameters> parameters(new cv::aruco::DetectorParameters);
  parameters->minCornerDistanceRate = 0.1;
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(
    cv::aruco::DICT_4X4_50);

  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> rects, rejected;

  std::optional<cv::Size> expected_size;
  for (const auto & img_fn : img_fns) {
    cv::Mat img = cv::imread(img_fn);
    if (img.empty()) {
      std::ostringstream ss;
      ss << "error loading image " << img_fn;
      throw std::runtime_error(ss.str());
    }

    img = checkAndFixImageSize(img);

    cv::aruco::detectMarkers(img, dictionary, rects, ids, parameters, rejected);

    if (ids.size() <= 2) {
      std::cout << "Warning not enough AR tags detected in " << img_fn << std::endl;
    }

    // make up a Uid for capture that isn't the same as the index
    CaptureUid cap_uid(captures_.size() + 100);
    Capture & capture = addCapture(cap_uid, img_fn);
    capture.img = img;

    for (unsigned ii = 0; ii < ids.size(); ++ii) {
      ArucoId ar_id(ids[ii]);
      Aruco & aruco = getOrAddAruco(ar_id);
      ArucoRect aruco_rect(rects.at(ii), img.size());
      addBlock(aruco_rect, capture.handle, aruco.handle);
    }
  }
}


unsigned ArSlamSolver::getNextCaptureIndex() const
{
  unsigned next_cap_idx = 0;
  for (const Capture & capture : captures_) {
    next_cap_idx = std::max(next_cap_idx, capture.handle.idx + 1);
  }
  return next_cap_idx;
}


void ArSlamSolver::loadYaml(const std::string & fn)
{
  // When loading
  unsigned cap_idx_offset = getNextCaptureIndex();

  YAML::Node doc = YAML::LoadFile(fn);

  YAML::Node captures = doc["captures"];
  for (auto cap_itr : captures) {
    CaptureUid cap_uid(cap_itr.first.as<int>() + cap_idx_offset);
    YAML::Node cap_data = cap_itr.second;
    std::string img_fn = cap_data["img_fn"].as<std::string>();
    Capture & capture = addCapture(cap_uid, img_fn);
    YAML::Node inv_pose_data = cap_data["inv_pose"];
    for (size_t ii = 0; ii < capture.inv_pose.params.size(); ++ii) {
      capture.inv_pose.params[ii] = inv_pose_data[ii].as<double>();
    }
  }

  YAML::Node arucos = doc["arucos"];
  for (auto ar_itr : arucos) {
    ArucoId ar_id(ar_itr.first.as<int>());
    Aruco & aruco = addAruco(ar_id);
    YAML::Node ar_data = ar_itr.second;
    YAML::Node pose_data = ar_data["pose"];
    for (size_t ii = 0; ii < aruco.pose.params.size(); ++ii) {
      aruco.pose.params[ii] = pose_data[ii].as<double>();
    }
  }

  YAML::Node blocks = doc["blocks"];
  for (auto block_data : blocks) {
    CaptureUid cap_uid(block_data["capture"].as<int>() + cap_idx_offset);

    CaptureHandle cap_handle = capture_map_.at(cap_uid);

    ArucoId ar_id(block_data["aruco"].as<int>());
    ArucoHandle & ar_handle = aruco_map_.at(ar_id);

    YAML::Node rect_data = block_data["aruco_rect"];
    ArucoRect aruco_rect;
    if (rect_data.size() != 2 * aruco_rect.corners.size()) {
      throw std::runtime_error("aruco_rect has wrong number of values");
    }
    for (unsigned ii = 0; ii < aruco_rect.corners.size(); ++ii) {
      aruco_rect.corners[ii].x = rect_data[2 * ii + 0].as<double>();
      aruco_rect.corners[ii].y = rect_data[2 * ii + 1].as<double>();
    }
    addBlock(aruco_rect, cap_handle, ar_handle);
  }

  {
    YAML::Node cam_data = doc["camera"];
    camera_.size = cv::Size(
      cam_data["width"].as<int>(),
      cam_data["height"].as<int>());

    YAML::Node cam_params = cam_data["params"];
    for (unsigned ii = 0; ii < cam_params.size(); ++ii) {
      camera_.params.at(ii) = cam_params[ii].as<double>();
    }
  }
}


void ArSlamSolver::saveYaml(std::ostream & output) const
{
  YAML::Emitter y;
  y << YAML::BeginMap;

  // Blocks
  y << YAML::Key << "blocks" << YAML::Value;
  {
    y << YAML::BeginSeq;
    for (const Block & block : blocks_) {
      y << YAML::BeginMap;
      //y << YAML::Key << "capture" << YAML::Value << at(block.capture).uid.uid;
      y << YAML::Key << "capture" << YAML::Value << at(block.capture).uid;
      y << YAML::Key << "aruco" << YAML::Value << at(block.aruco).id;
      y << YAML::Key << "aruco_rect" << YAML::Value << YAML::Flow << YAML::BeginSeq;
      for (const Point & point : block.aruco_rect.corners) {
        y << point.x << point.y;
      }
      y << YAML::EndSeq;
      y << YAML::EndMap;
    }
    y << YAML::EndSeq;
  }

  // Captures
  y << YAML::Key << "captures" << YAML::Value;
  {
    y << YAML::BeginMap;
    for (const Capture & capture : captures_) {
      y << YAML::Key << capture.uid.uid << YAML::Value << YAML::BeginMap;
      {
        // params
        y << YAML::Key << "inv_pose" << YAML::Value << YAML::Flow << YAML::BeginSeq;
        {
          for (double v : capture.inv_pose.params) {
            y << v;
          }
        }
        y << YAML::EndSeq;

        // image filename
        y << YAML::Key << "img_fn" << YAML::Value << capture.img_fn;
      }
      y << YAML::EndMap;
    }
    y << YAML::EndMap;
  }

  // Arucos
  y << YAML::Key << "arucos" << YAML::Value;
  {
    y << YAML::BeginMap;
    for (const Aruco & aruco : arucos_) {
      y << YAML::Key << aruco.id << YAML::Value << YAML::BeginMap;
      {
        // params
        y << YAML::Key << "pose" << YAML::Value << YAML::Flow << YAML::BeginSeq;
        {
          for (double v : aruco.pose.params) {
            y << v;
          }
        }
        y << YAML::EndSeq;
      }
      y << YAML::EndMap;
    }
    y << YAML::EndMap;
  }

  // Camera
  y << YAML::Key << "camera" << YAML::Value;
  {
    y << YAML::BeginMap;
    y << YAML::Key << "params" << YAML::Value << YAML::Flow << YAML::BeginSeq;
    {
      for (double v : camera_.params) {
        y << v;
      }
      y << YAML::EndSeq;
    }
    if (camera_.size.has_value()) {
      y << YAML::Key << "width" << YAML::Value << camera_.size.value().width;
      y << YAML::Key << "height" << YAML::Value << camera_.size.value().height;
    }
    y << YAML::EndMap;
  }

  y << YAML::EndMap;

  if (!y.good()) {
    throw std::runtime_error("Yaml emit is not good");
  }

  output << y.c_str() << std::endl;
}


void ArSlamSolver::displayDebug(const Capture & capture)
{
  if (capture.img.empty()) {
    throw std::runtime_error("image is not loaded for capture");
  }

  double max_img_dim = std::max(capture.img.size().width, capture.img.size().height);

  const double max_dim = 800;
  const double scale = std::min(max_dim / max_img_dim, 1.0);
  std::cerr << "Scaling debug image to " << std::round(scale * 100) << "% of original" << std::endl;

  cv::Mat scaled_img;
  cv::resize(capture.img, scaled_img, cv::Size(), scale, scale);

  auto draw_projected_aruco =
    [&](const Aruco & aruco, const Capture & capture, const cv::Scalar & color)
    {
      double points[4][2];
      for (unsigned idx = 0; idx < 4; ++idx) {
        projectCorner<double>(
          camera_.params.data(),
          capture.data(),
          aruco.data(),
          idx, points[idx]);
      }

      double center_x = 0.0;
      double center_y = 0.0;
      for (unsigned ii = 0; ii < 4; ++ii) {
        const double * p1 = points[ii];
        const double * p2 = points[(ii + 1) % 4];
        auto cv_pt1 = to_cv_img(p1[0], p1[1], scaled_img.size(), scale);
        auto cv_pt2 = to_cv_img(p2[0], p2[1], scaled_img.size(), scale);
        cv::line(
          scaled_img, cv_pt1, cv_pt2,
          color, 2);
        center_x += cv_pt1.x;
        center_y += cv_pt1.y;
      }
      center_x *= 0.25;
      center_y *= 0.25;

      cv::putText(
        scaled_img, std::to_string(aruco.id.id),
        cv::Point(center_x, center_y),
        cv::FONT_HERSHEY_DUPLEX, 0.5,
        color, 1);
    };


  std::unordered_set<ArucoId> detected_ar_ids;
  detected_ar_ids.reserve(capture.blocks.size());

  for (BlockHandle block_handle : capture.blocks) {
    const Block & block = at(block_handle);
    const Aruco & aruco = at(block.aruco);
    const Capture & capture = at(block.capture);
    detected_ar_ids.insert(aruco.id);
    double center_x = 0.0;
    double center_y = 0.0;
    for (unsigned ii = 0; ii < 4; ++ii) {
      const auto & p1 = block.aruco_rect.corners[ii];
      const auto & p2 = block.aruco_rect.corners[(ii + 1) % 4];
      auto cv_pt1 = to_cv_img(p1.x, p1.y, scaled_img.size(), scale);
      auto cv_pt2 = to_cv_img(p2.x, p2.y, scaled_img.size(), scale);
      cv::line(
        scaled_img, cv_pt1, cv_pt2,
        cv::Scalar(250, 0, 250), 2);
      center_x += cv_pt1.x;
      center_y += cv_pt1.y;
    }
    center_x *= 0.25;
    center_y *= 0.25;
    cv::putText(
      scaled_img, std::to_string(aruco.id.id),
      cv::Point(center_x, center_y),
      cv::FONT_HERSHEY_DUPLEX, 0.5,
      cv::Scalar(200, 0, 200), 1);

    draw_projected_aruco(aruco, capture, cv::Scalar(250, 250, 0));
  }

  // When show-all it true, try project all (un-detected) aruco tags onto
  // capture frame and draw ones that that overlap
  if (display_debug_show_all_ar_) {
    for (const Aruco & aruco : arucos_) {
      if (!detected_ar_ids.count(aruco.id)) {
        draw_projected_aruco(aruco, capture, cv::Scalar(0, 250, 250));
      }
    }
  }

  cv::imshow("Debug", scaled_img);
  cv::waitKey(1000 * display_wait_duration_);
}


void ArSlamSolver::printCameras() const
{
  std::cout
    << "\tf=" << camera_.params[0]
    << "\tl1=" << camera_.params[1]
    << "\tl1=" << camera_.params[2]
    << std::endl;
}


void ArSlamSolver::compareProjections() const
{
  for (unsigned idx = 0; idx < blocks_.size(); ++idx) {
    const auto & block = blocks_.at(idx);
    if (block.added) {
      std::cout << "Block " << idx << std::endl;
      compareProjection(
        block.aruco_rect,
        camera_.params.data(),
        at(block.capture).data(),
        at(block.aruco).data());
    }
  }
}

void ArSlamSolver::addDetections(const ar_slam_interfaces::msg::Detections & detections)
{
  if (detections.detections.empty()) {
    return;
  }

  cv::Size image_size(detections.image_width, detections.image_height);
  if (camera_.size.has_value()) {
    if (camera_.size.value() != image_size) {
      std::cerr << "WARN Mismatched image size"
                << " expected " << camera_.size.value()
                << " got " << image_size
                << std::endl;
      return;
    }
  } else {
    camera_.size = image_size;
  }

  // Create new capture for detection
  const unsigned capture_uid_offset = 1000;
  CaptureUid cap_uid(captures_.size() + capture_uid_offset);
  Capture & capture = addCapture(cap_uid, detections.image_path);

  for (const auto & detection : detections.detections) {
    ArucoId ar_id(detection.id);
    Aruco & aruco = getOrAddAruco(ar_id);
    addBlock(ArucoRect{detection}, capture.handle, aruco.handle);
  }

  unsolved_captures_.insert(capture.handle);
}

void ArSlamSolver::solveIncremental()
{
  if (display_debug_) {
    cv::namedWindow("Debug");
    usleep(50 * 1000);

    // Make sure all images are loaded for later
    for (auto & capture : captures_) {
      capture.loadImg();
    }
  }

  // make sure at least one capture is solved
  if (unsolved_captures_.size() == captures_.size()) {
    CaptureHandle cap_handle = *unsolved_captures_.begin();
    std::cout << "Solving initial capture " << cap_handle.idx << std::endl;
    unsolved_captures_.erase(cap_handle);
    Capture & capture = at(cap_handle);
    solveCapture(capture, std::nullopt);
  }

  std::cout << "Solve incremental with " << unsolved_captures_.size() << " unsolved captures" <<
    std::endl;
  bool repeat_solve = false;
  do{
    // search through unsolved captures
    repeat_solve = false;
    //for (unsigned cap_idx : unsolved_captures_)
    for (auto itr = unsolved_captures_.begin(); itr != unsolved_captures_.end(); ++itr) {
      Capture & capture = at(*itr);
      // see if any aruco detected in capture is connected
      //for (unsigned ar_id : capture.ar_ids)
      for (BlockHandle block_handle : capture.blocks) {
        Block & block = at(block_handle);
        Aruco & aruco = at(block.aruco);
        if (aruco.initialized) {
          std::cout << "Capture " << capture.uid << " can be solved through " << aruco.id <<
            std::endl;
          repeat_solve = true;
          itr = unsolved_captures_.erase(itr);
          solveCapture(capture, block_handle);
          break;
        }
      }
      if (itr == unsolved_captures_.end()) {
        break;
      }
    }
  } while (repeat_solve);
}

void ArSlamSolver::solveCapture(Capture & capture, std::optional<BlockHandle> init_block_handle)
{
  // Initialize capture pose using connected block
  if (init_block_handle.has_value()) {
    const Block & block = at(init_block_handle.value());
    const Aruco & aruco = at(block.aruco);
    std::cout << "Initializing capture " << capture.uid
              << " using block " << init_block_handle.value().idx
              << " and ar id " << aruco.id << std::endl;
    initCapturePose(
      block.aruco_rect,
      camera_.params.data(),
      aruco.data(),
      capture.data());
  }

  // Add all blocks for capture to problem
  for (BlockHandle block_handle : capture.blocks) {
    std::cerr << "block " << block_handle.idx << " of " << blocks_.size() << std::endl;
    Block & block = at(block_handle);
    Aruco & aruco = at(block.aruco);
    if (!aruco.initialized) {
      aruco.initialized = true;
      initArPose(
        block.aruco_rect,
        camera_.params.data(),
        capture.data(),
        aruco.data());
    }

    if (!block.added) {
      std::cout << "Adding block " << block_handle.idx << " of " << blocks_.size() << std::endl;
      block.added = true;
      ArucoReprojectionError * cost_functor = new ArucoReprojectionError(block.aruco_rect);
      ceres::CostFunction * cost_function =
        new ceres::AutoDiffCostFunction<ArucoReprojectionError, 8, 3, 6, 6>(cost_functor);
      problem_.AddResidualBlock(
        cost_function, nullptr,
        camera_.params.data(),
        capture.data(),
        aruco.data());
    } else {
      throw std::runtime_error("block for capture was somehow already added?");
    }
  }

  // optimize after adding new captures
  //compareProjections();
  //displayDebug(capture);
  optimize(capture);
  //compareProjections();

  if (display_debug_) {
    displayDebug(capture);
  }
}

void ArSlamSolver::solve()
{
  if (display_debug_) {
    cv::namedWindow("Debug");
    usleep(50 * 1000);

    // Make sure all images are loaded for later
    for (auto & capture : captures_) {
      capture.loadImg();
    }
  }

  std::deque<CaptureHandle> open_captures;

  // first find capture with most ar-tags
  unsigned best_cap_idx = 0;
  {
    unsigned best_ar_count = captures_.front().blocks.size();
    for (unsigned cap_idx = 1; cap_idx < captures_.size(); ++cap_idx) {
      unsigned ar_count = captures_[cap_idx].blocks.size();
      if (ar_count > best_ar_count) {
        best_ar_count = ar_count;
        best_cap_idx = cap_idx;
      }
    }
    std::cout << "using best capture " << best_cap_idx << " with " << best_ar_count << " tags" <<
      std::endl;
  }

  Capture & best_capture = captures_[best_cap_idx];
  best_capture.init_block = BlockHandle(~0); // prevents capture from being added again
  open_captures.emplace_back(best_capture.handle);
  if (false) { // forcing the first capture to be "fixed" actually makes conversion a little bit slower
    problem_.AddParameterBlock(best_capture.data(), best_capture.inv_pose.params.size());
    problem_.SetParameterBlockConstant(best_capture.data());
  }

  if (false) { // TODO debug
    double * pose = best_capture.data();
    // HERE
    //pose[0] = 1.0;
    pose[3] = 1.3;
    pose[5] = 0.3;
  }

  while (!open_captures.empty()) {
    CaptureHandle cap_handle = open_captures.front();
    std::cout << "Processing capture " << cap_handle.idx << std::endl;
    open_captures.pop_front();
    Capture & capture = at(cap_handle);

    // initialize captures pose using a spefic block with ar_tag that
    // has already been initialized
    if (cap_handle.idx != best_cap_idx) {
      BlockHandle init_block_handle = capture.init_block.value();
      const Block & block = at(init_block_handle);
      std::cout << "Initializing capture " << cap_handle.idx
                << " using block " << init_block_handle.idx
                << " and ar id " << at(block.aruco).id << std::endl;
      initCapturePose(
        block.aruco_rect,
        camera_.params.data(),
        at(block.aruco).data(),
        at(block.capture).data());
    }

    // TODO maybe add more than one new capture to problem on each optimize call

    // optimize problem
    for (BlockHandle block_handle : capture.blocks) {
      std::cerr << "block " << block_handle.idx << " of " << blocks_.size() << std::endl;
      Block & block = at(block_handle);
      Aruco & aruco = at(block.aruco);
      if (!aruco.initialized) {
        aruco.initialized = true;
        initArPose(
          block.aruco_rect,
          camera_.params.data(),
          capture.data(),
          aruco.data());
      }

      if (!block.added) {
        std::cout << "Adding block " << block_handle.idx << " of " << blocks_.size() << std::endl;
        block.added = true;
        ArucoReprojectionError * cost_functor = new ArucoReprojectionError(block.aruco_rect);
        ceres::CostFunction * cost_function =
          new ceres::AutoDiffCostFunction<ArucoReprojectionError, 8, 3, 6, 6>(cost_functor);
        problem_.AddResidualBlock(
          cost_function, nullptr,
          camera_.params.data(),
          capture.data(),
          aruco.data());
      } else {
        throw std::runtime_error("block for capture was somehow already added?");
      }
    }

    // optimize after adding new captures
    //compareProjections();
    //displayDebug(capture);
    optimize(capture);
    //compareProjections();

    if (display_debug_) {
      displayDebug(capture);
    }


    if (false) { // TODO debug
      const Block & block = blocks_.at(10);
      std::cerr << "Re-initialize capture with ar tag " << at(block.aruco).id << std::endl;
      initCapturePose(
        block.aruco_rect,
        camera_.params.data(),
        at(block.aruco).data(),
        at(block.capture).data());
      displayDebug(capture);
    }

    addConnectedCaptures(capture, open_captures);
  }
}


void ArSlamSolver::addConnectedCaptures(
  const Capture & base_capture,
  std::deque<CaptureHandle> & open_captures)
{
  for (BlockHandle base_block_handle : base_capture.blocks) {
    Aruco & base_aruco = at(at(base_block_handle).aruco);
    std::cout << "Finding connected captures for ar tag " << base_aruco.id << std::endl;
    for (BlockHandle block_handle : base_aruco.blocks) {
      Block & block = at(block_handle);
      Capture & capture = at(block.capture);
      if (!capture.init_block.has_value()) {
        capture.init_block = block_handle;
        open_captures.emplace_back(capture.handle);
      }
    }
  }
}


void ArSlamSolver::localizeMany(unsigned first_loc_cap_idx)
{
  if (display_debug_) {
    // when debugging localization try displaying all ar tags
    display_debug_show_all_ar_ = true;
    cv::namedWindow("Debug");
    usleep(50 * 1000);
  }

  for (unsigned cap_idx = first_loc_cap_idx; cap_idx < captures_.size(); ++cap_idx) {
    // This capture was part of mapping process
    localizeOne(captures_.at(cap_idx), first_loc_cap_idx);
  }
}

void ArSlamSolver::localizeOne(Capture & capture, unsigned first_loc_cap_idx)
{
  if (display_debug_) {
    capture.loadImg();
  }

  // before optimizing find an ar tag that is shared with
  // one of the "mapping" captures
  std::optional<BlockHandle> map_block_handle = [&]()
    {
      for (BlockHandle connected_block_handle : capture.blocks) {
        const Aruco & aruco = at(at(connected_block_handle).aruco);
        for (BlockHandle block_handle : aruco.blocks) {
          Capture & capture = at(at(block_handle).capture);
          if (capture.handle.idx < first_loc_cap_idx) {
            std::cout << "Capture " << capture.handle.idx
                      << " shares aruco " << aruco.id
                      << " with map capture " << capture.handle.idx
                      << std::endl;
            return std::optional<BlockHandle>(connected_block_handle);
          }
        }
      }
      return std::optional<BlockHandle>();
    }();

  if (!map_block_handle.has_value()) {
    std::cout << "WARNING : Cannot find connected ar tags for capture " << capture.handle.idx <<
      std::endl;
    return;
  }

  resetProblem();

  // Initialize capture with connected block
  {
    const Block & block = at(map_block_handle.value());
    std::cout << "Initializing capture " << capture.handle.idx
              << " using block " << map_block_handle.value().idx
              << " and ar id " << at(block.aruco).id << std::endl;
    initCapturePose(
      block.aruco_rect,
      camera_.params.data(),
      at(block.aruco).data(),
      at(block.capture).data());
  }

  for (BlockHandle block_handle : capture.blocks) {
    Block & block = at(block_handle);
    if (!block.added) {
      Aruco & aruco = at(block.aruco);
      std::cout << "Adding block " << block_handle.idx << " of " << blocks_.size() << std::endl;
      block.added = true;
      ArucoReprojectionError * cost_functor = new ArucoReprojectionError(block.aruco_rect);
      ceres::CostFunction * cost_function =
        new ceres::AutoDiffCostFunction<ArucoReprojectionError, 8, 3, 6, 6>(cost_functor);
      problem_.AddResidualBlock(
        cost_function, nullptr,
        camera_.params.data(),
        capture.data(),
        aruco.data());
      // All aruco tags should be constant
      problem_.SetParameterBlockConstant(aruco.data());
    } else {
      throw std::runtime_error("block for capture was somehow already added?");
    }
  }

  // Set camera block constant
  problem_.SetParameterBlockConstant(camera_.params.data());

  optimize(capture);

  if (display_debug_) {
    displayDebug(capture);
  }
}


struct DisplayDebugIterationCallback : ceres::IterationCallback
{
  ArSlamSolver & solver;
  const Capture & capture;
  DisplayDebugIterationCallback(ArSlamSolver & _solver, const Capture & _capture)
  : solver{_solver}, capture{_capture}
  {
  }

  ceres::CallbackReturnType operator()(const ceres::IterationSummary & summary)
  {
    if (summary.iteration == 0) {
      solver.displayDebug(capture);
    }
    return ceres::CallbackReturnType::SOLVER_CONTINUE;
  }
};


void ArSlamSolver::optimize(const Capture & capture)
{
  ceres::Solver::Options options;
  options.max_num_iterations = 50;

  if (display_debug_) {
    options.update_state_every_iteration = true;
    options.callbacks.emplace_back(new DisplayDebugIterationCallback(*this, capture));
  }

  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  std::cout << "Starting solver..." << std::endl;
  ceres::Solve(options, &problem_, &summary);
  //std::cout << summary.BriefReport() << std::endl;
  //std::cout << summary.FullReport() << std::endl;
}


void ArSlamSolver::resetProblem()
{
  problem_.~Problem();
  new (&problem_) ceres::Problem();
}

std::vector<geometry_msgs::msg::TransformStamped> ArSlamSolver::getTransforms(
  const rclcpp::Time & stamp) const
{
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  transforms.reserve(captures_.size() + arucos_.size());

  for (const Aruco & aruco: arucos_) {
    transforms.emplace_back();
    auto & t = transforms.back();
    t.header.stamp = stamp;
    t.header.frame_id = "world";
    t.child_frame_id = "aruco_" + std::to_string(aruco.id);

    const auto & pose = aruco.pose.params;
    t.transform.translation.x = pose.at(0);
    t.transform.translation.y = pose.at(1);
    t.transform.translation.z = pose.at(2);
    double q[4];
    ceres::AngleAxisToQuaternion(&pose.at(3), q);
    // Ceres uses w,x,y,z ordering for its quaternions
    t.transform.rotation.w = q[0];
    t.transform.rotation.x = q[1];
    t.transform.rotation.y = q[2];
    t.transform.rotation.z = q[3];
  }

  for (const Capture & capture: captures_) {
    transforms.emplace_back();
    auto & t = transforms.back();
    t.header.stamp = stamp;
    t.header.frame_id = "world";
    t.child_frame_id = "capture_" + std::to_string(capture.uid);

    const auto & inv_pose = capture.inv_pose.params;
    t.transform.translation.x = -inv_pose.at(0);
    t.transform.translation.y = -inv_pose.at(1);
    t.transform.translation.z = -inv_pose.at(2);
    double rot[3] = {-inv_pose.at(3), -inv_pose.at(4), -inv_pose.at(5)};
    double q[4];
    ceres::AngleAxisToQuaternion(rot, q);
    // Ceres uses w,x,y,z ordering for its quaternions
    t.transform.rotation.w = q[0];
    t.transform.rotation.x = q[1];
    t.transform.rotation.y = q[2];
    t.transform.rotation.z = q[3];
  }

  return transforms;
}


bool endswith(const std::string & str, const std::string & suffix)
{
  return (str.size() >=
         suffix.size()) and (str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0);
}


std::string filename_no_ext(std::string fp)
{
  size_t startp = fp.find_last_of('/');
  if (startp != fp.npos) {
    fp = fp.substr(startp + 1, fp.npos);
  }
  size_t endp = fp.find_last_of('.');
  if (endp != fp.npos) {
    fp = fp.substr(0, endp);
  }
  return fp;
}
