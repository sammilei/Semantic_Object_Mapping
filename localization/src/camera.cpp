#include "artifact_localization/camera.h"

#include <algorithm>

#include <librealsense2/rsutil.h>
#include <sensor_msgs/point_cloud2_iterator.h>

Camera::Camera(const std::string& name, const CameraConfig& config)
  : name_(name), config_(config) {
  initializeBuffers();
}

Camera::~Camera() {}

void Camera::initializeBuffers() {
  // Debug image buffers
  debug_img_bufs_["depth"] = cv::Mat();
  debug_img_encs_["depth"] = "mono8";
  debug_img_bufs_["depth_w_bb"] = cv::Mat();
  debug_img_encs_["depth_w_bb"] = "bgr8";
  debug_img_bufs_["depth_w_bb_plus_rgb"] = cv::Mat();
  debug_img_encs_["depth_w_bb_plus_rgb"] = "bgr8";
}

void Camera::setIntrinsicsFromCameraInfo(
    const sensor_msgs::CameraInfoConstPtr info) {
  model_.fromCameraInfo(info);
}

rs2_intrinsics Camera::getRSIntrinsics() const {
  rs2_intrinsics intrin;
  intrin.fx = model_.fx();
  intrin.fy = model_.fy();
  intrin.ppx = model_.cx();
  intrin.ppy = model_.cy();
  intrin.width = model_.cameraInfo().width;
  intrin.height = model_.cameraInfo().height;
  intrin.model = RS2_DISTORTION_NONE;
  return intrin;
}

bool Camera::isDetectionValid(
    const darknet_ros_msgs::ObjectConstPtr& msg) const {
  confidence_thresholds thresholds;
  try {
    thresholds = config_.confidence_thresholds_.at(msg->box.Class);
  } catch (const std::out_of_range &ex) {
    ROS_WARN("No confidence thresholds found for artifact detection message "
             "class %s!",
             msg->box.Class.c_str());
    return false;
  }
  double confidence_min = thresholds.confidence_min;
  double confidence_max = thresholds.confidence_max;
  if (msg->box.probability < confidence_min ||
      msg->box.probability > confidence_max) {
    return false;
  }
  return true;
}

Eigen::Vector2d
Camera::project3dToPixel(const Eigen::Vector3d& position) const {
  cv::Point3d point(position(0), position(1), position(2));
  auto pixel = model_.project3dToPixel(point);

  return Eigen::Vector2d(pixel.x, pixel.y);
}

bool Camera::isInFOV(const Eigen::Vector3d& position) const {
  // Given position is behind the camera
  if (position(2) < 0) {
    return false;
  }

  // Check projection is within the camera FOV
  auto pixel = project3dToPixel(position);
  auto size = model_.fullResolution();

  return pixel(0) >= 0 && pixel(0) < size.width && pixel(1) >= 0 &&
      pixel(1) < size.height;
}

bool Camera::isInFOV(const Eigen::Vector2d& pixel) const {
  auto size = model_.fullResolution();
  return pixel(0) >= 0 && pixel(0) < size.width && pixel(1) >= 0 &&
      pixel(1) < size.height;
}

bool Camera::isInBB(const Eigen::Vector3d& position,
                    darknet_ros_msgs::BoundingBox bb) const {
  auto pixel = project3dToPixel(position);
  return pixel(0) >= bb.xmin && pixel(1) >= bb.ymin && pixel(0) <= bb.xmax &&
      pixel(1) <= bb.ymax;
}

bool Camera::isInBB(const Eigen::Vector2d& pixel,
                    darknet_ros_msgs::BoundingBox bb) const {
  return pixel(0) >= bb.xmin && pixel(1) >= bb.ymin && pixel(0) <= bb.xmax &&
      pixel(1) <= bb.ymax;
}

bool Camera::getBearingFromBoundingBox(const cv::Rect& bbox,
                                       Eigen::Vector3d& bearing,
                                       double& bearing_sigma) const {
  // Compute pixel of the localization point
  auto bb_centroid = getBoundingBoxCentroid(bbox);

  // Project to a 3D ray
  auto ray = model_.projectPixelTo3dRay(bb_centroid);
  bearing = Eigen::Vector3d(ray.x, ray.y, ray.z).normalized();
  bearing_sigma = config_.bearing_sigma;
  return true;
}

// get the height and the width of the bounding box in the real world based on
// the depth and camera parameters
void Camera::getBboxSize(const double x1,
                         const double x2,
                         const double y1,
                         const double y2,
                         const double range,
                         const rs2_intrinsics camera_params,
                         double& height,
                         double& width) {
  double top_left_real_world_x, top_left_real_world_y;
  double top_right_real_world_x, top_right_real_world_y;
  double bot_left_real_world_x, bot_left_real_world_y;

  cv::Point3d top_left_real_world_xy =
      model_.projectPixelTo3dRay(cv::Point2d(x1, y1)) * range;
  cv::Point3d top_right_real_world_xy =
      model_.projectPixelTo3dRay(cv::Point2d(x2, y1)) * range;
  cv::Point3d bot_left_real_world_xy =
      model_.projectPixelTo3dRay(cv::Point2d(x1, y2)) * range;
  top_left_real_world_x = top_left_real_world_xy.x;
  top_left_real_world_y = top_left_real_world_xy.y;
  top_right_real_world_x = top_right_real_world_xy.x;
  top_right_real_world_y = top_right_real_world_xy.y;
  bot_left_real_world_x = bot_left_real_world_xy.x;
  bot_left_real_world_y = bot_left_real_world_xy.y;

  width = (double)top_right_real_world_x - top_left_real_world_x;
  height = -((double)top_left_real_world_y - bot_left_real_world_y);
}

void Camera::getScoreFromSizeAndLimits(const double size,
                                       const double min,
                                       const double min_min,
                                       const double max,
                                       const double max_max,
                                       double& score) {
  if (size < min_min) {
    score = 0;
  } else if (size <= min) {
    score = ((double)size - (double)min_min) / ((double)min - (double)min_min);
  } else if (size <= max) {
    score = 1;
  } else if (size <= max_max) {
    score = ((double)max_max - (double)size) / ((double)max_max - (double)max);
  } else {
    score = 0;
  }
}

void Camera::combineHeightWidthScore(const double height_score,
                                     const double width_score,
                                     double& size_score) {
  size_score = height_score * width_score;
}

// get the size score from the height, width and label
void Camera::getSizeScoreFromArtifactDimensions(const double height,
                                                const double width,
                                                const double range,
                                                const std::string label,
                                                double& size_score) {
  double height_score, width_score, height_score_turned, width_score_turned,
      size_score_turned;

  // not all artifacts can have a size score
  if (label == "Helmet" || label == "Drill" || label == "Fire Extinguisher" ||
      label == "Rope" || label == "Survivor" || label == "Backpack" ||
      label == "Cube") {
    // get the size limits
    size_configuration size_limits =
        config_.artifact_size_limits_for_size_score.at(label);
    int height_min_min = size_limits.height_limits[0];
    int height_min = size_limits.height_limits[1];
    int height_max = size_limits.height_limits[2];
    int height_max_max = size_limits.height_limits[3];

    int width_min_min = size_limits.width_limits[0];
    int width_min = size_limits.width_limits[1];
    int width_max = size_limits.width_limits[2];
    int width_max_max = size_limits.width_limits[3];
    double max_depth_check = size_limits.max_depth_allowed;

    if (range > max_depth_check) {
      // if we are too far from the artifact to have reliable depth
      ROS_WARN("This artifact is very far (%.2f while max allowed is %.2f) so "
               "no size score generated",
               range,
               max_depth_check);
      size_score = -1;
    } else {
      int height_artifact = height * 100; // convert to cm
      int width_artifact = width * 100;

      // height score
      getScoreFromSizeAndLimits(height_artifact,
                                height_min,
                                height_min_min,
                                height_max,
                                height_max_max,
                                height_score);
      getScoreFromSizeAndLimits(width_artifact,
                                width_min,
                                width_min_min,
                                width_max,
                                width_max_max,
                                width_score);
      combineHeightWidthScore(height_score, width_score, size_score);

      // if low confidence in the size, check if the artifact is in an unusual
      // position (90° rotation)
      if ((double)height_score * (double)width_score < 0.8) {
        getScoreFromSizeAndLimits(width_artifact,
                                  height_min,
                                  height_min_min,
                                  height_max,
                                  height_max_max,
                                  height_score_turned);
        getScoreFromSizeAndLimits(height_artifact,
                                  width_min,
                                  width_min_min,
                                  width_max,
                                  width_max_max,
                                  width_score_turned);
        combineHeightWidthScore(
            height_score_turned, width_score_turned, size_score_turned);

        if (0.8 * size_score_turned > size_score) {
          size_score = 0.8 * size_score_turned;
          height_score = height_score_turned;
          width_score = width_score_turned;
          ROS_WARN("Had to turn this artifact for the color score");
        }
      }
    }
  } else {
    ROS_WARN("The size score not supported for this artifact: %s",
             label.c_str());
    size_score = -1;
  }
}

// from the bounding box and the depth, get the dimensions of the object and
// thus also the associated depth
bool Camera::getSizeScore(const double range,
                          const double range_sigma,
                          const darknet_ros_msgs::BoundingBox& bbox,
                          double& height,
                          double& width,
                          double& size_score) {
  rs2_intrinsics camera_params;
  camera_params = getRSIntrinsics();

  // bounding box
  double x1 = bbox.xmin;
  double x2 = bbox.xmax;
  double y1 = bbox.ymin;
  double y2 = bbox.ymax;

  double bbox_width, bbox_height;
  // get the height and the width of the bounding box in the real world based on
  // the depth and camera parameters
  getBboxSize(x1, x2, y1, y2, range, camera_params, height, width);

  std::string label = bbox.Class;
  if (!isBoxOnEdge(x1, x2, y1, y2, camera_params)) {
    // get the size score from the height, width and label
    getSizeScoreFromArtifactDimensions(height, width, range, label, size_score);
  } else {
    ROS_INFO("Artifact on the edge of image, no size score");
    size_score = -1;
  }
  if (size_score == -1) {
    return false;
  } else {
    return true;
  }
}

// return true if he bounding box is on the endge of the image and thus
// the size score is unreliable and should not be relied on
bool Camera::isBoxOnEdge(const double x1,
                         const double x2,
                         const double y1,
                         const double y2,
                         const rs2_intrinsics camera_params) {
  double camera_width = camera_params.width;
  double camera_height = camera_params.height;

  double x_lower_bound = 0.02 * camera_width;
  double x_upper_bound = 0.98 * camera_width;
  double y_lower_bound = 0.02 * camera_height;
  double y_upper_bound = 0.98 * camera_height;

  if (x1 < x_lower_bound) {
    return true;
  } else if (y1 < y_lower_bound) {
    return true;
  } else if (x2 > x_upper_bound) {
    return true;
  } else if (y2 > y_upper_bound) {
    return true;
  }
  return false;
}

bool Camera::doubleTruncatedMean(const std::vector<double> vec,
                                 const double close_percentile,
                                 const double far_percentile,
                                 double& truncated_mean) const {
  if (vec.empty()) {
    return false;
  }
  std::vector<double> vec_copy = vec;
  int start_index = (int)floor((double)vec_copy.size() * close_percentile);
  int end_index = (int)ceil((double)vec_copy.size() * far_percentile);
  // keep only closest part of depth values & take mean
  std::sort(vec_copy.begin(), vec_copy.end());
  std::vector<double> closest_depth_values(end_index - start_index);
  std::copy(vec_copy.begin() + start_index,
            vec_copy.begin() + end_index,
            closest_depth_values.begin());
  if (closest_depth_values.empty()) {
    return false;
  }
  double sum = std::accumulate(
      closest_depth_values.begin(), closest_depth_values.end(), 0.0);
  truncated_mean = sum / closest_depth_values.size();

  return true;
}

bool Camera::doubleTruncatedMedian(const std::vector<double> vec,
                                   const double close_percentile,
                                   const double far_percentile,
                                   double& truncated_median) const {
  if (vec.empty()) {
    return false;
  }
  std::vector<double> vec_copy = vec;
  int start_index = (int)floor((double)vec_copy.size() * close_percentile);
  int end_index = (int)ceil((double)vec_copy.size() * far_percentile);
  // keep only closest part of depth values & take mean
  std::sort(vec_copy.begin(), vec_copy.end());
  std::vector<double> closest_depth_values(end_index - start_index);
  std::copy(vec_copy.begin() + start_index,
            vec_copy.begin() + end_index,
            closest_depth_values.begin());
  auto size = closest_depth_values.size();
  if (size <= 0) {
    return false;
  }
  if (size % 2 == 0) {
    truncated_median =
        (closest_depth_values[size / 2 - 1] + closest_depth_values[size / 2]) /
        2;
  } else {
    truncated_median = closest_depth_values[size / 2];
  }

  return true;
}

bool Camera::doubleNearestNeighbors(
    const std::vector<double> vec,
    const double query,
    const double radius,
    std::vector<double>& nearest_neighbors) const {
  if (vec.empty()) {
    return false;
  }
  for (auto& val : vec) {
    if (abs(query - val) <= radius) {
      nearest_neighbors.push_back(val);
    }
  }
  if (nearest_neighbors.empty()) {
    return false;
  }

  return true;
}

// Compute range for the bounding box by taking the median of valid values [old
// version] Compute range for the bounding box by taking the mean of the 15 to
// 35% closest valid values [new version]
bool Camera::getRangeFromBoundingBox(
    const std::string label,
    const darknet_ros_msgs::BoundingBox& bbox,
    const sensor_msgs::ImageConstPtr& depth_msg,
    double& range,
    double& range_sigma) const {
  // Get depth image size from message
  auto depth_img_size = cv::Size(depth_msg->width, depth_msg->height);

  // Scale bounding box to model
  auto bbox_ = scaleBoundingBoxToModel(bbox, depth_img_size);

  cv::Mat roi(toCV(*depth_msg), toCV(bbox_));
  std::vector<double> depth_values;
  depth_values.reserve(roi.rows * roi.cols);
  roi.forEach<uint16_t>([&depth_values](uint16_t& value, const int position[]) {
    const double depth_scale = 0.001;
    if (value != 0 && !std::isnan(value)) {
      depth_values.push_back(depth_scale * value);
    }
  });
  if (depth_values.size() < config_.depth_min_points) {
    return false;
  }

  // commenting this out as this is for the mean [old version]
  // std::nth_element(depth_values.begin(),
  //                 depth_values.begin() + depth_values.size() / 2,
  //                 depth_values.end());
  // range = depth_values[depth_values.size() / 2]; // this is the median

  // find the range as mean of closest part of bounding box
  depth_percentile percentiles = config_.depth_percentile_.at(label);
  double close_percentile = percentiles.close_percentile;
  double far_percentile = percentiles.far_percentile;
  try {
    auto success = doubleTruncatedMean(
        depth_values, close_percentile, far_percentile, range);
    assert(success);
  } catch (const std::exception& ex) {
    ROS_ERROR("Failed to compute truncated mean when getting depth image range "
              "bounding box: %s",
              ex.what());
    return false;
  }
  range_sigma = config_.depth_range_sigma_coeff0 +
      config_.depth_range_sigma_coeff1 * range;

  // Sanity check
  if (range < config_.depth_min || range > config_.depth_max) {
    ROS_WARN("Rejecting out-of-range depth: %.1fm", range);
    return false;
  }
  return true;
}

bool Camera::getRangeFromBoundingBox(
    const darknet_ros_msgs::BoundingBox& bbox,
    const sensor_msgs::PointCloud2ConstPtr& lidar_msg,
    const sensor_msgs::Image& rgb_msg,
    double& range,
    double& range_sigma) {
  // Get class label from bbox msg
  auto label = bbox.Class;

  // Get rgb image size from message
  auto rgb_img_size = cv::Size(rgb_msg.width, rgb_msg.height);

  // Get camera model size
  auto model_size = model_.fullResolution();

  // Scale bounding box to model
  auto bbox_ = scaleBoundingBoxToModel(bbox, rgb_img_size, model_size);

  // Convert rgb message to opencv image
  cv::Mat rgb_img;
  if (!rgb_msg.data.empty()) {
    rgb_img = toCV(rgb_msg);
  } else {
    rgb_img = cv::Mat();
  }

  // Find points that lie within bounding box and generate debug depth images
  const double depth_scale = 1.0;
  std::vector<double> depth_values;
  cv::Mat depth_image =
      cv::Mat(model_size.height, model_size.width, CV_32FC1, cv::Scalar(0));
  cv::Mat depth_w_bb_image = cv::Mat(
      model_size.height, model_size.width, CV_32FC3, cv::Scalar(0, 0, 0));
  cv::Mat depth_w_bb_plus_rgb_image;
  if (!rgb_img.empty()) {
    depth_w_bb_plus_rgb_image = rgb_img.clone();
    // Scale to match camera model size
    if (rgb_img_size.width != model_size.width ||
        rgb_img_size.height != model_size.height) {
      cv::resize(depth_w_bb_plus_rgb_image,
                 depth_w_bb_plus_rgb_image,
                 depth_w_bb_image.size());
    }
    depth_w_bb_plus_rgb_image.convertTo(depth_w_bb_plus_rgb_image, CV_32FC3);
  }
  for (sensor_msgs::PointCloud2ConstIterator<float> point_iter(*lidar_msg, "x");
       point_iter != point_iter.end();
       ++point_iter) {
    Eigen::Vector3d point(point_iter[0], point_iter[1], point_iter[2]);

    // Project the 3D point to 2D pixel in the camera frame
    auto pixel = project3dToPixel(point);

    if (point(2) >= 0 && isInFOV(pixel)) {
      // If the pixel lies within the bounding box, add it to the list
      if (isInBB(pixel, bbox_)) {
        if (point(2) != 0 && !std::isnan(point(2))) {
          depth_values.push_back(depth_scale * point(2));
        }
      }

      if (!rgb_img.empty()) {
        depth_image.at<float>(pixel[1], pixel[0]) = point(2) * 255.0;
        if (isInBB(pixel, bbox_)) {
          // If the pixel lies within the bounding box, color it red
          depth_w_bb_image.at<cv::Vec3f>(pixel[1], pixel[0])[0] = 0.0;
          depth_w_bb_image.at<cv::Vec3f>(pixel[1], pixel[0])[1] = 0.0;
          depth_w_bb_image.at<cv::Vec3f>(pixel[1], pixel[0])[2] =
              point(2) * 255.0;
          if (!depth_w_bb_plus_rgb_image.empty()) {
            depth_w_bb_plus_rgb_image.at<cv::Vec3f>(pixel[1], pixel[0])[0] =
                0.0;
            depth_w_bb_plus_rgb_image.at<cv::Vec3f>(pixel[1], pixel[0])[1] =
                0.0;
            depth_w_bb_plus_rgb_image.at<cv::Vec3f>(pixel[1], pixel[0])[2] =
                point(2) * 255.0;
          }
        } else {
          // If the pixel lies outside the bounding box, color it white
          depth_w_bb_image.at<cv::Vec3f>(pixel[1], pixel[0])[0] =
              point(2) * 255.0;
          depth_w_bb_image.at<cv::Vec3f>(pixel[1], pixel[0])[1] =
              point(2) * 255.0;
          depth_w_bb_image.at<cv::Vec3f>(pixel[1], pixel[0])[2] =
              point(2) * 255.0;
          if (!depth_w_bb_plus_rgb_image.empty()) {
            depth_w_bb_plus_rgb_image.at<cv::Vec3f>(pixel[1], pixel[0])[0] =
                point(2) * 255.0;
            depth_w_bb_plus_rgb_image.at<cv::Vec3f>(pixel[1], pixel[0])[1] =
                point(2) * 255.0;
            depth_w_bb_plus_rgb_image.at<cv::Vec3f>(pixel[1], pixel[0])[2] =
                point(2) * 255.0;
          }
        }
      }
    }
  }

  // Find the foreground depth values
  double median_closest_depth_value;
  std::vector<double> foreground_depth_values;
  if (!depth_values.empty()) {
    // Find the median of epsilon-percentile of the closest depth values to the
    // camera
    try {
      depth_percentile percentiles = config_.foreground_depth_percentiles;
      double close_percentile = percentiles.close_percentile;
      double far_percentile = percentiles.far_percentile;
      try {
        auto success = doubleTruncatedMedian(depth_values,
                                             close_percentile,
                                             far_percentile,
                                             median_closest_depth_value);
        assert(success);
      } catch (const std::exception& ex) {
        ROS_ERROR("Failed to compute truncated median when getting lidar range "
                  "from bounding box: %s",
                  ex.what());
        return false;
      }
      ROS_DEBUG("Successfully computed median of closest depth values to "
                "camera in [%f, %f] percentiles: %f meters",
                close_percentile,
                far_percentile,
                median_closest_depth_value);
    } catch (const std::exception& ex) {
      ROS_ERROR(
          "Failed to compute median of closest depth values to camera: %s",
          ex.what());
      return false;
    }

    // Find the foreground nearest-neighbor depth values within a given radius
    // from the median closest depth value
    try {
      auto success =
          doubleNearestNeighbors(depth_values,
                                 median_closest_depth_value,
                                 config_.foreground_depth_nn_dist_to_median,
                                 foreground_depth_values);
      auto is_empty = foreground_depth_values.empty();
      assert(success);
      assert(!is_empty);
      ROS_DEBUG(
          "Successfully computed foreground nearest-neighbor depth "
          "values within radius %f from query depth %f: %d out of %d points",
          config_.foreground_depth_nn_dist_to_median,
          median_closest_depth_value,
          (int)foreground_depth_values.size(),
          (int)depth_values.size());
    } catch (const std::exception& ex) {
      ROS_ERROR("Failed to compute nearest-neighbor depth values to median of "
                "closest depth values to camera: %s",
                ex.what());
      return false;
    }
  } else {
    ROS_WARN("Unable to calculate foreground depth values: no lidar points "
             "within bounding box!");
    return false;
  }

  // Find the range as mean of closest part of bounding box
  if (!foreground_depth_values.empty()) {
    try {
      depth_percentile percentiles = config_.depth_percentile_.at(label);
      double close_percentile = percentiles.close_percentile;
      double far_percentile = percentiles.far_percentile;
      doubleTruncatedMean(
          foreground_depth_values, close_percentile, far_percentile, range);
      range_sigma = config_.depth_range_sigma_coeff0 +
          config_.depth_range_sigma_coeff1 * range;
    } catch (const std::exception& ex) {
      ROS_ERROR("Failed to compute mean of depth values in bounding box: %s",
                ex.what());
      return false;
    }
  } else {
    ROS_WARN("Unable to calculate mean: no lidar points within bounding box!");
    return false;
  }

  // Sanity check
  if (range < config_.depth_min || range > config_.depth_max) {
    ROS_WARN("Rejecting out-of-range depth: %.1fm", range);
    return false;
  }

  // Create debug image output
  if (!rgb_img.empty()) {
    try {
      depth_image.convertTo(depth_image, CV_8UC1);
      debug_img_bufs_["depth"] = depth_image;
      ROS_DEBUG("Successfully created %s-rectified LiDAR depth image.",
                name_.c_str());
    } catch (const std::exception& ex) {
      ROS_ERROR("Failed to create %s-rectified LiDAR depth image: %s",
                name_.c_str(),
                ex.what());
    }

    try {
      depth_w_bb_image.convertTo(depth_w_bb_image, CV_8UC3);
      // Add a text overlay to the image
      cv::putText(depth_w_bb_image,
                  "Observation: " + label,
                  cvPoint(10, 20),
                  cv::FONT_HERSHEY_COMPLEX_SMALL,
                  0.8,
                  cvScalar(0, 255, 0),
                  1,
                  CV_AA);
      std::ostringstream range_ss;
      range_ss << std::fixed;
      range_ss << std::setprecision(2);
      range_ss << range;
      cv::putText(depth_w_bb_image,
                  "Range: " + range_ss.str() + "m",
                  cvPoint(10, 40),
                  cv::FONT_HERSHEY_COMPLEX_SMALL,
                  0.8,
                  cvScalar(0, 255, 0),
                  1,
                  CV_AA);
      std::ostringstream bb_min_ss;
      bb_min_ss << std::fixed;
      bb_min_ss << std::setprecision(2);
      bb_min_ss << *std::min_element(depth_values.begin(), depth_values.end());
      std::ostringstream bb_max_ss;
      bb_max_ss << std::fixed;
      bb_max_ss << std::setprecision(2);
      bb_max_ss << *std::max_element(depth_values.begin(), depth_values.end());
      cv::putText(depth_w_bb_image,
                  "BB: [" + bb_min_ss.str() + "m, " + bb_max_ss.str() + "m]",
                  cvPoint(10, 60),
                  cv::FONT_HERSHEY_COMPLEX_SMALL,
                  0.8,
                  cvScalar(0, 255, 0),
                  1,
                  CV_AA);
      std::ostringstream bb_fg_min_ss;
      bb_fg_min_ss << std::fixed;
      bb_fg_min_ss << std::setprecision(2);
      bb_fg_min_ss << *std::min_element(foreground_depth_values.begin(),
                                        foreground_depth_values.end());
      std::ostringstream bb_fg_max_ss;
      bb_fg_max_ss << std::fixed;
      bb_fg_max_ss << std::setprecision(2);
      bb_fg_max_ss << *std::max_element(foreground_depth_values.begin(),
                                        foreground_depth_values.end());
      cv::putText(depth_w_bb_image,
                  "BB FG: [" + bb_fg_min_ss.str() + "m, " + bb_fg_max_ss.str() +
                      "m]",
                  cvPoint(10, 80),
                  cv::FONT_HERSHEY_COMPLEX_SMALL,
                  0.8,
                  cvScalar(0, 255, 0),
                  1,
                  CV_AA);
      debug_img_bufs_["depth_w_bb"] = depth_w_bb_image;
      ROS_DEBUG("Successfully created %s-rectified LiDAR depth with bounding "
                "box image.",
                name_.c_str());
    } catch (const std::exception& ex) {
      ROS_ERROR("Failed to create %s-rectified LiDAR depth with bounding box "
                "image: %s",
                name_.c_str(),
                ex.what());
    }

    try {
      depth_w_bb_plus_rgb_image.convertTo(depth_w_bb_plus_rgb_image, CV_8UC3);
      // Add a text overlay to the image
      cv::putText(depth_w_bb_plus_rgb_image,
                  "Observation: " + label,
                  cvPoint(10, 20),
                  cv::FONT_HERSHEY_COMPLEX_SMALL,
                  0.8,
                  cvScalar(0, 255, 0),
                  1,
                  CV_AA);
      std::ostringstream range_ss;
      range_ss << std::fixed;
      range_ss << std::setprecision(2);
      range_ss << range;
      cv::putText(depth_w_bb_plus_rgb_image,
                  "Range: " + range_ss.str() + "m",
                  cvPoint(10, 40),
                  cv::FONT_HERSHEY_COMPLEX_SMALL,
                  0.8,
                  cvScalar(0, 255, 0),
                  1,
                  CV_AA);
      std::ostringstream bb_min_ss;
      bb_min_ss << std::fixed;
      bb_min_ss << std::setprecision(2);
      bb_min_ss << *std::min_element(depth_values.begin(), depth_values.end());
      std::ostringstream bb_max_ss;
      bb_max_ss << std::fixed;
      bb_max_ss << std::setprecision(2);
      bb_max_ss << *std::max_element(depth_values.begin(), depth_values.end());
      cv::putText(depth_w_bb_plus_rgb_image,
                  "BB: [" + bb_min_ss.str() + "m, " + bb_max_ss.str() + "m]",
                  cvPoint(10, 60),
                  cv::FONT_HERSHEY_COMPLEX_SMALL,
                  0.8,
                  cvScalar(0, 255, 0),
                  1,
                  CV_AA);
      std::ostringstream bb_fg_min_ss;
      bb_fg_min_ss << std::fixed;
      bb_fg_min_ss << std::setprecision(2);
      bb_fg_min_ss << *std::min_element(foreground_depth_values.begin(),
                                        foreground_depth_values.end());
      std::ostringstream bb_fg_max_ss;
      bb_fg_max_ss << std::fixed;
      bb_fg_max_ss << std::setprecision(2);
      bb_fg_max_ss << *std::max_element(foreground_depth_values.begin(),
                                        foreground_depth_values.end());
      cv::putText(depth_w_bb_plus_rgb_image,
                  "BB FG: [" + bb_fg_min_ss.str() +
                      "m, : " + bb_fg_max_ss.str() + "m]",
                  cvPoint(10, 80),
                  cv::FONT_HERSHEY_COMPLEX_SMALL,
                  0.8,
                  cvScalar(0, 255, 0),
                  1,
                  CV_AA);
      debug_img_bufs_["depth_w_bb_plus_rgb"] = depth_w_bb_plus_rgb_image;
      ROS_DEBUG("Successfully created %s-rectified LiDAR depth with bounding "
                "box plus RGB image.",
                name_.c_str());
    } catch (const std::exception& ex) {
      ROS_ERROR("Failed to create %s-rectified LiDAR depth with bounding box "
                "plus RGB image: %s",
                name_.c_str(),
                ex.what());
    }
  }

  return true;
}

std::vector<std::string> Camera::getDebugImageNames() const {
  std::vector<std::string> names;
  for (auto const& smap : debug_img_bufs_)
    names.push_back(smap.first);

  return names;
}

cv::Mat Camera::getDebugImage(const std::string& image_name) const {
  return debug_img_bufs_.at(image_name);
}

std::string Camera::getDebugImageEncoding(const std::string& image_name) const {
  return debug_img_encs_.at(image_name);
}

// Calculates the coordinates of the center of a bounding box
cv::Point2d Camera::getBoundingBoxCentroid(const cv::Rect& bbox) const {
  cv::Point2d c;
  c.x = bbox.x + 0.5 * bbox.width;
  c.y = bbox.y + 0.5 * bbox.height;
  return c;
}

darknet_ros_msgs::BoundingBox
Camera::scaleBoundingBoxToModel(const darknet_ros_msgs::BoundingBox& bbox,
                                const cv::Size& image_size,
                                const cv::Size& model_size) const {
  // Scale bbox to match camera resolution
  darknet_ros_msgs::BoundingBox bbox_;
  if ((image_size.width != model_size.width ||
       image_size.height != model_size.height) &&
      image_size.width > 0 && image_size.height > 0) {
    bbox_.Class = bbox.Class;
    bbox_.probability = bbox.probability;
    bbox_.yolo_probability = bbox.yolo_probability;
    bbox_.color_score = bbox.color_score;
    bbox_.xmin = (int)(((float)bbox.xmin / (float)image_size.width) *
                       (float)model_size.width);
    bbox_.xmax = (int)(((float)bbox.xmax / (float)image_size.width) *
                       (float)model_size.width);
    bbox_.ymin = (int)(((float)bbox.ymin / (float)image_size.height) *
                       (float)model_size.height);
    bbox_.ymax = (int)(((float)bbox.ymax / (float)image_size.height) *
                       (float)model_size.height);
  } else {
    bbox_ = bbox;
  }

  return bbox_;
}
