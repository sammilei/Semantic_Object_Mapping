#ifndef CAMERA_H
#define CAMERA_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <numeric>
#include <unordered_map>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <librealsense2/rs.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>

#include <artifact_msgs/ScorabilityMetrics.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/Object.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

class ArtifactLocalizationFixtureTest;

struct size_configuration {
  int height_limits[4];
  int width_limits[4];
  double max_depth_allowed;
};
struct depth_percentile {
  double close_percentile;
  double far_percentile;
};
struct confidence_thresholds {
  double confidence_min;
  double confidence_max;
};

struct CameraConfig {
  double bearing_sigma = 0.2;
  double depth_range_sigma_coeff0 = 0.4;
  double depth_range_sigma_coeff1 = 0.4;
  int depth_min_points = 10;
  double depth_min = 0.1;
  double depth_max = 25.0;

  // Percentiles of closest depth points to camera from which to calculate
  // median
  depth_percentile foreground_depth_percentiles = {0.05, 0.15};
  // Nearest-neighbor distance (m) to median of closest depth points for
  // selecting foreground points
  double foreground_depth_nn_dist_to_median = 0.5;
  // size limits (in cm) gor each artifact (for size score):
  // {min_min_height, min_height, max_height, max_max_height},
  // {min_min_width, min_width, max_width, max_max_width},
  // max_distance
  const std::map<std::string, size_configuration>
      artifact_size_limits_for_size_score = {
          {"Survivor", {{30, 40, 125, 200}, {40, 50, 175, 225}, 17.5}},
          {"Backpack", {{15, 30, 80, 90}, {0, 15, 60, 75}, 12.5}},
          {"Rope", {{5, 15, 75, 85}, {15, 25, 100, 120}, 12.5}},
          {"Helmet", {{0, 10, 30, 50}, {0, 15, 40, 60}, 12.5}},
          {"Drill", {{5, 15, 35, 45}, {5, 10, 30, 40}, 12.5}},
          {"Fire Extinguisher", {{15, 25, 60, 75}, {0, 10, 40, 50}, 12.5}},
          {"Vent", {{30, 40, 125, 200}, {40, 50, 175, 225}, 17.5}},
          {"Cube", {{0, 10, 30, 50}, {0, 15, 40, 60}, 12.5}}};

  // the percentiles of the depth values in a bounding box to keep and do mean
  // for box range chosing 0 and 1 is the normal mean using 0.15 and 0.3 takes
  // the closest 15% to 30% points in the box
  const std::map<std::string, depth_percentile> depth_percentile_ = {
      {"Survivor", {0.15, 0.3}}, {"Backpack", {0.15, 0.3}},
      {"Rope", {0.15, 0.3}},     {"Helmet", {0.15, 0.3}},
      {"Drill", {0.15, 0.3}},    {"Fire Extinguisher", {0.15, 0.3}},
      {"Vent", {0.15, 0.3}},     {"Cube", {0.15, 0.3}}};

  // Per-artifact confidence thresholds
  const std::map<std::string, confidence_thresholds> confidence_thresholds_ = {
      {"Survivor", {0.35, 1.0}}, {"Backpack", {0.325, 1.0}},
      {"Rope", {0.125, 1.0}},     {"Helmet", {0.25, 1.0}},
      {"Drill", {0.125, 1.0}},    {"Fire Extinguisher", {0.225, 1.0}},
      {"Vent", {0.225, 1.0}},     {"Cube", {0.125, 1.0}}};
};

class Camera {
  friend class ArtifactLocalizationFixtureTest;
  FRIEND_TEST(ArtifactLocalizationFixtureTest, doubleTruncatedMedianTest5);
  FRIEND_TEST(ArtifactLocalizationFixtureTest, getRangeFromBoundingBoxTest2);
  FRIEND_TEST(ArtifactLocalizationFixtureTest, getRangeFromBoundingBoxTest3);
  FRIEND_TEST(ArtifactLocalizationFixtureTest, getRangeFromBoundingBoxTest4);
  FRIEND_TEST(ArtifactLocalizationFixtureTest, getRangeFromBoundingBoxTest5);
  FRIEND_TEST(ArtifactLocalizationFixtureTest, getRangeFromBoundingBoxTest6);

public:
  typedef std::shared_ptr<Camera> Ptr;
  typedef std::shared_ptr<Camera const> ConstPtr;

  typedef std::unordered_map<std::string, cv::Mat> MatMap;
  typedef std::unordered_map<std::string, std::string> StrMap;

  Camera(const std::string& name, const CameraConfig& config = CameraConfig());
  ~Camera();

  image_geometry::PinholeCameraModel getModel() const {
    return model_;
  }
  rs2_intrinsics getRSIntrinsics() const;
  void setIntrinsicsFromCameraInfo(const sensor_msgs::CameraInfoConstPtr info);

  Eigen::Affine3d getExtrinsics() const {
    return base_link_T_camera_;
  }
  void setExtrinsics(const Eigen::Affine3d& base_link_T_camera) {
    base_link_T_camera_ = base_link_T_camera;
  }

  std::string getName() const {
    return name_;
  }
  std::string getFrame() const {
    return model_.tfFrame();
  }

  bool isThermal() const {
    // Thermal camera name starts with 'boson'
    return name_.find("boson") == 0;
  }
  bool isRGB() const {
    return !isThermal();
  }

  bool isDetectionValid(const darknet_ros_msgs::ObjectConstPtr& msg) const;

  Eigen::Vector2d project3dToPixel(const Eigen::Vector3d& position) const;
  bool isInFOV(const Eigen::Vector3d& position) const;
  bool isInFOV(const Eigen::Vector2d& pixel) const;
  bool isInBB(const Eigen::Vector3d& position,
              darknet_ros_msgs::BoundingBox bb) const;
  bool isInBB(const Eigen::Vector2d& pixel,
              darknet_ros_msgs::BoundingBox bb) const;

  bool getBearingFromBoundingBox(const cv::Rect& bbox,
                                 Eigen::Vector3d& bearing,
                                 double& bearing_sigma) const;
  bool getBearingFromBoundingBox(const darknet_ros_msgs::BoundingBox& bbox,
                                 Eigen::Vector3d& bearing,
                                 double& bearing_sigma) const {
    return getBearingFromBoundingBox(toCV(bbox), bearing, bearing_sigma);
  }

  void getBboxSize(const double x1,
                   const double x2,
                   const double y1,
                   const double y2,
                   const double range,
                   const rs2_intrinsics camera_params,
                   double& height,
                   double& width);

  void getScoreFromSizeAndLimits(const double size,
                                 const double min,
                                 const double min_min,
                                 const double max,
                                 const double max_max,
                                 double& score);
  void combineHeightWidthScore(const double height_score,
                               const double width_score,
                               double& size_score);
  void getSizeScoreFromArtifactDimensions(const double height,
                                          const double width,
                                          const double range,
                                          const std::string label,
                                          double& size_score);
  bool getSizeScore(const double range,
                    const double range_sigma,
                    const darknet_ros_msgs::BoundingBox& bbox,
                    double& height,
                    double& width,
                    double& size_score);
  bool isBoxOnEdge(const double x1,
                   const double x2,
                   const double y1,
                   const double y2,
                   const rs2_intrinsics camera_params);
  bool doubleTruncatedMean(const std::vector<double> vec,
                           const double close_percentile,
                           const double far_percentile,
                           double& truncated_mean) const;
  bool doubleTruncatedMedian(const std::vector<double> vec,
                             const double close_percentile,
                             const double far_percentile,
                             double& truncated_median) const;
  bool doubleNearestNeighbors(const std::vector<double> vec,
                              const double query,
                              const double radius,
                              std::vector<double>& nearest_neighbors) const;
  bool getRangeFromBoundingBox(const std::string label,
                               const darknet_ros_msgs::BoundingBox& bbox,
                               const sensor_msgs::ImageConstPtr& depth_msg,
                               double& range,
                               double& range_sigma) const;
  bool getRangeFromBoundingBox(const darknet_ros_msgs::BoundingBox& bbox,
                               const sensor_msgs::ImageConstPtr& depth_msg,
                               double& range,
                               double& range_sigma) const {
    return getRangeFromBoundingBox(
        bbox.Class, bbox, depth_msg, range, range_sigma);
  }
  bool
  getRangeFromBoundingBox(const darknet_ros_msgs::BoundingBox& bbox,
                          const sensor_msgs::PointCloud2ConstPtr& lidar_msg,
                          const sensor_msgs::Image& rgb_msg,
                          double& range,
                          double& range_sigma);

  std::vector<std::string> getDebugImageNames() const;
  cv::Mat getDebugImage(const std::string& image_name) const;
  std::string getDebugImageEncoding(const std::string& image_name) const;

private:
  // Debug image buffers
  MatMap debug_img_bufs_;

  // Debug image encodings
  StrMap debug_img_encs_;

  // Initialization helpers
  void initializeBuffers();

  cv::Point2d getBoundingBoxCentroid(const cv::Rect& bbox) const;
  darknet_ros_msgs::BoundingBox
  scaleBoundingBoxToModel(const darknet_ros_msgs::BoundingBox& bbox,
                          const cv::Size& image_size,
                          const cv::Size& model_size) const;
  darknet_ros_msgs::BoundingBox
  scaleBoundingBoxToModel(const darknet_ros_msgs::BoundingBox& bbox,
                          const cv::Size& image_size) const {
    return scaleBoundingBoxToModel(bbox, image_size, model_.fullResolution());
  }
  inline cv::Rect toCV(const darknet_ros_msgs::BoundingBox& bbox) const {
    return cv::Rect(
        bbox.xmin, bbox.ymin, bbox.xmax - bbox.xmin, bbox.ymax - bbox.ymin);
  }
  inline cv::Mat toCV(const sensor_msgs::Image& img) const {
    return cv_bridge::toCvCopy(img)->image;
  }

  std::string name_;
  CameraConfig config_;
  image_geometry::PinholeCameraModel model_;
  Eigen::Affine3d base_link_T_camera_ = Eigen::Affine3d::Identity();
};

#endif
