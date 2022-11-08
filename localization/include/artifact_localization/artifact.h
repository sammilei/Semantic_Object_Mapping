#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <math.h>
#include <numeric>

#include <ros/ros.h>

#include <artifact_msgs/Artifact.h>
#include <artifact_msgs/PointSourceDetection.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/Object.h>

#include "artifact_localization/batch_optimizer.h"
#include "artifact_localization/observation.h"

class ArtifactLocalizationFixtureTest;

struct ArtifactConfig {
  // Names
  std::string name;
  std::string abbreviation;
  std::string robot_name;

  // Frames
  std::string map_frame;
  std::string base_link_frame;

  // Confidence thresholds
  double confidence_min = 0.35; // this needs a change if in camera.h  double
                                // confidence_min = 0.35 changes
  double confidence_max = 0.95;

  // Uncertainty thresholds
  double sigma_min = 0.5;
  double sigma_max = 25.0;

  // Publication conditions
  double stale_timeout = 100; // [s]
  int min_num_observations = 2;

  // Publication conditions when debugging (comment the above)
  // double confidence_min = 0.5;
  // double confidence_max = 0.95;
  // double sigma_min = 1;
  // double sigma_max = 25.0;
  // double stale_timeout = 10; // [s]
  // double min_num_observations = 2;

  // Geometry assumptions
  double range_min = 0.2;
  double range_max = 25.0;
  double height_sigma = 4.0;

  // Reconciliation parameters
  double reconcile_range = 15.0;
  double reconcile_timeout = 300.0;
  double reconcile_angle_tolerance = M_PI / 8;
  double reconcile_height_tolerance = 3.0;

  // Scorability parameters
  int scorability_min_num_observations = 6; // if less observation, penalty
  double remove_tail = 0.2; // % of best and worst scores from color/size/yolo
                            // confidence list to ignore
};

class Artifact {
  friend class ArtifactLocalizationFixtureTest;
  FRIEND_TEST(ArtifactLocalizationFixtureTest, cubeBTPublishArtifactClbkTest1);
  FRIEND_TEST(ArtifactLocalizationFixtureTest,
              cubeVisionPublishArtifactClbkTest1);

public:
  typedef std::shared_ptr<Artifact> Ptr;
  typedef std::shared_ptr<Artifact const> ConstPtr;

  Artifact(const ArtifactConfig& config,
           const std::string& id_hint = std::string());
  ~Artifact();

  std::string getLabel() const {
    return data_.label;
  }
  std::string getId() const {
    return data_.id;
  }
  ros::Time getFirstObservationTime() const {
    return first_observation_time_;
  }
  ros::Time getBestObservationTime() const {
    return best_observation_time_;
  }
  ros::Time getLastObservationTime() const {
    return last_observation_time_;
  }
  double getConfidence() const {
    return data_.confidence;
  }
  Eigen::Vector3d getPositionInMap() const {
    return position_;
  }
  Eigen::Matrix3d getCovarianceInMap() const {
    return covariance_;
  }
  Eigen::Vector3d getSigmas() const {
    if (!isPositionValid()) {
      return Eigen::Vector3d::Constant(config_.sigma_max);
    }
    return covariance_.eigenvalues().real().array().sqrt();
  }
  double getLargestSigma() const {
    return getSigmas().maxCoeff();
  }
  int getNumObservations() const {
    return std::max(num_bearing_, num_signal_);
  }

  std::string getSummaryString() {
    std::ostringstream ss;
    ss << "<Artifact "
       << "type='" << getLabel() << "' "
       << "id='" << getId() << "' "
       << "confidence='" << int(getConfidence() * 100.0) << "' "
       << "count='" << getNumObservations() << "' "
       << "sigma='" << getLargestSigma() << "' "
       << ">";
    return ss.str();
  }

  bool isThumbnailValid() {
    return bool(thumbnail_);
  }

  void setPublished() {
    published_latest_data_ = true;
  }
  bool isPublished() const {
    return published_latest_data_;
  }

  bool isPositionValid() const {
    return !position_.array().isNaN().any() &&
        !covariance_.array().isNaN().any();
  }

  bool minimalObservationsMade();
  bool enoughObservationsMade();

  artifact_msgs::ArtifactPtr toMessageInMap();
  artifact_msgs::ArtifactPtr toMessageInBaseLink();
  sensor_msgs::CompressedImagePtr toImageMessage();
  sensor_msgs::ImagePtr toAnnotatedImageMessage();

  // scorability
  void reconcileAllScores(std::vector<double> const& color_scores,
                          std::vector<double> const& yolo_scores,
                          std::vector<double> const& size_scores,
                          double& scorability_color_size_conf,
                          double& final_yolo_conf,
                          double& final_color_conf,
                          double& final_size_conf);
  bool updateScorability(artifact_msgs::ArtifactPtr& one_artifact);
  bool createMosaique();
  void removeInvalidValue(std::vector<double>& vec, const double invalid_value);
  bool getTruncatedMeanFromVector(const std::vector<double> vec,
                                  double& truncated_mean,
                                  const double remove_tail_default);
  void combineAllScores(const double final_yolo_score,
                        const double final_color_score,
                        const double final_size_score,
                        double& scorability);

  // Check if a new observation belongs to the same object.
  // Returns value from 0 (different object) to 1 (definitely the same object)
  double getReconciliationScore(const Observation& obs);

  // compute image propeties
  float calcBlurriness(const cv::Mat& src);
  void updateBoundingBoxLocation(const Observation& obs, const int idx);
  bool updateThumbnails(const Observation& obs);
  cv_bridge::CvImagePtr concatenateThumbnails();

  // Add observation
  bool addBearingObservation(const Observation& obs);
  bool addRangeObservation(const Observation& obs);
  bool addImageObservation(const Observation& obs);
  bool addSignalObservation(const Observation& obs);

protected:
  // Check if an observation belongs to the same object
  double getCellPhoneReconciliationScore(const Observation& obs);
  double getCubeSignalReconciliationScore(
      const Observation& obs); // seperate reconciliation from signal and visual
  double getGasReconciliationScore(const Observation& obs);
  double getVisualReconciliationScore(const Observation& obs);

  // Assign unique ID based on properties
  std::string generateUniqueId();

  // Update information
  void updateConfidence(const Observation& obs);
  void updateObservationInfo(const ros::Time& stamp,
                             const Eigen::Affine3d& map_T_base_link);
  bool updatePositionEstimate();
  void updateObservationsize(const double height,
                             const double width,
                             const double size_score);

  // Transform utils
  Eigen::Vector3d transformPosition(const Eigen::Affine3d& map_T_base_link,
                                    const Eigen::Vector3d& position_in_map) {
    return map_T_base_link.inverse() * position_in_map;
  }
  Eigen::Matrix3d
  transformCovariance(const Eigen::Affine3d& map_T_base_link,
                      const Eigen::Matrix3d& covariance_in_map) {
    return map_T_base_link.linear().inverse() * covariance_in_map *
        map_T_base_link.linear();
  }

  // Configuration
  ArtifactConfig config_;

  // Optimizer object
  BatchOptimizer optimizer_;

  // Artifact position
  Eigen::Vector3d position_ = Eigen::Vector3d::Constant(NAN);
  Eigen::Matrix3d covariance_ = Eigen::Matrix3d::Constant(NAN);

  // Artifact metadata
  artifact_msgs::Artifact data_;
  sensor_msgs::CompressedImageConstPtr thumbnail_;
  sensor_msgs::CompressedImageConstPtr thumbnail_closest_;
  sensor_msgs::CompressedImageConstPtr thumbnail_last_;
  sensor_msgs::CompressedImageConstPtr thumbnail_brightest_;
  sensor_msgs::CompressedImageConstPtr thumbnail_best_;
  sensor_msgs::CompressedImageConstPtr thumbnail_less_blurry_;

  double thumbnail_closest_distance_ = 9999999;
  double thumbnail_brightest_value_ = 0;
  double thumbnail_best_score_ = 0;
  float least_bluriness_ = 99999;
  bool use_thumbnail_close_ = false;

  // Artifact count
  int num_bearing_ = 0;
  int num_signal_ = 0;
  static int num_total_artifacts_;

  // Key observations
  ros::Time first_observation_time_ = ros::Time(0);
  ros::Time last_observation_time_ = ros::Time(0);
  ros::Time best_observation_time_ = ros::Time(0);

  Eigen::Affine3d first_map_T_base_link_;
  Eigen::Affine3d last_map_T_base_link_;
  Eigen::Affine3d best_map_T_base_link_;

  // Status flags
  bool published_latest_data_ = false;
};
