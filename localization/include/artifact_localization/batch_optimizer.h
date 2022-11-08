#pragma once

#include <ros/ros.h>

#include <Eigen/Dense>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

class BatchOptimizerConfig {
public:
  // Noise model for odometry prior
  double odom_position_noise = 0.2;
  double odom_rotation_noise = 0.2;

  // Debug mode
  bool debug = false;
};

/** Localize an artifact from multiple masurements
 *  with batch optimization.
 */
class BatchOptimizer {
public:
  BatchOptimizer();
  ~BatchOptimizer();

  void setConfig(const BatchOptimizerConfig& config) {
    config_ = config;
  }
  BatchOptimizerConfig getConfig() const {
    return config_;
  }

  // Get artifact position in global frame
  // Return NaNs if the estimate is not valid
  Eigen::Vector3d getPosition() const {
    return position_;
  }

  // Get artifact position covariance in global frame
  // Return NaNs if the estimate is not valid
  Eigen::Matrix3d getCovariance() const {
    return covariance_;
  };

  // Return timestamp information
  double getLatestMeasurementTime() {
    return latest_stamp_.toSec();
  }
  double getInitialMeasurementTime() {
    return initial_stamp_.toSec();
  }

  // Add measurement factors to the graph.
  // All measurements should be described in its sensor frame.
  // `sensor_pose` is the pose of the sensor in the global frame.
  void addRangeMeasurement(const double range,
                           const double sigma,
                           const ros::Time& stamp,
                           const Eigen::Affine3d& sensor_pose);
  void addBearingMeasurement(const Eigen::Vector3d& bearing,
                             const double sigma,
                             const ros::Time& stamp,
                             const Eigen::Affine3d& sensor_pose);
  void addBearingRangeMeasurement(const Eigen::Vector3d& bearing,
                                  const double range,
                                  const double bearing_sigma,
                                  const double range_sigma,
                                  const ros::Time& stamp,
                                  const Eigen::Affine3d& sensor_pose);

  // Run graph optimization. A valid result can be obtained
  // only when this function returns true.
  bool optimize();

  bool isPositionValid() {
    return !position_.array().isNaN().any();
  }

  void printGraph() {
    graph_.print();
  }

private:
  void updateTimeStamp(const ros::Time& stamp);
  gtsam::Key addOdometryFactor(const ros::Time& stamp,
                               const Eigen::Affine3d& pose);
  Eigen::Vector3d makeInitialGuess();

  BatchOptimizerConfig config_;

  ros::Time initial_stamp_, latest_stamp_;

  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_values_;
  gtsam::Values result_values_;

  // Key of artifact node. Must be unique
  const gtsam::Key artifact_key_ = 0;

  Eigen::Vector3d position_;
  Eigen::Matrix3d covariance_;

  gtsam::noiseModel::Diagonal::shared_ptr odom_noise_;

  bool has_new_measurements_ = false;
};
