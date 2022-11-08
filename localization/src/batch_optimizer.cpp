#include "artifact_localization/batch_optimizer.h"

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/sam/BearingFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/sam/RangeFactor.h>
#include <gtsam/slam/PriorFactor.h>

using namespace gtsam;

BatchOptimizer::BatchOptimizer()
{
  // Initially, the position/covariance estimations are NaNs
  position_.fill(NAN);
  covariance_.fill(NAN);
}

BatchOptimizer::~BatchOptimizer() {}

void BatchOptimizer::addRangeMeasurement(const double range,
                                         const double sigma,
                                         const ros::Time& stamp,
                                         const Eigen::Affine3d& sensor_pose) {
  // Update common information
  updateTimeStamp(stamp);
  Key odom_key = addOdometryFactor(stamp, sensor_pose);
  has_new_measurements_ = true;

  // Add measurement
  auto meas_noise = noiseModel::Diagonal::Sigmas(Vector1(sigma));
  auto factor =
      RangeFactor<Pose3, Point3>(odom_key, artifact_key_, range, meas_noise);
  graph_.add(factor);
}

void BatchOptimizer::addBearingMeasurement(const Eigen::Vector3d& bearing,
                                           const double sigma,
                                           const ros::Time& stamp,
                                           const Eigen::Affine3d& sensor_pose) {
  // Update common information
  updateTimeStamp(stamp);
  Key odom_key = addOdometryFactor(stamp, sensor_pose);
  has_new_measurements_ = true;

  // Add measurement
  auto meas_noise = noiseModel::Diagonal::Sigmas(Vector2(sigma, sigma));
  auto factor = BearingFactor<Pose3, Point3>(
      odom_key, artifact_key_, Pose3().bearing(bearing), meas_noise);
  graph_.add(factor);
}

void BatchOptimizer::addBearingRangeMeasurement(
    const Eigen::Vector3d& bearing,
    const double range,
    const double bearing_sigma,
    const double range_sigma,
    const ros::Time& stamp,
    const Eigen::Affine3d& sensor_pose) {
  // Update common information
  updateTimeStamp(stamp);
  Key odom_key = addOdometryFactor(stamp, sensor_pose);
  has_new_measurements_ = true;

  // Add measurement
  auto meas_noise = noiseModel::Diagonal::Sigmas(
      Vector3(bearing_sigma, bearing_sigma, range_sigma));
  auto factor = BearingRangeFactor<Pose3, Point3>(
      odom_key, artifact_key_, Pose3().bearing(bearing), range, meas_noise);
  graph_.add(factor);
}

bool BatchOptimizer::optimize() {
  // Do not work if not necessary
  if (graph_.empty()) {
    return false;
  }

  if (!has_new_measurements_) {
    return isPositionValid() ? true : false;
  }
  has_new_measurements_ = false;

  // Initial guess for the artifact position
  Point3 guess = makeInitialGuess();
  if (initial_values_.exists(artifact_key_)) {
    initial_values_.update(artifact_key_, guess);
  } else {
    initial_values_.insert(artifact_key_, guess);
  }

  // Run optimization
  // TODO: Use robust optimization
  ROS_DEBUG("Optimizing a graph with %lu factors", graph_.size());

  if (config_.debug) {
    graph_.print();
    initial_values_.print();
  }

  result_values_ =
      LevenbergMarquardtOptimizer(graph_, initial_values_).optimize();

  if (config_.debug) {
    result_values_.print();
  }

  // Extract artifact position and covariance. In degenerated case,
  // they are set to NAN and the function returns false
  position_ = result_values_.at<Point3>(artifact_key_);

  try {
    Marginals marginals(graph_, result_values_);
    covariance_ = marginals.marginalCovariance(artifact_key_);
  } catch (IndeterminantLinearSystemException) {
    ROS_DEBUG("Failed to compute covariance (degenerated case)");
    covariance_.fill(NAN);
    position_.fill(NAN);
    return false;
  }

  return true;
}

void BatchOptimizer::updateTimeStamp(const ros::Time& stamp) {
  if (!initial_stamp_.isValid()) {
    initial_stamp_ = stamp;
  }
  latest_stamp_ = std::max<ros::Time>(latest_stamp_, stamp);
}

gtsam::Key BatchOptimizer::addOdometryFactor(const ros::Time& stamp,
                                             const Eigen::Affine3d& pose) {
  // Initialize odometry noise if not exist
  if (!odom_noise_) {
    Vector sigmas(6);
    sigmas.head(3).fill(config_.odom_position_noise);
    sigmas.tail(3).fill(config_.odom_rotation_noise);
    odom_noise_ = noiseModel::Diagonal::Sigmas(sigmas);
  }

  // Add odometry factor
  Key odom_key = stamp.toNSec();
  graph_.add(PriorFactor<Pose3>(odom_key, Pose3(pose.matrix()), odom_noise_));
  if (!initial_values_.exists(odom_key)) {
    initial_values_.insert(odom_key, Pose3(pose.matrix()));
  }
  return odom_key;
}

Eigen::Vector3d BatchOptimizer::makeInitialGuess() {
  // Use the last artifact position if available
  // Otherwise, use the origin (maybe better alternatives?)
  // z-offset is added to avoid optimization failure (why?)
  return isPositionValid() ? position_ : Eigen::Vector3d(0, 0, 10);
}