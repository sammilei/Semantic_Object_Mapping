/**
 * Kalman filter implementation using Eigen. Based on the following
 * introductory paper:
 *
 *     http://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf
 *
 * @author: Hayk Martirosyan
 * @date: 2014.11.15
 */

#include "ros/ros.h"
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#pragma once

class KalmanFilter {
public:
  /**
   * Create a Kalman filter with the specified matrices.
   *   A - System dynamics matrix
   *   C - Output matrix
   *   Q - Process noise covariance
   *   R - Measurement noise covariance
   *   P - Estimate error covariance
   */
  KalmanFilter();

  /**
   * Initialize the filter with a guess for initial states.
   */
  void init(const double& t_start, const Eigen::Vector3d& x0);

  /**
   * Update the estimated state based on measured values. The
   * time step is assumed to remain constant.
   */
  void update(const double& t_at_pose,
              const Eigen::Vector3d& z,
              const Eigen::Vector3d& u);

  /**
   * Return the current state.
   */
  Eigen::Vector3d state() {
    return x_hat;
  };

  double getLatestPoseTime() {
    return t;
  }

  // double getLatestObsTime() {
  //   return t_for_recency_check;
  // }

  double getInitialTime() {
    return t0;
  }

  Eigen::Matrix3d getP() {
    return P;
  };

  bool isInitialized() {
    return initialized;
  }

  void deinitialize() {
    initialized = false;
    x_hat_history.clear();
  }

  std::vector<Eigen::Vector3d> getStateHistory() {
    return x_hat_history;
  }

private:
  // Matrices for computation
  Eigen::Matrix3d A, B, C, Q, R, P, K, P0;

  // Is the filter initialized?
  bool initialized;

  // n-size identity
  Eigen::Matrix3d I;

  // Estimated states
  Eigen::Vector3d x_hat, x_hat_new;
  std::vector<Eigen::Vector3d> x_hat_history;

  // Time of most recent observation
  double t, t0; // t_for_recency_check;
};
