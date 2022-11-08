/**
 * Implementation of KalmanFilter class.
 *
 * @author: Hayk Martirosyan
 * @date: 2014.11.15
 */

#include <iostream>
#include <stdexcept>

#include "artifact_localization/kalman.h"

KalmanFilter::KalmanFilter() {
  initialized = false;

  A << 1, 0, 0, 0, 1, 0, 0, 0, 1;

  B << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  C << 1, 0, 0, 0, 1, 0, 0, 0, 1;

  // Covariance matrices
  // Q << 0.5, 0, 0, 0, 0.5, 0, 0, 0, 0.1;
  // R << 1, 0, 0, 0, 0.5, 0, 0, 0, 0.5;
  Q << 0.5, 0, 0, 0, 0.5, 0, 0, 0, 0.1;
  R << 0.3, 0, 0, 0, 0.3, 0, 0, 0, 0.3;
  P << 1, 0, 0, 0, 1, 0, 0, 0, 1;

  P0 = P;
  I.setIdentity();
}

void KalmanFilter::init(const double& t_start, const Eigen::Vector3d& x0) {
  t0 = t_start;
  t = t0;
  // t_for_recency_check = t0;
  x_hat = x0;
  P = P0;
  initialized = true;
}

void KalmanFilter::update(const double& t_at_pose,
                          const Eigen::Vector3d& z,
                          const Eigen::Vector3d& u) {
  if (!initialized)
    throw std::runtime_error("Filter is not initialized!");
  t = t_at_pose;
  x_hat_new = A * x_hat + B * u;
  P = A * P * A.transpose() + Q;
  K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
  // std::cout << "K is: \n" << K << std::endl;
  x_hat_new += K * (z - C * x_hat_new);
  P = (I - K * C) * P;
  x_hat = x_hat_new;
  x_hat_history.push_back(x_hat);
}