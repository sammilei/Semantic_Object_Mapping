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
#include <artifact_localization/msl_raptor_backend.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/Object.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <opencv2/imgproc.hpp>

#pragma once

class MSLRaptorWrapper {
public:
  MSLRaptorWrapper();
  /**
   * Initialize the filter with a guess for initial states.
   */
  void init(const double& t_start,
            const float width,
            const float height,
            const float depth,
            std::vector<double>& init_cov_diag,
            std::vector<double>& process_noise_diag,
            std::vector<double>& meas_noise_diag,
            rs2_intrinsics& intrinsics,
            const Eigen::Affine3d& extrinsics,
            const darknet_ros_msgs::BoundingBox& box_for_ini);

  /**
   * Update the estimated state based on measured values. The
   * time step is assumed to remain constant.
   */
  void update(const double& t_at_pose,
              const darknet_ros_msgs::BoundingBox& z,
              const Eigen::Affine3d& u);

  /**
   * Return the current state.
   */
  Eigen::Vector3d state() {
    return msl_raptor_ukf_.getState().head<3>();
  };

  double getLatestPoseTime() {
    return t_;
  }

  // double getLatestObsTime() {
  //   return t_for_recency_check;
  // }

  double getInitialTime() {
    return t0_;
  }

  std::string getLastCamUsedName() {
    return last_camera_used_;
  }

  void updateCamera(rs2_intrinsics& intrinsics,
                    const Eigen::Affine3d& extrinsics_mat,
                    const std::string& cam_name);

  Eigen::Matrix3d getP() {
    // TODO: Implement correctly
    return Eigen::Matrix3d::Identity() * 0.5;
  };

  bool isInitialized() {
    return initialized_;
  }

  // void deinitialize()
  // {
  //   initialized = false;
  //   x_hat_history.clear();
  // }

  // std::vector<Eigen::Vector3d> getStateHistory()
  // {
  //   return x_hat_history;
  // }

private:
  // Is the filter initialized?
  bool initialized_;
  std::string last_camera_used_;

  // Time of most recent observation
  double t_, t0_; // t_for_recency_check;

  MSLRaptorUKF<true> msl_raptor_ukf_;
};
