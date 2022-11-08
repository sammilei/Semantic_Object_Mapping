#include <iostream>
#include <stdexcept>

#include "artifact_localization/msl_raptor_wrapper.h"

MSLRaptorWrapper::MSLRaptorWrapper() {
  initialized_ = false;
}

void MSLRaptorWrapper::init(const double& t_start,
                            const float width,
                            const float height,
                            const float depth,
                            std::vector<double>& init_cov_diag,
                            std::vector<double>& process_noise_diag,
                            std::vector<double>& meas_noise_diag,
                            rs2_intrinsics& intrinsics,
                            const Eigen::Affine3d& extrinsics,
                            const darknet_ros_msgs::BoundingBox& box_for_ini) {
  t0_ = t_start;
  t_ = t0_;
  // t_for_recency_check = t0;

  cv::Mat dist_params;

  if (intrinsics.model != RS2_DISTORTION_NONE) {
    dist_params = cv::Mat(1, 5, CV_32F, intrinsics.coeffs);
  }

  msl_raptor_backend::CameraParams cam_params(intrinsics.ppx,
                                              intrinsics.ppy,
                                              intrinsics.fx,
                                              intrinsics.fy,
                                              extrinsics.cast<float>(),
                                              dist_params);
  msl_raptor_backend::ObjectParams obj_params(
      width, height, depth, init_cov_diag, process_noise_diag, meas_noise_diag);
  MSLRaptorUKF<true>::MeasureVec m;
  m << (box_for_ini.xmax + box_for_ini.xmin) / 2,
      (box_for_ini.ymax + box_for_ini.ymin) / 2,
      box_for_ini.xmax - box_for_ini.xmin, box_for_ini.ymax - box_for_ini.ymin;

  msl_raptor_ukf_ = MSLRaptorUKF<true>(obj_params, cam_params, m);
  initialized_ = true;
}

void MSLRaptorWrapper::updateCamera(rs2_intrinsics& intrinsics,
                                    const Eigen::Affine3d& extrinsics_mat,
                                    const std::string& cam_name) {
  cv::Mat dist_params;

  if (intrinsics.model != RS2_DISTORTION_NONE) {
    dist_params = cv::Mat(1, 5, CV_32F, intrinsics.coeffs);
  }

  msl_raptor_backend::CameraParams cam_params(intrinsics.ppx,
                                              intrinsics.ppy,
                                              intrinsics.fx,
                                              intrinsics.fy,
                                              extrinsics_mat.cast<float>(),
                                              dist_params);

  msl_raptor_ukf_.updateCamera(cam_params);
  last_camera_used_ = cam_name;
}

void MSLRaptorWrapper::update(const double& t_at_pose,
                              const darknet_ros_msgs::BoundingBox& z,
                              const Eigen::Affine3d& u) {
  MSLRaptorUKF<true>::MeasureVec m;
  m << (z.xmax + z.xmin) / 2, (z.ymax + z.ymin) / 2, z.xmax - z.xmin,
      z.ymax - z.ymin;

  msl_raptor_ukf_.update(t_at_pose - t_, m, u);
  t_ = t_at_pose;
}
