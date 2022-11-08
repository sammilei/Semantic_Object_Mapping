/*
Copyright 2021 The Board Of Trustees Of The Leland Stanford Junior University and Polyvalor, Limited Partnership, All Rights Reserved

This software is available under the GNU General Public License v3 (GPL v3):https://www.gnu.org/licenses/gpl-3.0.txt

For commercial licensing, please contact the Stanford Office of Technology Licensing: info@otlmail.stanford.edu
*/
#pragma once

#include "UKF.h"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <numeric>
#include <opencv2/core/eigen.hpp>
#include <vector>

#define COS_45 0.52532198881

namespace msl_raptor_backend {
struct CameraParams {
  cv::Mat dist_coeffs;
  cv::Mat K;
  Eigen::Matrix3d K_inv;
  cv::Mat rvec;
  cv::Mat tvec;
  Eigen::Affine3d tf_ego_cam;
  Eigen::Affine3f tf_cam_ego;

  void init(const float ppx,
            const float ppy,
            const float fx,
            const float fy,
            const Eigen::Affine3f& extrinsics_in,
            const cv::Mat& dist_coeffs_in) {
    cv::Mat cam_matrix = cv::Mat::eye(3, 3, CV_32F);
    cam_matrix.at<float>(0, 0) = fx;
    cam_matrix.at<float>(1, 1) = fy;
    cam_matrix.at<float>(0, 2) = ppx;
    cam_matrix.at<float>(1, 2) = ppy;
    init(cam_matrix, extrinsics_in, dist_coeffs_in);
  }

  void init(const std::vector<float>& camera_matrix_in,
            const std::vector<float>& rvec_in,
            const std::vector<float>& tvec_in,
            const std::vector<float>& dist_coeffs_in) {
    cv::Mat camera_matrix = cv::Mat(camera_matrix_in, true).reshape(1, 3);
    cv::Mat rvec = cv::Mat(rvec_in, true);
    cv::Mat tvec = cv::Mat(tvec_in, true);
    cv::Mat dist_coeffs = cv::Mat(dist_coeffs_in, true);
    init(camera_matrix, rvec, tvec, dist_coeffs);
  }

  void init(const cv::Mat& camera_matrix_in,
            const Eigen::Affine3f& extrinsics,
            const cv::Mat& dist_coeffs_in) {
    cv::Mat rvec(1, 3, CV_32F);
    cv::Mat tvec(1, 3, CV_32F);
    cv::Mat rmat(3, 3, CV_32F);

    Eigen::MatrixXf rmat_eigen = extrinsics.matrix().topLeftCorner<3, 3>();
    Eigen::MatrixXf tvec_eigen = extrinsics.matrix().topRightCorner<3, 1>();

    cv::eigen2cv(rmat_eigen, rmat);
    cv::eigen2cv(tvec_eigen, tvec);

    cv::Rodrigues(rmat, rvec);

    init(camera_matrix_in, rvec, tvec, dist_coeffs_in);
  }

  void init(const cv::Mat& camera_matrix_in,
            const cv::Mat& rvec_in,
            const cv::Mat& tvec_in,
            const cv::Mat& dist_coeffs_in) {
    K = camera_matrix_in.clone();
    Eigen::Matrix3f K_eigen;
    cv::cv2eigen(K, K_eigen);
    K_inv = K_eigen.cast<double>().inverse();
    rvec = rvec_in.clone();
    tvec = tvec_in.clone();

    cv::Mat rot_mat;
    cv::Rodrigues(rvec, rot_mat);
    Eigen::Matrix3f rot_mat_eigen;
    cv::cv2eigen(rot_mat, rot_mat_eigen);
    // ;
    tf_ego_cam = Eigen::Translation3d(double(tvec.at<float>(0)),
                                      double(tvec.at<float>(1)),
                                      double(tvec.at<float>(2))) *
        Eigen::AngleAxisd(rot_mat_eigen.cast<double>());
    tf_cam_ego = tf_ego_cam.inverse().cast<float>();
    dist_coeffs = dist_coeffs_in.clone();
  }

  void updateIntrinsics(const float ppx,
                        const float ppy,
                        const float fx,
                        const float fy,
                        const cv::Mat& dist_coeffs_in) {
    cv::Mat cam_matrix = cv::Mat::eye(3, 3, CV_32F);
    cam_matrix.at<float>(0, 0) = fx;
    cam_matrix.at<float>(1, 1) = fy;
    cam_matrix.at<float>(0, 2) = ppx;
    cam_matrix.at<float>(1, 2) = ppy;
    updateIntrinsics(cam_matrix, dist_coeffs);
  }

  void updateIntrinsics(const std::vector<float>& camera_matrix_in,
                        const std::vector<float>& dist_coeffs_in) {
    cv::Mat camera_matrix = cv::Mat(camera_matrix_in, true).reshape(1, 3);
    cv::Mat dist_coeffs = cv::Mat(dist_coeffs_in, true);
    updateIntrinsics(camera_matrix, dist_coeffs);
  }

  void updateIntrinsics(const cv::Mat& camera_matrix_in,
                        const cv::Mat& dist_coeffs_in) {
    K = camera_matrix_in.clone();
    Eigen::Matrix3f K_eigen;
    cv::cv2eigen(K, K_eigen);
    K_inv = K_eigen.cast<double>().inverse();
    dist_coeffs = dist_coeffs_in.clone();
  }

  void updateExtrinsics(const Eigen::Affine3d& extrinsics) {
    cv::Mat rmat(3, 3, CV_32F);
    Eigen::MatrixXf rmat_eigen =
        extrinsics.matrix().topLeftCorner<3, 3>().cast<float>();
    Eigen::MatrixXf tvec_eigen =
        extrinsics.matrix().topRightCorner<3, 1>().cast<float>();

    cv::eigen2cv(rmat_eigen, rmat);
    cv::eigen2cv(tvec_eigen, tvec);

    cv::Rodrigues(rmat, rvec);
    tf_ego_cam = extrinsics;
    tf_cam_ego = tf_ego_cam.inverse().cast<float>();
  }

  void updateExtrinsics(const cv::Mat& rvec_in, const cv::Mat& tvec_in) {
    cv::Mat rot_mat;
    rvec = rvec_in;
    tvec = tvec_in;
    cv::Rodrigues(rvec, rot_mat);
    Eigen::Matrix3f rot_mat_eigen;
    cv::cv2eigen(rot_mat, rot_mat_eigen);
    // ;
    tf_ego_cam = Eigen::Translation3d(double(tvec.at<float>(0)),
                                      double(tvec.at<float>(1)),
                                      double(tvec.at<float>(2))) *
        Eigen::AngleAxisd(rot_mat_eigen.cast<double>());
    tf_cam_ego = tf_ego_cam.inverse().cast<float>();
  }

  void updateExtrinsics(const std::vector<float>& rvec_in,
                        const std::vector<float>& tvec_in) {
    cv::Mat rvec = cv::Mat(rvec_in, true);
    cv::Mat tvec = cv::Mat(tvec_in, true);
    updateExtrinsics(rvec, tvec);
  }

  CameraParams(){};

  CameraParams(const float ppx,
               const float ppy,
               const float fx,
               const float fy,
               const Eigen::Affine3f& extrinsics,
               const cv::Mat& dist_coeffs_in) {
    init(ppx, ppy, fx, fy, extrinsics, dist_coeffs_in);
  }

  CameraParams(const cv::Mat& camera_matrix_in,
               const Eigen::Affine3f& extrinsics,
               const cv::Mat& dist_coeffs_in) {
    init(camera_matrix_in, extrinsics, dist_coeffs_in);
  }

  CameraParams(const std::vector<float>& camera_matrix_in,
               const std::vector<float>& rvec_in,
               const std::vector<float>& tvec_in,
               const std::vector<float>& dist_coeffs_in) {
    init(camera_matrix_in, rvec_in, tvec_in, dist_coeffs_in);
  }
};

struct ObjectParams {
  Eigen::MatrixXf model_points_ado;
  Eigen::MatrixXf model_points_ado_aug_t;

  float width;
  float height;
  float length;
  float aspect_ratio;

  Eigen::VectorXd state_cov_diags;
  Eigen::VectorXd process_noise_cov_diags;
  Eigen::VectorXd measure_noise_cov_diags;

  ObjectParams(){};

  ObjectParams(const Eigen::MatrixXf& model_points_ado_in,
               std::vector<double>& state_cov_diags_in,
               std::vector<double>& process_noise_cov_diags_in,
               std::vector<double>& measure_noise_cov_diags_in) {
    init(model_points_ado_in,
         state_cov_diags_in,
         process_noise_cov_diags_in,
         measure_noise_cov_diags_in);
  }
  ObjectParams(float width,
               float height,
               float length,
               std::vector<double>& state_cov_diags_in,
               std::vector<double>& process_noise_cov_diags_in,
               std::vector<double>& measure_noise_cov_diags_in) {
    float half_height = height / 2;
    float half_width = width / 2;
    float half_length = length / 2;
    Eigen::Matrix<float, 8, 3> model_points;
    model_points << half_width, half_height, half_length, //
        half_width, half_height, -half_length,            //
        half_width, -half_height, -half_length,           //
        half_width, -half_height, half_length,            //
        -half_width, -half_height, half_length,           //
        -half_width, -half_height, -half_length,          //
        -half_width, half_height, -half_length,           //
        -half_width, half_height, half_length;            //

    init(model_points,
         state_cov_diags_in,
         process_noise_cov_diags_in,
         measure_noise_cov_diags_in);
  }

  void init(const Eigen::MatrixXf& model_points_ado_in,
            std::vector<double>& state_cov_diags_in,
            std::vector<double>& process_noise_cov_diags_in,
            std::vector<double>& measure_noise_cov_diags_in) {
    model_points_ado = model_points_ado_in;

    width =
        model_points_ado.col(0).maxCoeff() - model_points_ado.col(0).minCoeff();
    height =
        model_points_ado.col(1).maxCoeff() - model_points_ado.col(1).minCoeff();
    length =
        model_points_ado.col(2).maxCoeff() - model_points_ado.col(2).minCoeff();

    aspect_ratio = width / height;
    model_points_ado_aug_t = model_points_ado;
    model_points_ado_aug_t.conservativeResize(
        model_points_ado_aug_t.rows(), model_points_ado_aug_t.cols() + 1);
    model_points_ado_aug_t.col(model_points_ado_aug_t.cols() - 1) =
        Eigen::VectorXf::Ones(model_points_ado_aug_t.rows());
    model_points_ado_aug_t.transposeInPlace();

    state_cov_diags = Eigen::VectorXd::Map(state_cov_diags_in.data(),
                                           state_cov_diags_in.size());
    process_noise_cov_diags = Eigen::VectorXd::Map(
        process_noise_cov_diags_in.data(), process_noise_cov_diags_in.size());
    measure_noise_cov_diags = Eigen::VectorXd::Map(
        measure_noise_cov_diags_in.data(), measure_noise_cov_diags_in.size());
  }
};
} // namespace msl_raptor_backend

template <bool aligned_bb>
class MSLRaptorUKF {
public:
  /** define state vector: <scalars, 3-vectors, quaternions> */
  typedef kalman::Vector<0, 3, 1> StateVec; // the layout is: (pos x 3, vel x 3,
                                            // angularvel x 3, attitude x 4)
  typedef Eigen::Affine3d InputType;
  /** define measurement vector <scalars, 3-vectors, quaternions> */
  typedef typename std::conditional<
      /*    */ aligned_bb,
      /* y? */ kalman::Vector<4, 0, 0>,
      /* n? */ kalman::Vector<5, 0, 0>>::type MeasureVec;

  static void init(kalman::UKF<MSLRaptorUKF, InputType>& ukf,
                   MSLRaptorUKF msl_raptor_ukf) {
    ukf.stateRootCov = msl_raptor_ukf.obj_params_.state_cov_diags.asDiagonal();
    ukf.stateRootCov =
        ukf.stateRootCov.llt()
            .matrixU(); // note that we are using sqrt of actual cov matrix

    ukf.measureNoiseRootCov =
        msl_raptor_ukf.obj_params_.measure_noise_cov_diags.asDiagonal();
    ukf.measureNoiseRootCov = ukf.measureNoiseRootCov.llt().matrixU();

    ukf.processNoiseRootCov =
        msl_raptor_ukf.obj_params_.process_noise_cov_diags.asDiagonal();
    ukf.processNoiseRootCov =
        ukf.processNoiseRootCov.llt()
            .matrixU(); // note that we are using sqrt of actual cov matrix

    ukf.ukfModel = msl_raptor_ukf;
  }

  static StateVec dF(const StateVec& state, const InputType& u) {
    StateVec out = state;

    /* differentiate the quaternions automatically.
     * Second argument specifies start of angular velocity params in state
     * vector */
    kalman::util::diffQuaternion(out, 6);

    /* differentiate position automatically.
     * arguments are: (output state vector, start of position param, end of
     * position param, beginning of velocity param)  */
    kalman::util::diffPosition(out, 0, 3, 3);
    return out;
  }

  // Used if bounding box is angled
  template <bool is_aligned_bb = aligned_bb>
  StateVec approxStateFromBb(
      typename std::enable_if<!is_aligned_bb, const MeasureVec&>::type bb) {
    std::vector<double> width_height;

    width_height = (obj_params_.width > obj_params_.height) ?
        ((bb(2) > bb(3)) ? std::vector<double>{bb(2), bb(3)} :
                           std::vector<double>{bb(3), bb(2)}) :
        ((bb(2) > bb(3)) ? std::vector<double>{bb(3), bb(2)} :
                           std::vector<double>{bb(2), bb(3)});

    double d;
    Eigen::Vector3d bb_center;
    bb_center << bb.segment(0, 2), 1;
    // Get a guess approximate size of object visible in the image. Assume that
    // the object is at the same height as camera. Average between size of the
    // smallest object side and the object oriented 45 deg.
    float expected_average_width = 0.5 *
        (COS_45 * (obj_params_.width + obj_params_.length) +
         std::min(obj_params_.width, obj_params_.length));
    d = cam_params_.K.at<float>(0, 0) * expected_average_width /
        width_height[0];
    // Compensate for object center point and not closest face
    d += obj_params_.length / 2;
    Eigen::Vector3d pos = cam_params_.K_inv * d * bb_center;
    Eigen::Quaterniond quat;
    quat =
        Eigen::Quaterniond(Eigen::AngleAxisd(bb[4], Eigen::Vector3d::UnitZ()));

    pos = cam_params_.tf_ego_cam * pos;

    StateVec s;
    s << pos, 0, 0, 0, quat.coeffs(), 0, 0, 0;
    return s;
  }

  // Used if bounding box is aligned
  template <bool is_aligned_bb = aligned_bb>
  StateVec approxStateFromBb(
      typename std::enable_if<is_aligned_bb, const MeasureVec&>::type bb) {
    std::vector<double> width_height;

    width_height = (obj_params_.width > obj_params_.height) ?
        ((bb(2) > bb(3)) ? std::vector<double>{bb(2), bb(3)} :
                           std::vector<double>{bb(3), bb(2)}) :
        ((bb(2) > bb(3)) ? std::vector<double>{bb(3), bb(2)} :
                           std::vector<double>{bb(2), bb(3)});

    double d;
    Eigen::Vector3d bb_center;
    bb_center << bb.segment(0, 2), 1;
    // Get a guess approximate size of object visible in the image. Assume that
    // the object is at the same height as camera. Average between size of the
    // smallest object side and the object oriented 45 deg.
    float expected_average_width = 0.5 *
        (COS_45 * (obj_params_.width + obj_params_.length) +
         std::min(obj_params_.width, obj_params_.length));
    d = cam_params_.K.at<float>(0, 0) * expected_average_width /
        width_height[0];
    // Compensate for object center point and not closest face
    d += obj_params_.length / 2;
    Eigen::Vector3d pos = cam_params_.K_inv * d * bb_center;

    pos = cam_params_.tf_ego_cam * pos;

    StateVec s;
    s << pos, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    s.quat(0).setIdentity();
    return s;
  }

  static void state_to_tf(const StateVec& state, Eigen::Affine3f& tf) {
    tf = Eigen::Affine3f::Identity() *
        Eigen::Translation3f(Eigen::Vector3f(state(0), state(1), state(2))) *
        Eigen::Quaternionf(state.quat(0).w(),
                           state.quat(0).x(),
                           state.quat(0).y(),
                           state.quat(0).z());
  }

  // Used if bounding box is angled
  template <bool is_aligned_bb = aligned_bb>
  MeasureVec convertRotRectToMeasureVec(
      typename std::enable_if<!is_aligned_bb, const cv::RotatedRect&>::type
          rect) {
    MeasureVec out;
    float angle, ar_meas;
    ar_meas = rect.size.width / rect.size.height;
    // Transform angle to wanted convention
    if ((ar_meas > 1 && obj_params_.aspect_ratio < 1) ||
        (ar_meas < 1 && obj_params_.aspect_ratio > 1)) {
      angle = rect.angle + 90;
    } else {
      angle = rect.angle;
    }

    out << rect.center.x, rect.center.y, rect.size.width, rect.size.height,
        angle * M_PI / 180;
    // std::cout << "Angle " << rect.angle << std::endl;
    return out;
  }

  // Used if bounding box is aligned
  template <bool is_aligned_bb = aligned_bb>
  static MeasureVec convertRotRectToMeasureVec(
      typename std::enable_if<is_aligned_bb, const cv::RotatedRect&>::type
          rect) {
    cv::Rect2f upright_rect = rect.boundingRect2f();
    MeasureVec out;
    // Top left corner position given
    out << upright_rect.x + upright_rect.width / 2,
        upright_rect.y + upright_rect.height / 2, upright_rect.width,
        upright_rect.height;
    return out;
  }

  /* measurement model definition */
  MeasureVec H(const StateVec& state, const InputType& u) {
    Eigen::Affine3f tf_ego_ado;
    state_to_tf(state, tf_ego_ado);

    Eigen::MatrixXf model_points_cam;

    model_points_cam = ((cam_params_.tf_cam_ego * tf_ego_ado).matrix() *
                        obj_params_.model_points_ado_aug_t)
                           .topRows<3>();
    model_points_cam.transposeInPlace();
    std::vector<cv::Point2f> projected_points;
    cv::Mat points_cv;
    cv::eigen2cv(model_points_cam, points_cv);

    cv::projectPoints(points_cv,
                      cv::Mat::zeros(3, 1, CV_32F),
                      cv::Mat::zeros(3, 1, CV_32F),
                      cam_params_.K,
                      cam_params_.dist_coeffs,
                      projected_points);

    MeasureVec out =
        convertRotRectToMeasureVec(cv::minAreaRect(projected_points));
    return out;
  }

  /* process model definition */
  /* Applied before the propagation using dF (defined above) and the
   * integrators. Updates that rely on integrations of the state should be
   * defined on the dF function. Here is good for changing the frame before
   * integration. */
  StateVec G(const StateVec& state, const InputType& tf_ego_egoprev) {
    StateVec out;

    // Position
    out.head<3>() = tf_ego_egoprev * state.head<3>();
    // Linear velocity
    out.segment<3>(3) = tf_ego_egoprev.linear() * state.segment<3>(3);
    // Angular velocity
    out.segment<3>(6) = tf_ego_egoprev.linear() * state.segment<3>(6);
    // Quaternion representation of orientation
    out.quat(0) = Eigen::Quaterniond(tf_ego_egoprev.linear() * state.quat(0));
    return out;
  }

  void updateCamera(msl_raptor_backend::CameraParams cam_params) {
    cam_params_ = cam_params;
  }

  void updateCamIntrinsics(const float ppx,
                           const float ppy,
                           const float fx,
                           const float fy,
                           const cv::Mat& dist_coeffs_in) {
    cam_params_.updateIntrinsics(ppx, ppy, fx, fy, dist_coeffs_in);
  }

  void updateCamIntrinsics(const std::vector<float>& camera_matrix_in,
                           const std::vector<float>& dist_coeffs_in) {
    cam_params_.updateIntrinsics(camera_matrix_in, dist_coeffs_in);
  }

  void updateCamIntrinsics(const cv::Mat& camera_matrix_in,
                           const cv::Mat& dist_coeffs_in) {
    cam_params_.updateIntrinsics(camera_matrix_in, dist_coeffs_in);
  }

  void updateCamExtrinsics(const Eigen::Affine3d& extrinsics_mat) {
    cam_params_.updateExtrinsics(extrinsics_mat);
  }

  void updateCamExtrinsics(const cv::Mat& rvec_in, const cv::Mat& tvec_in) {
    cam_params_.updateExtrinsics(rvec_in, tvec_in);
  }

  void updateCamExtrinsics(const std::vector<float>& rvec_in,
                           const std::vector<float>& tvec_in) {
    cam_params_.updateExtrinsics(rvec_in, tvec_in);
  }

  void update(double delta_t,
              const MeasureVec& z,
              const InputType& input = Eigen::Affine3d::Identity()) {
    ukf_->update(delta_t, z, input);
  }

  StateVec getState() {
    return ukf_->state;
  }

  Eigen::MatrixXd getStateRootCov() {
    return ukf_->stateRootCov;
  }

  void init(msl_raptor_backend::ObjectParams obj_params,
            msl_raptor_backend::CameraParams cam_params,
            StateVec init_state) {
    obj_params_ = obj_params;
    cam_params_ = cam_params;
    ukf_ = new kalman::UKF<MSLRaptorUKF, InputType>(*this);
    ukf_->state = init_state;
  }

  MSLRaptorUKF(void) {}

  MSLRaptorUKF(msl_raptor_backend::ObjectParams obj_params,
               msl_raptor_backend::CameraParams cam_params,
               StateVec init_state) {
    init(obj_params, cam_params, init_state);
  }

  MSLRaptorUKF(msl_raptor_backend::ObjectParams obj_params,
               msl_raptor_backend::CameraParams cam_params,
               MeasureVec bb_for_approx_init) {
    obj_params_ = obj_params;
    cam_params_ = cam_params;
    StateVec init_state = approxStateFromBb(bb_for_approx_init);
    init(obj_params, cam_params, init_state);
  }

private:
  msl_raptor_backend::ObjectParams obj_params_;
  msl_raptor_backend::CameraParams cam_params_;
  kalman::UKF<MSLRaptorUKF, InputType>* ukf_;
};
