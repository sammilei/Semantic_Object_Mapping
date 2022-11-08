#include "artifact_localization/artifact.h"

#include <cv_bridge/cv_bridge.h>
#include <tf2_eigen/tf2_eigen.h>

#include <opencv2/highgui/highgui_c.h> // C
#include <opencv2/imgproc/imgproc_c.h> // C
#include <opencv2/opencv.hpp>          // C++

int Artifact::num_total_artifacts_ = 0;

Artifact::Artifact(const ArtifactConfig& config, const std::string& id_hint)
  : config_(config) {
  // Copy artifact properties to internal data storage
  data_.label = config.name;
  data_.name = config.robot_name;

  // Assign unique ID
  if (id_hint.empty()) {
    data_.id = generateUniqueId();
    ROS_DEBUG("Auto-generated ID: %s", data_.id.c_str());
  } else {
    data_.id = id_hint;
    ROS_DEBUG("Using provided ID: %s", data_.id.c_str());
  }
  data_.parent_id = data_.id;
}

Artifact::~Artifact() {}

bool Artifact::minimalObservationsMade() {
  if (!isPositionValid()) {
    return false;
  }

  if (getNumObservations() < config_.min_num_observations) {
    return false;
  }

  return true;
}

bool Artifact::enoughObservationsMade() {
  // Prerequisite
  if (!minimalObservationsMade()) {
    return false;
  }

  // Condition 1:
  // - Max detection confidence
  // - Localization uncertainty
  if (getLargestSigma() <= config_.sigma_min &&
      data_.confidence >= config_.confidence_max) {
    return true;
  }

  // Condition 2:
  // - Timestamp of latest observation
  if ((ros::Time::now() - last_observation_time_).toSec() >=
      config_.stale_timeout) {
    return true;
  }

  return false;
}

cv_bridge::CvImagePtr Artifact::concatenateThumbnails() {
  // Add image if available
  bool best_thumbnail = false;
  bool closest_thumbnail = false;
  bool last_thumbnail = false;
  bool brightest_thumbnail = false;
  bool least_blurry_thumbnail = false;

  // last seen thumbnail
  if (thumbnail_) {
    cv::Mat thumbnail_last_mat;
    if (thumbnail_last_) {
      last_thumbnail = true;
      thumbnail_last_mat = cv_bridge::toCvCopy(thumbnail_last_, "bgr8")->image;
    } else {
      thumbnail_last_mat = cv_bridge::toCvCopy(thumbnail_, "bgr8")->image;
      thumbnail_last_mat.convertTo(thumbnail_last_mat, -1, 0, 0);
      ROS_ERROR("it does not have last !!");
    }

    // best thumbnail
    cv::Mat thumbnail_best_mat;
    if (thumbnail_best_) {
      best_thumbnail = true;
      thumbnail_best_mat = cv_bridge::toCvCopy(thumbnail_best_, "bgr8")->image;
    } else {
      thumbnail_best_mat = cv_bridge::toCvCopy(thumbnail_, "bgr8")->image;
      thumbnail_best_mat.convertTo(thumbnail_best_mat, -1, 0, 0);
      ROS_ERROR("it does not have best !!");
    }

    // brightest thumbnail
    cv::Mat thumbnail_brightest_mat;
    if (thumbnail_brightest_) {
      brightest_thumbnail = true;
      thumbnail_brightest_mat =
          cv_bridge::toCvCopy(thumbnail_brightest_, "bgr8")->image;
    } else {
      thumbnail_brightest_mat = cv_bridge::toCvCopy(thumbnail_, "bgr8")->image;
      thumbnail_brightest_mat.convertTo(thumbnail_brightest_mat, -1, 0, 0);
      ROS_ERROR("it does not have brightest !!");
    }

    // closest thumbnail
    cv::Mat thumbnail_closest_mat;
    if (thumbnail_closest_) {
      closest_thumbnail = true;
      thumbnail_closest_mat =
          cv_bridge::toCvCopy(thumbnail_closest_, "bgr8")->image;
    } else {
      thumbnail_closest_mat = cv_bridge::toCvCopy(thumbnail_, "bgr8")->image;
      thumbnail_closest_mat.convertTo(thumbnail_closest_mat, -1, 0, 0);
      ROS_ERROR("it does not have closest !!");
    }

    // less blurry thumbnail
    cv::Mat least_bluriness_mat;
    if (thumbnail_less_blurry_) {
      least_blurry_thumbnail = true;
      least_bluriness_mat =
          cv_bridge::toCvCopy(thumbnail_less_blurry_, "bgr8")->image;
    } else {
      least_bluriness_mat = cv_bridge::toCvCopy(thumbnail_, "bgr8")->image;
      least_bluriness_mat.convertTo(least_bluriness_mat, -1, 0, 0);
      ROS_ERROR("it does not have least blurry !!");
    }

    // concatenate thumbnails
    // best: 0, brightest: 1, least blurry: 2, closest:3, (last:4)

    cv_bridge::CvImagePtr cv_ptr_thumbnail_top_row;
    try {
      cv_ptr_thumbnail_top_row = cv_bridge::toCvCopy(thumbnail_, "bgr8");
      cv::hconcat(thumbnail_best_mat,
                  thumbnail_brightest_mat,
                  cv_ptr_thumbnail_top_row->image);
    } catch (const std::exception& ex) {
      ROS_ERROR("Failed to horizontally concatenate best confidence and "
                "brightest image (step 1): %s",
                ex.what());
      return cv_bridge::toCvCopy(thumbnail_, "bgr8");
    }

    cv_bridge::CvImagePtr cv_ptr_thumbnail_bottom_row;
    try {
      cv_ptr_thumbnail_bottom_row = cv_bridge::toCvCopy(thumbnail_, "bgr8");
      if (thumbnail_closest_) {
        cv::hconcat(least_bluriness_mat,
                    thumbnail_closest_mat,
                    cv_ptr_thumbnail_bottom_row->image);
      } else {
        data_.xmin[3] = data_.xmin[4];
        data_.ymin[3] = data_.ymin[4];
        data_.xmax[3] = data_.xmax[4];
        data_.ymax[3] = data_.ymax[4];
        cv::hconcat(least_bluriness_mat,
                    thumbnail_last_mat,
                    cv_ptr_thumbnail_bottom_row->image);
      }
    } catch (const std::exception& ex) {
      ROS_ERROR("Failed to horizontally concatenate least blurry and closest "
                "image (step 2): %s",
                ex.what());
      return cv_bridge::toCvCopy(thumbnail_, "bgr8");
    }

    cv_bridge::CvImagePtr cv_ptr_thumbnail;
    try {
      cv_ptr_thumbnail = cv_bridge::toCvCopy(thumbnail_, "bgr8");
      cv::vconcat(cv_ptr_thumbnail_top_row->image,
                  cv_ptr_thumbnail_bottom_row->image,
                  cv_ptr_thumbnail->image);
    } catch (const std::exception& ex) {
      ROS_ERROR(
          "Failed to vertically concatenate the 4 images together (step 3): %s",
          ex.what());
      return cv_bridge::toCvCopy(thumbnail_, "bgr8");
    }

    // resize the thumbnail. Height to be around 800 pixels for kyon to see
    // clearely
    int width = cv_ptr_thumbnail->image.cols;
    int height = cv_ptr_thumbnail->image.rows;

    int desired_height = 360;
    float scale = float(height) / float(desired_height);

    if (scale > 1) {
      int down_width = floor(width / scale);
      int down_height = floor(height / scale);
      if (down_width % 2 == 1) {
        down_width = down_width + 1;
      }
      if (down_height % 2 == 1) {
        down_height = down_height + 1;
      }
      cv::resize(cv_ptr_thumbnail->image,
                 cv_ptr_thumbnail->image,
                 cv::Size(down_width, down_height),
                 cv::INTER_LINEAR);
      int width = cv_ptr_thumbnail->image.cols;
      int height = cv_ptr_thumbnail->image.rows;
      // TODO in the future change the rosnode to output xmin, xmax etc between
      // 0 and 1 and update whole pipeline
      for (int i = 0; i < data_.xmin.size(); i++) {
        data_.xmin[i] = floor(data_.xmin[i] / scale);
        data_.ymin[i] = floor(data_.ymin[i] / scale);
        data_.xmax[i] = floor(data_.xmax[i] / scale);
        data_.ymax[i] = floor(data_.ymax[i] / scale);
      }
    }

    return cv_ptr_thumbnail;
  } else {
    ROS_WARN("Could not create a mosaic thumbnail");
    cv_bridge::CvImagePtr cv_ptr_thumbnail;
    return cv_ptr_thumbnail;
  }
}

bool Artifact::createMosaique() {
  if (thumbnail_) {
    cv_bridge::CvImagePtr cv_ptr_thumbnail = concatenateThumbnails();
    thumbnail_ = cv_ptr_thumbnail->toCompressedImageMsg();
    return true;
  }
  return false;
}

artifact_msgs::ArtifactPtr Artifact::toMessageInMap() {
  auto msg = boost::make_shared<artifact_msgs::Artifact>(data_);

  // Update header
  msg->header.stamp = last_observation_time_;
  msg->header.frame_id = config_.map_frame;

  if (thumbnail_) {
    msg->thumbnail = *thumbnail_;
  }

  // Add position in map frame
  // Use best (highest confidence) observation time if available
  if (best_observation_time_.isZero()) {
    ROS_DEBUG("Composing artifact message using the latest time");
    msg->point.header.stamp = last_observation_time_;
  } else {
    ROS_DEBUG("Composing artifact message using the highest confidence time");
    msg->point.header.stamp = best_observation_time_;
  }
  msg->point.header.frame_id = config_.map_frame;

  auto pos = getPositionInMap();
  if (pos.array().isNaN().any()) {
    return nullptr;
  }
  msg->point.point = tf2::toMsg(pos);

  if (covariance_.array().isNaN().any()) {
    msg->covariance[0] = -1;
  } else {
    auto cov = getCovarianceInMap();
    for (int i = 0; i < msg->covariance.size(); ++i) {
      msg->covariance[i] = cov(i);
    }
  }

  // Add statistics
  msg->num_observations = getNumObservations();

  return msg;
}

// Convert map frame to base link frame at the time
// the best observation was taken
artifact_msgs::ArtifactPtr Artifact::toMessageInBaseLink() {
  // Fill in all fields using map frame
  auto msg = toMessageInMap();
  if (!msg) {
    return nullptr;
  }

  // Get transform from timestamp
  Eigen::Affine3d map_T_base_link;
  if (msg->point.header.stamp == best_observation_time_) {
    map_T_base_link = best_map_T_base_link_;
  } else if (msg->point.header.stamp == last_observation_time_) {
    map_T_base_link = last_map_T_base_link_;
  } else {
    ROS_ERROR("Could not infer map to base link transform");
    return nullptr;
  }

  // Transform positions and covariance
  msg->header.frame_id = config_.base_link_frame;
  msg->point.header.frame_id = config_.base_link_frame;

  auto pos = transformPosition(map_T_base_link, position_);
  if (pos.array().isNaN().any()) {
    return nullptr;
  }
  msg->point.point = tf2::toMsg(pos);

  if (!covariance_.array().isNaN().any()) {
    auto cov = transformCovariance(map_T_base_link, covariance_);
    for (int i = 0; i < msg->covariance.size(); ++i) {
      msg->covariance[i] = cov(i);
    }
  }

  for (auto& point_msg : msg->observation_points) {
    Eigen::Vector3d point;
    tf2::fromMsg(point_msg, point);
    point_msg = tf2::toMsg(transformPosition(map_T_base_link, point));
  }

  return msg;
}

void Artifact::removeInvalidValue(std::vector<double>& vec,
                                  const double invalid_value) {
  if (!vec.empty()) {
    // remove the elements = invalid_value in the vector
    vec.erase(std::remove(vec.begin(), vec.end(), invalid_value), vec.end());
    if (vec.empty()) {
      ROS_WARN("The vector is now empty because all values were invalid");
    }
  }
}

bool Artifact::getTruncatedMeanFromVector(const std::vector<double> vec,
                                          double& truncated_mean,
                                          const double remove_tail_default) {
  // remove_tail must be between 0 and 0.5
  double remove_tail = remove_tail_default;
  if (remove_tail >= 0.5 || remove_tail < 0) {
    ROS_ERROR("The remove_tail parameter must be between 0 and 0.5. The "
              "current value of %.2f is not valid - will do normal mean",
              remove_tail);
    remove_tail = 0; // do a normal mean because truncated mean would be invalid
  }
  // the vector must be sorted
  std::vector<double> vec_copy = vec;
  std::sort(vec_copy.begin(), vec_copy.end());
  if (vec_copy.empty()) {
    ROS_WARN("Vector is empty - truncated mean return -1");
    truncated_mean = -1;
    return false;
  } else {
    int v_size = vec_copy.size();
    int start = floor(v_size * remove_tail);
    int end = v_size - start;
    std::vector<double>::const_iterator first = vec_copy.begin() + start;
    std::vector<double>::const_iterator last = vec_copy.begin() + end;
    std::vector<double> v(first, last);

    double sum_artifact = std::accumulate(v.begin(), v.end(), 0.0);
    double mean_artifact = sum_artifact / v.size();

    double sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / v.size() - mean_artifact * mean_artifact);
    truncated_mean = mean_artifact;
    return true;
  }
}

// combine the results for scorability
void Artifact::combineAllScores(const double final_yolo_score,
                                const double final_color_score,
                                const double final_size_score,
                                double& scorability) {
  bool has_color_score = (final_color_score != -1);
  bool has_size_score = (final_size_score != -1);
  bool has_yolo_score = (final_yolo_score != -1);
  // VERY WEIRD situtation, when we do not have a yolo score -- SHOULD NOT
  // HAPPEN

  if (!has_yolo_score) {
    if ((!has_color_score) || (!has_size_score)) {
      if ((!has_color_score) && (!has_size_score)) {
        scorability = 0;
        ROS_ERROR("BIG ERROR - no color - no yolo score - no size score --> "
                  "scorability = 0");
      } else if (!has_color_score) {
        scorability = final_size_score;
        ROS_ERROR("BIG ERROR - no color and no yolo score --> scorability = "
                  "size score");
      } else {
        scorability = final_color_score;
        ROS_ERROR("BIG ERROR - no yolo and no size score --> scorability = "
                  "color score");
      }
    } else {
      scorability = final_color_score * final_size_score;
      ROS_ERROR("BIG ERROR - no yolo score --> scorability = color score * "
                "size score");
    }
  }
  // normal situtation, when we have a yolo score but maybe not color or size
  else {
    if ((!has_color_score) || (!has_size_score)) {
      if ((!has_color_score) && (!has_size_score)) {
        scorability = final_yolo_score;
        ROS_WARN(
            "WARNING - no color - no size score --> scorability = yolo score");
      } else if (!has_color_score) {
        scorability = final_yolo_score * final_size_score;
        ROS_WARN("WARNING - no color score --> scorability = yolo score * size "
                 "score");
      } else {
        scorability = final_yolo_score * final_color_score;
        ROS_WARN("WARNING - no size score --> scorability = yolo score * color "
                 "score");
      }
    } else {
      scorability = final_yolo_score * final_color_score * final_size_score;
      ROS_INFO("PERFECT - We have a yolo + color + size score");
    }
  }
}

void Artifact::reconcileAllScores(std::vector<double> const& color_scores,
                                  std::vector<double> const& yolo_scores,
                                  std::vector<double> const& size_scores,
                                  double& scorability,
                                  double& final_yolo_score,
                                  double& final_color_score,
                                  double& final_size_score) {
  std::vector<double> color_scores_copy = color_scores;
  std::vector<double> yolo_scores_copy = yolo_scores;
  std::vector<double> size_scores_copy = size_scores;

  // get the color score from vector
  removeInvalidValue(color_scores_copy, -1);
  if (!getTruncatedMeanFromVector(
          color_scores_copy, final_color_score, config_.remove_tail)) {
    ROS_WARN("Failed to compute truncated mean for color score");
  };
  if (final_color_score != -1) {
    final_color_score = final_color_score /
        100.0; // the color score is between 0-100 not 0-1 like others
  }

  // get the size score from vector
  removeInvalidValue(size_scores_copy, -1);
  if (!getTruncatedMeanFromVector(
          size_scores_copy, final_size_score, config_.remove_tail)) {
    ROS_WARN("Failed to compute truncated mean for size score");
  }

  // get the size score from vector
  removeInvalidValue(yolo_scores_copy, -1);
  if (!getTruncatedMeanFromVector(
          yolo_scores_copy, final_yolo_score, config_.remove_tail)) {
    ROS_WARN("Failed to compute truncated mean for yolo score");
  }

  // get the final scorability results by combining all scores
  combineAllScores(
      final_yolo_score, final_color_score, final_size_score, scorability);
}

bool Artifact::updateScorability(artifact_msgs::ArtifactPtr& one_artifact) {
  // non visual artifacts: scorability = confidence
  if (one_artifact->detection_source == "wifi" ||
      one_artifact->detection_source == "gas" ||
      one_artifact->detection_source == "bluetooth") {
    one_artifact->scorability_metrics.detection_confidence_value = -1;
    one_artifact->scorability_metrics.color_confidence_value = -1;
    one_artifact->scorability_metrics.size_confidence_value = -1;
    one_artifact->scorability = one_artifact->confidence;
    ROS_WARN("This is not a visual artifact so scorability = confidence");
    return true;
  } else {
    double confidence_artifact = one_artifact->confidence;
    std::string artifact_id = one_artifact->id;
    std::string artifact_type = artifact_id.substr(0, 2);
    int min_nbr_observations_scorability =
        config_.scorability_min_num_observations;

    // from the list of confidence, get the final confidence (could be max,
    // mean, median etc.)
    double yolo_confidence_artifact;
    std::vector<double> list_yolo_confidence_artifact =
        one_artifact->scorability_metrics.yolo_confidence_vector;

    // from the list of color score, get the final color score (could be max,
    // mean, median etc.)
    double color_confidence_artifact;
    std::vector<double> list_color_confidence_artifact =
        one_artifact->scorability_metrics.color_confidence_vector;

    // from the list of size score, get the final size score (could be max,
    // mean, median etc.)
    double size_confidence_artifact;
    std::vector<double> list_size_confidence_artifact =
        one_artifact->scorability_metrics.size_confidence_vector;

    // combine the yolo confidence, color score and size score to get the
    // scorability
    double scorability_color_size_conf;
    double final_yolo_conf;
    double final_color_conf;
    double final_size_conf;

    reconcileAllScores(list_color_confidence_artifact,
                       list_yolo_confidence_artifact,
                       list_size_confidence_artifact,
                       scorability_color_size_conf,
                       final_yolo_conf,
                       final_color_conf,
                       final_size_conf);
    ROS_WARN("the scorability color-conf-size is: %.4f",
             scorability_color_size_conf);

    // linear penalty for low number of detections
    int nbr_obs = one_artifact->num_observations;
    if (nbr_obs < min_nbr_observations_scorability) {
      double mult_factor;
      mult_factor = (double)nbr_obs / min_nbr_observations_scorability;
      one_artifact->scorability = scorability_color_size_conf * mult_factor;
      ROS_WARN(
          "the artifact had only %d observations so it is penalized because "
          "we want at least %d",
          nbr_obs,
          min_nbr_observations_scorability);
    } else {
      one_artifact->scorability = scorability_color_size_conf;
    }
    one_artifact->scorability_metrics.detection_confidence_value =
        final_yolo_conf;
    one_artifact->scorability_metrics.color_confidence_value = final_color_conf;
    one_artifact->scorability_metrics.size_confidence_value = final_size_conf;
    // one_artifact->confidence = -10;
    return true;
  }
}

sensor_msgs::CompressedImagePtr Artifact::toImageMessage() {
  if (!thumbnail_) {
    return nullptr;
  }

  return boost::make_shared<sensor_msgs::CompressedImage>(*thumbnail_);
}

sensor_msgs::ImagePtr Artifact::toAnnotatedImageMessage() {
  if (!thumbnail_) {
    return nullptr;
  }

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(*thumbnail_);
  } catch (cv_bridge::Exception& ex) {
    return nullptr;
  }

  // Gamma correction
  double gamma = 0.7;
  cv::Mat table(1, 256, CV_8U);
  for (int i = 0; i < table.cols; ++i) {
    table.at<uchar>(i) =
        cv::saturate_cast<uchar>(std::pow(i / 255.0, gamma) * 255.0);
  }
  cv::LUT(cv_ptr->image, table, cv_ptr->image);

  // Draw bounding box
  cv::rectangle(cv_ptr->image,
                cv::Point(data_.xmin[0], data_.ymin[0]),
                cv::Point(data_.xmax[0], data_.ymax[0]),
                /* color = */ cv::Scalar(255),
                /* linewidth = */ 1);

  return cv_ptr->toImageMsg();
}

double Artifact::getReconciliationScore(const Observation& obs) {
  // Artifact class must match
  if (getLabel() != obs.label) {
    return 0.0;
  }

  // Type-spefific reconciliation
  if (getLabel() == "Cell Phone") {
    return getCellPhoneReconciliationScore(obs);
  } else if (getLabel() == "Cube" && obs.signal_msg) {
    return getCubeSignalReconciliationScore(obs);
  } else if (getLabel() == "Gas") {
    return getGasReconciliationScore(obs);
  } else {
    return getVisualReconciliationScore(obs);
  }
}

double Artifact::getCellPhoneReconciliationScore(const Observation& obs) {
  // Cell phone should have the same SSID
  if (getId() == obs.signal_msg->id) {
    return 1.0;
  } else {
    return 0.0;
  }
}

double Artifact::getCubeSignalReconciliationScore(const Observation& obs) {
  // Cube should have the same SSID
  if (getId() == obs.signal_msg->id) {
    return 1.0;
  } else {
    return 0.0;
  }
}

double Artifact::getGasReconciliationScore(const Observation& obs) {
  // Check distance to the closest past observation
  Eigen::MatrixXd past_poses(3, data_.observation_points.size());
  for (int i = 0; i < data_.observation_points.size(); ++i) {
    past_poses(0, i) = data_.observation_points[i].x;
    past_poses(1, i) = data_.observation_points[i].y;
    past_poses(2, i) = data_.observation_points[i].z;
  }

  Eigen::VectorXd distances =
      Eigen::MatrixXd(past_poses.colwise() - obs.map_T_base_link.translation())
          .colwise()
          .norm();

  // Penalize if the measurements has large z differences
  // (likely on different levels)
  for (int i = 0; i < data_.observation_points.size(); ++i) {
    if (std::abs(data_.observation_points[i].z -
                 obs.map_T_base_link.translation()(2)) >
        config_.reconcile_height_tolerance) {
      distances[i] = 1e6;
    }
  }

  // Set linear score based on distance
  double min_distance = distances.minCoeff();
  return std::max<double>(0.0, 1.0 - min_distance / config_.reconcile_range);
}

double Artifact::getVisualReconciliationScore(const Observation& obs) {
  // Reject if the time gap is too large
  double delta_time = (obs.stamp - last_observation_time_).toSec();
  if (delta_time > config_.reconcile_timeout) {
    ROS_DEBUG("Reconciliation to %s rejected due to time gap: %.1f s",
              getId().c_str(),
              delta_time);
    return 0.0;
  }

  // Reject if the direction is far off
  Eigen::Vector3d current_estimate = getPositionInMap();
  Eigen::Vector3d current_bearing =
      (obs.map_T_sensor.inverse() * current_estimate).normalized();
  double angle_error = std::acos(current_bearing.dot(obs.bearing));
  if (angle_error > config_.reconcile_angle_tolerance) {
    ROS_DEBUG("Reconciliation to %s rejected due to angular error: %.2f deg",
              getId().c_str(),
              180.0 / M_PI * angle_error);
    return 0.0;
  }

  // Reject far-away objects
  double range = (obs.map_T_sensor.inverse() * current_estimate).norm();
  if (range > config_.range_max) {
    ROS_DEBUG("Reconciliation to %s rejected due to range: %.2f m",
              getId().c_str(),
              range);
    return 0.0;
  }

  // Reject objects at different levels
  double z_offset =
      std::abs(current_estimate[2] - obs.map_T_base_link.translation()[2]);
  if (z_offset > config_.reconcile_height_tolerance) {
    ROS_DEBUG("Reconciliation to %s rejected due to height difference: %.2f m",
              getId().c_str(),
              z_offset);
    return 0.0;
  }

  // Approximate distance error by point-to-line distance
  double distance = (obs.map_T_sensor.translation() - current_estimate)
                        .cross(obs.bearing)
                        .norm();
  ROS_DEBUG("Point-to-line distance: %.2f m", distance);

  return std::max<double>(0.0, 1.0 - distance / config_.reconcile_range);
}

bool Artifact::addBearingObservation(const Observation& obs) {
  // Sanity check
  if (!obs.isBearingValid()) {
    return false;
  }

  // Update statistics
  updateConfidence(obs); // in addImageObservation already update for max conf
                         // only if enoughobservation() is false
  updateObservationInfo(obs.stamp, obs.map_T_base_link);
  data_.observation_points.push_back(tf2::toMsg(Eigen::Vector3d(
      obs.map_T_base_link.translation()))); // here we could do something
                                            // similar for artifact likelihood
  ++num_bearing_;

  // If enough observations are made, do not update anymore   // this is
  // disagree that it is a good choice (at least worth discussing)
  if (enoughObservationsMade()) {
    ROS_DEBUG("Rejecting redundant bearing observation for %s",
              getId().c_str());
    return false;
  }

  // Add range prior for the first measurement
  if (num_bearing_ == 1) {
    double nominal_range = (config_.range_min + config_.range_max) / 2.0;
    optimizer_.addRangeMeasurement(
        nominal_range, nominal_range, obs.stamp, obs.map_T_sensor);
  }

  // Add bearing measurement
  optimizer_.addBearingMeasurement(
      obs.bearing, obs.bearing_sigma, obs.stamp, obs.map_T_sensor);

  // Update estimate
  updatePositionEstimate();

  return true;
}

bool Artifact::addRangeObservation(const Observation& obs) {
  // Sanity check
  if (!obs.isRangeValid()) {
    return false;
  }

  const double height = obs.height;
  const double width = obs.width;
  const double size_score = obs.size_confidence;
  double range = obs.range;

  // Update statistics
  updateObservationInfo(obs.stamp, obs.map_T_base_link);

  // add the height and width of observation
  updateObservationsize(height, width, size_score);

  // If enough observations are made, do not update anymore
  if (enoughObservationsMade()) {
    ROS_DEBUG("Rejecting redundant range observation for %s", getId().c_str());
    return false;
  }

  // Add range measurement
  optimizer_.addRangeMeasurement(
      obs.range, obs.range_sigma, obs.stamp, obs.map_T_sensor);

  // Update estimate
  updatePositionEstimate();

  // Update closest thumbnail
  if (range != -1) {
    if (range < thumbnail_closest_distance_) {
      thumbnail_closest_distance_ = range;
      use_thumbnail_close_ = true;
    }
  }

  return true;
}

float Artifact::calcBlurriness(const cv::Mat& src) {
  cv::Mat Gx, Gy;
  cv::Sobel(src, Gx, CV_32F, 1, 0);
  cv::Sobel(src, Gy, CV_32F, 0, 1);
  double normGx = cv::norm(Gx);
  double normGy = cv::norm(Gy);
  double sumSq = normGx * normGx + normGy * normGy;
  return static_cast<float>(1. / (sumSq / src.size().area() + 1e-6));
}

void Artifact::updateBoundingBoxLocation(const Observation& obs,
                                         const int idx) {
  if (obs.vision_msg) {
    data_.xmin[idx] = obs.vision_msg->box.xmin;
    data_.ymin[idx] = obs.vision_msg->box.ymin;
    data_.xmax[idx] = obs.vision_msg->box.xmax;
    data_.ymax[idx] = obs.vision_msg->box.ymax;
  } else {
    data_.xmin[idx] = 0;
    data_.ymin[idx] = 0;
    data_.xmax[idx] = obs.image->width - 1;
    data_.ymax[idx] = obs.image->height - 1;
  }
}

bool Artifact::updateThumbnails(const Observation& obs) {
  // Convert color to BGR8
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat receivedImg = cv_bridge::toCvCopy(*obs.image, "bgr8")->image;

  try {
    cv_ptr = cv_bridge::toCvCopy(*obs.image, "bgr8");
  } catch (cv_bridge::Exception& ex) {
    ROS_ERROR("Failed to change image encoding (Source: %s)",
              obs.image->encoding.c_str());
    return false;
  }

  // get the brightness
  cv::Mat hsv;
  cv::cvtColor(receivedImg, hsv, CV_BGR2HSV);
  const auto result = cv::mean(hsv);
  // cv::mean() will return 3 numbers, one for each channel:
  //      0=hue
  //      1=saturation
  //      2=value (brightness)
  double brightness = result[2];
  float bluriness = calcBlurriness(receivedImg);

  // best: 0, brightest: 1, least blurry: 2, closest:3, last:4
  if (bluriness < least_bluriness_) {
    least_bluriness_ = bluriness;
    thumbnail_less_blurry_ = cv_ptr->toCompressedImageMsg();
    updateBoundingBoxLocation(obs, 2);
  }
  if (brightness > thumbnail_brightest_value_) {
    thumbnail_brightest_value_ = brightness;
    thumbnail_brightest_ = cv_ptr->toCompressedImageMsg();
    updateBoundingBoxLocation(obs, 1);
  }
  if (use_thumbnail_close_) {
    use_thumbnail_close_ = false;
    thumbnail_closest_ = cv_ptr->toCompressedImageMsg();
    updateBoundingBoxLocation(obs, 3);
  }
  thumbnail_last_ = cv_ptr->toCompressedImageMsg();
  updateBoundingBoxLocation(obs, 4);
  return true;
}

bool Artifact::addImageObservation(const Observation& obs) {
  // Sanity check
  if (!obs.isImageValid()) {
    return false;
  }

  // Update statistics
  updateObservationInfo(obs.stamp, obs.map_T_base_link);

  // Update thumbnails
  if (!updateThumbnails(obs)) {
    ROS_WARN("Issue when updating thumbnails");
    return false;
  }

  // If enough observations are made, do not update anymore
  if (enoughObservationsMade()) {
    ROS_DEBUG("Rejecting redundant image observation for %s", getId().c_str());
    return false;
  }

  // Update image only if confidence is higher
  if (obs.confidence < data_.confidence) {
    ROS_DEBUG("Rejecting low-confidence image observations: %s "
              "(current: %.1f%%, best: %.1f%%)",
              getId().c_str(),
              100.0 * obs.confidence,
              100.0 * data_.confidence);
    return false;
  }

  // Copy fields
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat receivedImg = cv_bridge::toCvCopy(*obs.image, "bgr8")->image;
  try {
    cv_ptr = cv_bridge::toCvCopy(*obs.image, "bgr8");
  } catch (cv_bridge::Exception& ex) {
    ROS_ERROR("Failed to change image encoding (Source: %s)",
              obs.image->encoding.c_str());
    return false;
  }

  data_.confidence = obs.confidence;
  thumbnail_ = cv_ptr->toCompressedImageMsg();
  thumbnail_best_ = cv_ptr->toCompressedImageMsg();
  data_.detection_source = obs.detection_source;
  updateBoundingBoxLocation(obs, 0);

  best_observation_time_ = obs.stamp;
  best_map_T_base_link_ = obs.map_T_base_link;
  return true;
}

bool Artifact::addSignalObservation(const Observation& obs) {
  // Update statistics
  updateObservationInfo(obs.stamp, obs.map_T_base_link);
  data_.observation_points.push_back(
      tf2::toMsg(Eigen::Vector3d(obs.map_T_base_link.translation())));
  ++num_signal_;

  // If enough observations are made, do not update anymore
  if (enoughObservationsMade()) {
    ROS_DEBUG("Rejecting redundant range observation for %s", getId().c_str());
    return false;
  }

  // NOTE: The range measurement of signal measurement is not available
  //       Assume the current robot position as the artifact position
  //       and update only when it reduces covariance
  if (!isPositionValid() || obs.range_sigma <= getLargestSigma()) {
    position_ = obs.map_T_sensor.translation();
    Eigen::Vector3d sigmas(
        obs.range_sigma, obs.range_sigma, config_.height_sigma);
    covariance_ = Eigen::Matrix3d(sigmas.asDiagonal()).array().pow(2);

    ROS_DEBUG("Position updated: %s (%.1f, %.1f, %.1f)",
              getId().c_str(),
              position_.x(),
              position_.y(),
              position_.z());
  }

  return true;
}

std::string Artifact::generateUniqueId() {
  // Label
  // ex) Backpack --> ba
  std::string label =
      config_.abbreviation.empty() ? "na" : config_.abbreviation;

  // Robot name
  // ex) husky4 --> h4
  std::string robot_name;
  if (data_.name.empty()) {
    robot_name = "r0";
  } else {
    robot_name =
        data_.name.substr(0, 1) + data_.name.substr(data_.name.size() - 1);
  }

  // Compose ID
  // ex) su001_h4
  char id[10];
  snprintf(id,
           sizeof(id),
           "%s%03d_%s",
           label.c_str(),
           ++num_total_artifacts_,
           robot_name.c_str());
  return std::string(id);
}

void Artifact::updateObservationInfo(const ros::Time& stamp,
                                     const Eigen::Affine3d& map_T_base_link) {
  if (first_observation_time_.isZero()) {
    first_observation_time_ = stamp;
    first_map_T_base_link_ = map_T_base_link;
  }
  if (stamp > last_observation_time_) {
    last_observation_time_ = stamp;
    last_map_T_base_link_ = map_T_base_link;
  }
}

void Artifact::updateObservationsize(const double height,
                                     const double width,
                                     const double size_score) {
  if (height != -1 && width != -1) {
    data_.scorability_metrics.height_vector.push_back(height);
    data_.scorability_metrics.width_vector.push_back(width);
    data_.scorability_metrics.size_confidence_vector.push_back(size_score);
  }
}

void Artifact::updateConfidence(const Observation& obs) {
  // append the list of color and yolo confidence to this artifact
  data_.scorability_metrics.color_confidence_vector.push_back(
      obs.color_confidence);
  data_.scorability_metrics.yolo_confidence_vector.push_back(
      obs.yolo_confidence);

  // it is debatable if we want to do this
  // this will update the confidence to get the max at every observation
  // if we don't do this, the confidence only updated from 0 to max number of
  // observation
  data_.confidence = std::max((double)obs.confidence, (double)data_.confidence);
}

bool Artifact::updatePositionEstimate() {
  ROS_DEBUG("Running position estimate update");

  if (!optimizer_.optimize()) {
    ROS_WARN("Pose optimization failed");
    return false;
  }

  position_ = optimizer_.getPosition();
  covariance_ = optimizer_.getCovariance();

  ROS_DEBUG("Position updated: %s (%.1f, %.1f, %.1f)",
            getId().c_str(),
            position_.x(),
            position_.y(),
            position_.z());

  // Sanity check
  if (getLargestSigma() > config_.sigma_max) {
    ROS_WARN("Large covariance! sigma=%.1fm", getLargestSigma());
  }
  return true;
}
