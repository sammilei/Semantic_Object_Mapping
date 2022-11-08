#include "artifact_localization/gas_strength_processor.h"

GasStrengthProcessor::GasStrengthProcessor() {}

GasStrengthProcessor::~GasStrengthProcessor() {}

bool GasStrengthProcessor::loadParameters(const ros::NodeHandle& nh) {
  if (!nh.getParam("co2_detection/dynamic_window_size", dynamic_window_size_))
    return false;
  if (!nh.getParam("co2_detection/ratio_1", ratio_1_))
    return false;
  if (!nh.getParam("co2_detection/ratio_2", ratio_2_))
    return false;
  if (!nh.getParam("co2_detection/variance_penalty", variance_penalty_))
    return false;
  return true;
}

void GasStrengthProcessor::updateStrengthBuffer(
    const artifact_msgs::PointSourceDetection& det) {
  if (strength_buffer_.size() >= dynamic_window_size_) {
    strength_buffer_.erase(strength_buffer_.begin());
  }
  strength_buffer_.push_back(det.strength);
  float standard_deviation_strength;
  calculateMeanVarianceStrength(mean_strength_, standard_deviation_strength);
  auto variance_penalty_strength =
      variance_penalty_ * standard_deviation_strength;
  ROS_INFO_STREAM("Mean: " << mean_strength_);
  ROS_INFO_STREAM("Variance Penalty: " << variance_penalty_strength);
  dynamic_threshold_min_ =
      ratio_1_ * mean_strength_ + variance_penalty_strength;
  dynamic_threshold_max_ = ratio_2_ * dynamic_threshold_min_;
  dynamic_threshold_without_variance_penalty_ = ratio_1_ * mean_strength_;
  ROS_INFO_STREAM("CO2 dynamic threshold: " << dynamic_threshold_min_);
}

float GasStrengthProcessor::getMinDynamicThreshold() const {
  return dynamic_threshold_min_;
}

float GasStrengthProcessor::getMaxDynamicThreshold() const {
  return dynamic_threshold_max_;
}

float GasStrengthProcessor::getMeanStrength() const {
  return mean_strength_;
}

float GasStrengthProcessor::getDynamicThresholdWithoutVariancePenalty() const {
  return dynamic_threshold_without_variance_penalty_;
}

void GasStrengthProcessor::calculateMeanVarianceStrength(
    float& mean_strength, float& standard_deviation_strength) {
  float sum_strength = 0;
  auto data_size = strength_buffer_.size();
  for (const auto strength : strength_buffer_) {
    sum_strength += strength;
  }
  mean_strength = sum_strength / (float)(data_size);
  float deviation_sum = 0;
  for (const auto strength : strength_buffer_) {
    deviation_sum += std::pow((strength - mean_strength), 2.0);
  }
  standard_deviation_strength = std::sqrt(deviation_sum / (float)(data_size));
}
