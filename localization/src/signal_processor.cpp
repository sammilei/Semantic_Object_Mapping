#include "artifact_localization/signal_processor.h"

SignalProcessor::SignalProcessor(const SignalProcessorConfig& config)
  : config_(config) {}

SignalProcessor::~SignalProcessor() {}

bool SignalProcessor::getRangeFromSignalStrength(const double strength,
                                                 double& range,
                                                 double& range_sigma) {
  double trunc_strength =
      truncate(strength, config_.strength_min, config_.strength_max);

  // TODO: Get valid range estimation
  range = 0;
  range_sigma =
      config_.range_sigma_coeff0 + config_.range_sigma_coeff1 * trunc_strength;
  return true;
}

bool SignalProcessor::getConfidenceFromSignalStrength(const double strength,
                                                      double& confidence) {
  double trunc_strength =
      truncate(strength, config_.strength_min, config_.strength_max);
  confidence = scaleBetween(trunc_strength,
                            config_.strength_min,
                            config_.strength_max,
                            config_.confidence_min,
                            config_.confidence_max);
  return true;
}

bool SignalProcessor::isDetectionValid(
    const artifact_msgs::PointSourceDetectionConstPtr& msg) {
  if (msg->strength < config_.strength_min) {
    return false;
  }

  return true;
}

double SignalProcessor::filterMovingAverage(const double strength) {
  // Update queue
  queue_.push_back(strength);
  while (queue_.size() > config_.moving_window_size) {
    queue_.pop_front();
  }

  // Compute average over window
  double ave_strength = 0;
  for (auto value : queue_) {
    ave_strength += value / queue_.size();
  }

  return ave_strength;
}
