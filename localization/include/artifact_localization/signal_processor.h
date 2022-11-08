#pragma once

#include <algorithm>
#include <deque>

#include <artifact_msgs/PointSourceDetection.h>

struct SignalProcessorConfig {
  double strength_min;
  double strength_max;
  double range_sigma_coeff0 = 0;
  double range_sigma_coeff1 = 0;
  double confidence_min = 0;
  double confidence_max = 1;

  // Moving filter
  int moving_window_size = 3;
};

class SignalProcessor {
public:
  typedef std::shared_ptr<SignalProcessor> Ptr;
  typedef std::shared_ptr<SignalProcessor const> ConstPtr;

  SignalProcessor(
      const SignalProcessorConfig& config = SignalProcessorConfig());
  ~SignalProcessor();

  void setConfig(const SignalProcessorConfig& config) {
    config_ = config;
  }

  double filterMovingAverage(const double strength);

  bool getRangeFromSignalStrength(const double strength,
                                  double& range,
                                  double& range_sigma);
  bool getConfidenceFromSignalStrength(const double strength,
                                       double& confidence);

  bool isDetectionValid(const artifact_msgs::PointSourceDetectionConstPtr& msg);

protected:
  double scaleBetween(double value,
                      double in_min,
                      double in_max,
                      double out_min,
                      double out_max) {
    return (value - in_min) / (in_max - in_min) * (out_max - out_min) + out_min;
  }
  double truncate(double value, double min, double max) {
    return std::min(std::max(value, min), max);
  }

  SignalProcessorConfig config_;

  // For moving window filter
  std::deque<double> queue_;
};