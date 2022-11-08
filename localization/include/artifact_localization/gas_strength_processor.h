#pragma once

#include <artifact_msgs/PointSourceDetection.h>
#include <ros/ros.h>

class GasStrengthProcessor {
public:
  GasStrengthProcessor();
  ~GasStrengthProcessor();
  bool loadParameters(const ros::NodeHandle& nh);
  void updateStrengthBuffer(const artifact_msgs::PointSourceDetection& det);
  float getMinDynamicThreshold() const;
  float getMaxDynamicThreshold() const;
  float getMeanStrength() const;
  float getDynamicThresholdWithoutVariancePenalty() const;

private:
  std::vector<float> strength_buffer_;
  int dynamic_window_size_;
  float mean_strength_;
  float dynamic_threshold_min_;
  float dynamic_threshold_max_;
  float dynamic_threshold_without_variance_penalty_;
  float ratio_1_;
  float ratio_2_;
  float variance_penalty_;

  void calculateMeanVarianceStrength(float& mean_strength,
                                     float& variance_strength);
};
