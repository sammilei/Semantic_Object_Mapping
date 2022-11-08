#pragma once

#include <unordered_map>

#include <ros/ros.h>

#include "artifact_localization/artifact.h"
#include "artifact_localization/signal_processor.h"

struct LocalizerConfig {
  // Topic configurations
  std::string rgb_image_topic;
  std::string rgb_info_topic;
  std::string rgb_detection_topic;
  int rgb_cache_size = 100;
  double rgb_sync_tolerance = 0.5; // [s]

  std::string thermal_image_topic;
  std::string thermal_info_topic;
  std::string thermal_detection_topic;
  int thermal_cache_size = 100;
  double thermal_sync_tolerance = 0.5; // [s]

  std::string depth_image_topic;
  int depth_cache_size = 100;
  double depth_sync_tolerance = 0.5; // [s]

  std::string lidar_scan_topic;
  std::string wifi_detection_topic;
  std::string bluetooth_detection_topic;
  std::string gas_detection_topic;

  // Camera names
  std::vector<std::string> rgb_cameras;
  std::vector<std::string> depth_cameras;
  std::vector<std::string> thermal_cameras;

  // TFs
  std::string map_frame;
  std::string base_link_frame;
  std::string odom_frame;
  double tf_timeout = 3.0;

  // Artifact-specific parameters
  std::unordered_map<std::string, ArtifactConfig> artifact_configs;
  double min_translation_between_observations = 0.1;

  // saving detection report for analysing results true or false
  bool save_detection_report = false;

  // Signal sensor parameters
  std::vector<std::string> signal_sensor_names = {"wifi", "bluetooth", "gas"};
  std::unordered_map<std::string, SignalProcessorConfig> signal_configs;

  // Algorithm configuration
  enum EstimationMethod {
    BATCH_OPTIMIZATION,
    KALMAN,
    MSL_RAPTOR,
  };
  EstimationMethod estimation_method = EstimationMethod::BATCH_OPTIMIZATION;

  enum DepthMethod {
    IMAGE = 1,
    LIDAR = 2,
  };
  DepthMethod depth_method = DepthMethod::IMAGE;

  // Misc
  std::string robot_name = "robot0";
  std::string debug_dir = "";
  std::string debug_topic = "";
  int debug_topic_queue_size = 1;
  bool debug_topic_latch = false;

public:
  void loadROSParams(const ros::NodeHandle& nh);

protected:
  template <typename T>
  void loadParamWithDefault(const ros::NodeHandle& nh,
                            const std::string& name,
                            T& value) noexcept;
  template <typename T>
  void loadParam(const ros::NodeHandle& nh, const std::string& name, T& value);
  template <typename T>
  void loadParam(const ros::NodeHandle& nh,
                 const std::string& name,
                 std::vector<T>& value);
};

template <typename T>
void LocalizerConfig::loadParamWithDefault(const ros::NodeHandle& nh,
                                           const std::string& name,
                                           T& value) noexcept {
  T default_value = value;
  if (nh.getParam(name, value)) {
    ROS_INFO_STREAM("Loaded: " << nh.resolveName(name) << " = " << value);
  } else {
    ROS_WARN_STREAM("Using default: " << nh.resolveName(name) << " = "
                                      << default_value);
    value = default_value;
  }
}

template <typename T>
void LocalizerConfig::loadParam(const ros::NodeHandle& nh,
                                const std::string& name,
                                T& value) {
  if (nh.getParam(name, value)) {
    ROS_INFO_STREAM("Loaded: " << nh.resolveName(name) << " = " << value);
  } else {
    ROS_ERROR_STREAM("Failed to get param '" << nh.resolveName(name)
                                             << "'. Terminating");
    throw std::runtime_error("Missing required parameter");
  }
}

template <typename T>
void LocalizerConfig::loadParam(const ros::NodeHandle& nh,
                                const std::string& name,
                                std::vector<T>& value) {
  if (nh.getParam(name, value)) {
    ROS_INFO_STREAM("Loaded: " << nh.resolveName(name));
  } else {
    ROS_ERROR_STREAM("Failed to get param '" << nh.resolveName(name)
                                             << "'. Terminating");
    throw std::runtime_error("Missing required parameter");
  }
}
