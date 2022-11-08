#include "artifact_localization/config.h"

#include <ros/ros.h>

void LocalizerConfig::loadROSParams(const ros::NodeHandle& nh) {
  // Topic names
  loadParam(nh, "rgb/image_topic", rgb_image_topic);
  loadParam(nh, "rgb/info_topic", rgb_info_topic);
  loadParam(nh, "rgb/detection_topic", rgb_detection_topic);
  loadParamWithDefault(nh, "rgb/cache_size", rgb_cache_size);
  loadParamWithDefault(nh, "rgb/sync_tolerance", rgb_sync_tolerance);

  loadParam(nh, "thermal/image_topic", thermal_image_topic);
  loadParam(nh, "thermal/info_topic", thermal_info_topic);
  loadParam(nh, "thermal/detection_topic", thermal_detection_topic);
  loadParamWithDefault(nh, "thermal/cache_size", thermal_cache_size);
  loadParamWithDefault(nh, "thermal/sync_tolerance", thermal_sync_tolerance);

  loadParam(nh, "depth/image_topic", depth_image_topic);
  loadParamWithDefault(nh, "depth/cache_size", depth_cache_size);
  loadParamWithDefault(nh, "depth/sync_tolerance", depth_sync_tolerance);

  loadParam(nh, "lidar/scan_topic", lidar_scan_topic);
  loadParam(nh, "wifi/detection_topic", wifi_detection_topic);
  loadParam(nh, "bluetooth/detection_topic", bluetooth_detection_topic);
  loadParam(nh, "gas/detection_topic", gas_detection_topic);

  // Camera names
  loadParam(nh, "rgb/cameras", rgb_cameras);
  loadParam(nh, "depth/cameras", depth_cameras);
  loadParam(nh, "thermal/cameras", thermal_cameras);

  // Frame IDs
  loadParam(nh, "frames/map_frame", map_frame);
  loadParam(nh, "frames/base_link_frame", base_link_frame);
  loadParam(nh, "frames/odom_frame", odom_frame);

  // Misc
  loadParamWithDefault(nh, "robot_name", robot_name);
  loadParamWithDefault(nh, "debug_dir", debug_dir);
  loadParamWithDefault(nh, "debug_topic", debug_topic);
  loadParamWithDefault(nh, "debug_topic_queue_size", debug_topic_queue_size);
  loadParamWithDefault(nh, "debug_topic_latch", debug_topic_latch);

  // Artifact-specific parameters
  // clang-format off
  std::vector<std::string> artifact_types;
  loadParam(nh, "artifacts/types", artifact_types);
  for (auto& type : artifact_types) {
    std::string name;
    loadParam(nh, "artifacts/" + type + "/name", name);
    auto& config = artifact_configs[name];

    auto artifact_nh = ros::NodeHandle(nh, "artifacts/" + type);
    loadParam(artifact_nh, "name", config.name);
    loadParam(artifact_nh, "abbreviation", config.abbreviation);
    loadParamWithDefault(artifact_nh, "confidence_min", config.confidence_min);
    loadParamWithDefault(artifact_nh, "confidence_max", config.confidence_max);
    loadParamWithDefault(artifact_nh, "sigma_min", config.sigma_min);
    loadParamWithDefault(artifact_nh, "sigma_max", config.sigma_max);
    loadParamWithDefault(artifact_nh, "stale_timeout", config.stale_timeout);
    loadParamWithDefault(artifact_nh, "min_num_observations", config.min_num_observations);
    loadParamWithDefault(artifact_nh, "range_min", config.range_min);
    loadParamWithDefault(artifact_nh, "range_max", config.range_max);
    loadParamWithDefault(artifact_nh, "height_sigma", config.height_sigma);
    loadParamWithDefault(artifact_nh, "reconcile/range", config.reconcile_range);
    loadParamWithDefault(artifact_nh, "reconcile/timeout", config.reconcile_timeout);
    loadParamWithDefault(artifact_nh, "scorability/min_num_observations", config.scorability_min_num_observations);

    config.map_frame = map_frame;
    config.base_link_frame = base_link_frame;
    config.robot_name = robot_name;
  }
  // clang-format on

  // Signal sensor parameters
  // clang-format off
  for (auto& sensor_name : signal_sensor_names) {
    auto& config = signal_configs[sensor_name];
    auto sensor_nh = ros::NodeHandle(nh, sensor_name);

    loadParam(sensor_nh, "strength_min", config.strength_min);
    loadParam(sensor_nh, "strength_max", config.strength_max);
    loadParamWithDefault(sensor_nh, "range_sigma_coeff0", config.range_sigma_coeff0);
    loadParamWithDefault(sensor_nh, "range_sigma_coeff1", config.range_sigma_coeff1);
    loadParamWithDefault(sensor_nh, "confidence_min", config.confidence_min);
    loadParamWithDefault(sensor_nh, "confidence_max", config.confidence_max);
    loadParamWithDefault(sensor_nh, "moving_window_size", config.moving_window_size);
  }
  // clang-format on

  // Estimation algorithm
  std::string estimation_method_str;
  loadParam(nh, "estimation_method", estimation_method_str);
  if (estimation_method_str == "batch_optimization") {
    estimation_method = EstimationMethod::BATCH_OPTIMIZATION;
  } else if (estimation_method_str == "kalman") {
    estimation_method = EstimationMethod::KALMAN;
  } else if (estimation_method_str == "msl_raptor") {
    estimation_method = EstimationMethod::MSL_RAPTOR;
  } else {
    ROS_WARN("Unknown estimation method: %s", estimation_method_str.c_str());
  }

  // Depth algorithm
  std::string depth_method_str;
  loadParam(nh, "depth_method", depth_method_str);
  if (depth_method_str == "image") {
    depth_method = DepthMethod::IMAGE;
  } else if (depth_method_str == "lidar") {
    depth_method = DepthMethod::LIDAR;
  } else {
    ROS_WARN("Unknown depth method: %s", depth_method_str.c_str());
  }
}
