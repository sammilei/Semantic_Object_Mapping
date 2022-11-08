#include "artifact_localization/localizer.h"

#include <sstream>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <artifact_msgs/Artifact.h>
#include <darknet_ros_msgs/DetectionReport.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>

Localizer::Localizer(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : nh_(nh),
    pnh_(pnh),
    it_(nh_),
    tf_buf_(ros::Duration(120)),
    tf_listener_(tf_buf_) {
  tf_buf_.setUsingDedicatedThread(true);
  loadParameters();
}

Localizer::~Localizer() {}

void Localizer::run() {
  // Wait until TF becomes available
  ros::Rate r(10);
  while (!tf_buf_.canTransform(
      config_.map_frame, config_.base_link_frame, ros::Time(0))) {
    ROS_INFO_THROTTLE(10.0,
                      "Waiting for TF to become available: %s --> %s",
                      config_.map_frame.c_str(),
                      config_.base_link_frame.c_str());
    ros::spinOnce();
    r.sleep();
  }
  ROS_INFO("TF now available");

  // Initialize message processing flow
  initializePublishers();
  initializeSubscribers();
  initializeQueues();
  initializeServices();

  // Set timer callbacks
  auto setup_timer =
      nh_.createTimer(ros::Duration(5.0), &Localizer::setupSensorClbk, this);
  auto pub_timer = nh_.createTimer(
      ros::Duration(1.0), &Localizer::publishArtifactClbk, this);

  ros::spin();
}

void Localizer::loadParameters() {
  ROS_INFO("Loading parameters");

  config_.loadROSParams(pnh_);

  ROS_INFO("Successfully initialized all parameters");
}

void Localizer::initializePublishers() {
  artifact_pub_ = nh_.advertise<artifact_msgs::Artifact>("artifact", 100);
  artifact_update_pub_ =
      nh_.advertise<artifact_msgs::Artifact>("artifact/update", 100);
  thumbnail_pub_ = pnh_.advertise<sensor_msgs::Image>("thumbnail", 10);

  raw_meas_pub_ =
      pnh_.advertise<visualization_msgs::Marker>("measurement_raw", 10);
}

void Localizer::initializeSubscribers() {
  // Detection subscribers
  auto vision_callback = boost::bind(&Localizer::visionDetectionClbk, this, _1);
  auto gas_callback =
      boost::bind(&Localizer::signalDetectionClbk, this, _1, "Gas", "gas");
  auto wifi_callback = boost::bind(
      &Localizer::signalDetectionClbk, this, _1, "WiFi", "wifi");
  auto bluetooth_callback = boost::bind(
      &Localizer::signalDetectionClbk, this, _1, "Bluetooth", "bluetooth");

  subs_[config_.rgb_detection_topic] = nh_.subscribe<darknet_ros_msgs::Object>(
      config_.rgb_detection_topic, 1000, vision_callback);
  subs_[config_.thermal_detection_topic] =
      nh_.subscribe<darknet_ros_msgs::Object>(
          config_.thermal_detection_topic, 1000, vision_callback);
  subs_[config_.gas_detection_topic] =
      nh_.subscribe<artifact_msgs::PointSourceDetection>(
          config_.gas_detection_topic, 1, gas_callback);
  subs_[config_.wifi_detection_topic] =
      nh_.subscribe<artifact_msgs::PointSourceDetection>(
          config_.wifi_detection_topic, 1, wifi_callback);
  subs_[config_.bluetooth_detection_topic] =
      nh_.subscribe<artifact_msgs::PointSourceDetection>(
          config_.bluetooth_detection_topic, 1, bluetooth_callback);

  // Image subscribers
  for (const auto& camera_name : config_.rgb_cameras) {
    std::string topic = camera_name + "/" + config_.rgb_image_topic;
    auto f = boost::bind(&Localizer::rgbClbk, this, _1, camera_name);
    it_subs_[topic] = it_.subscribe(topic, 10, f);
  }


  if (config_.depth_method == LocalizerConfig::DepthMethod::IMAGE) {
    for (const auto& camera_name : config_.depth_cameras) {
      std::string topic = camera_name + "/" + config_.depth_image_topic;
      auto f = boost::bind(&Localizer::depthClbk, this, _1, camera_name);
      it_subs_[topic] = it_.subscribe(topic, 10, f);
    }
  }

  for (const auto& camera_name : config_.thermal_cameras) {
    std::string topic = camera_name + "/" + config_.thermal_image_topic;
    auto f = boost::bind(&Localizer::thermalClbk, this, _1, camera_name);
    it_subs_[topic] = it_.subscribe(topic, 10, f);
  }

  // Lidar subscriber
  subs_[config_.lidar_scan_topic] =
      nh_.subscribe(config_.lidar_scan_topic, 1, &Localizer::lidarClbk, this);
}

// Set up message filter pipeline so the callbacks are invoked nicely as the
// messages arrive
void Localizer::initializeQueues() {
  // Image buffers
  for (auto& camera_name : config_.rgb_cameras) {
    rgb_bufs_[camera_name] = std::make_shared<ImageBuffer>();
  }

  for (auto& camera_name : config_.depth_cameras) {
    depth_bufs_[camera_name] = std::make_shared<ImageBuffer>();
  }

  for (auto& camera_name : config_.thermal_cameras) {
    thermal_bufs_[camera_name] = std::make_shared<ImageBuffer>();
  }

  // Observation buffers
  for (auto& camera_name : config_.rgb_cameras) {
    obs_bufs_[camera_name] = std::make_shared<ObservationBuffer>();
  }

  for (auto& camera_name : config_.thermal_cameras) {
    obs_bufs_[camera_name] = std::make_shared<ObservationBuffer>();
  }

  for (auto& sensor_name : config_.signal_sensor_names) {
    signal_bufs_[sensor_name] = std::make_shared<ObservationBuffer>();
  }

  // Observation-image synchronization buffers
  for (auto& camera_name : config_.rgb_cameras) {
    SyncPolicy policy(config_.rgb_cache_size);
    obs_image_bufs_[camera_name] =
        std::make_shared<ObservationImageSync>(policy);
    obs_image_bufs_[camera_name]->connectInput(*obs_bufs_[camera_name],
                                               *rgb_bufs_[camera_name]);
  }

  for (auto& camera_name : config_.thermal_cameras) {
    SyncPolicy policy(config_.thermal_cache_size);
    policy.setMaxIntervalDuration(ros::Duration(config_.rgb_sync_tolerance));
    obs_image_bufs_[camera_name] =
        std::make_shared<ObservationImageSync>(policy);
    obs_image_bufs_[camera_name]->connectInput(*obs_bufs_[camera_name],
                                               *thermal_bufs_[camera_name]);
  }

  for (auto& camera_name : config_.depth_cameras) {
    SyncPolicy policy(config_.depth_cache_size);
    policy.setMaxIntervalDuration(ros::Duration(config_.depth_sync_tolerance));
    obs_depth_bufs_[camera_name] =
        std::make_shared<ObservationImageSync>(policy);
    obs_depth_bufs_[camera_name]->connectInput(*obs_bufs_[camera_name],
                                               *depth_bufs_[camera_name]);
  }

  auto signal_camera_name = config_.rgb_cameras.front(); // Pick first camera
  for (auto& sensor_name : config_.signal_sensor_names) {
    SyncPolicy policy(config_.rgb_cache_size);
    policy.setMaxIntervalDuration(
        ros::Duration(config_.thermal_sync_tolerance));
    signal_image_bufs_[sensor_name] =
        std::make_shared<ObservationImageSync>(policy);
    signal_image_bufs_[sensor_name]->connectInput(
        *signal_bufs_[sensor_name], *rgb_bufs_[signal_camera_name]);
  }

  // Add measurement callbacks
  for (auto& it : obs_bufs_) {
    auto& sensor_name = it.first;
    auto& buf = it.second;

    // Observation statistics
    buf->registerCallback([&](const Observation::ConstPtr& obs) {
      observationStatsClbk(boost::const_pointer_cast<Observation>(obs),
                           sensor_name + "/obs");
    });

    // Bearing
    if (config_.estimation_method ==
        LocalizerConfig::EstimationMethod::BATCH_OPTIMIZATION) {
      ROS_INFO_ONCE("Registering 'bearing optimization' callback");
      buf->registerCallback([&](const Observation::ConstPtr& obs) {
        bearingClbk(boost::const_pointer_cast<Observation>(obs));
      });
    }

    // Range from lidar
    if (config_.depth_method == LocalizerConfig::DepthMethod::LIDAR &&
        config_.debug_dir == "" && config_.debug_topic == "") {
      ROS_INFO_ONCE("Registering 'lidar range' callback");
      buf->registerCallback([&](const Observation::ConstPtr& obs) {
        lidarRangeClbk(boost::const_pointer_cast<Observation>(obs));
      });
    }
  }

  for (auto& it : signal_bufs_) {
    auto& sensor_name = it.first;
    auto& buf = it.second;

    // Observation statistics
    buf->registerCallback([&](const Observation::ConstPtr& obs) {
      observationStatsClbk(boost::const_pointer_cast<Observation>(obs),
                           sensor_name + "/obs");
    });

    // Signal from point source
    ROS_INFO_ONCE("Registering 'point-source' callback");
    buf->registerCallback([&](const Observation::ConstPtr& obs) {
      signalClbk(boost::const_pointer_cast<Observation>(obs));
    });
  }

  for (auto& it : obs_image_bufs_) {
    auto& sensor_name = it.first;
    auto& buf = it.second;

    // Observation statistics
    buf->registerCallback(boost::bind<void>(
        [&](const Observation::ConstPtr& obs,
            const sensor_msgs::ImageConstPtr& img) {
          observationStatsClbk(boost::const_pointer_cast<Observation>(obs),
                               sensor_name + "/obs+image");
        },
        _1,
        _2));

    // Thumbnail
    ROS_INFO_ONCE("Registering 'visual thumbnail' callback");
    buf->registerCallback(boost::bind<void>(
        [&](const Observation::ConstPtr& obs,
            const sensor_msgs::ImageConstPtr& img) {
          thumbnailClbk(boost::const_pointer_cast<Observation>(obs), img);
        },
        _1,
        _2));

    // Thumbnail
    if (config_.depth_method == LocalizerConfig::DepthMethod::LIDAR &&
        (config_.debug_dir != "" || config_.debug_topic != "")) {
      ROS_INFO_ONCE("Registering 'lidar range debug' callback");
      buf->registerCallback(boost::bind<void>(
          [&](const Observation::ConstPtr& obs,
              const sensor_msgs::ImageConstPtr& img) {
            lidarRangeClbk(boost::const_pointer_cast<Observation>(obs), img);
          },
          _1,
          _2));
    }

    // MSL-Raptor
    if (config_.estimation_method ==
        LocalizerConfig::EstimationMethod::MSL_RAPTOR) {
      ROS_INFO_ONCE("Registering 'MSL-Raptor' callback");
      buf->registerCallback(boost::bind<void>(
          [&](const Observation::ConstPtr& obs,
              const sensor_msgs::ImageConstPtr& img) {
            mslRaptorClbk(boost::const_pointer_cast<Observation>(obs), img);
          },
          _1,
          _2));
    }
  }

  for (auto& it : obs_depth_bufs_) {
    auto& sensor_name = it.first;
    auto& buf = it.second;

    // Observation statistics
    buf->registerCallback(boost::bind<void>(
        [&](const Observation::ConstPtr& obs,
            const sensor_msgs::ImageConstPtr& img) {
          observationStatsClbk(boost::const_pointer_cast<Observation>(obs),
                               sensor_name + "/obs+depth");
        },
        _1,
        _2));

    // Range from depth
    if (config_.estimation_method ==
        LocalizerConfig::EstimationMethod::BATCH_OPTIMIZATION) {
      ROS_INFO_ONCE("Registering 'depth optimization' callback");
      buf->registerCallback(boost::bind<void>(
          [&](const Observation::ConstPtr& obs,
              const sensor_msgs::ImageConstPtr& img) {
            depthRangeClbk(boost::const_pointer_cast<Observation>(obs), img);
          },
          _1,
          _2));
    }

    // Kalman filter
    if (config_.estimation_method ==
        LocalizerConfig::EstimationMethod::KALMAN) {
      ROS_INFO_ONCE("Registering 'kalman' filter callback");
      buf->registerCallback(boost::bind<void>(
          [&](const Observation::ConstPtr& obs,
              const sensor_msgs::ImageConstPtr& img) -> void {
            kalmanClbk(boost::const_pointer_cast<Observation>(obs), img);
          },
          _1,
          _2));
    }
  }

  for (auto& it : signal_image_bufs_) {
    auto& sensor_name = it.first;
    auto& buf = it.second;

    // Observation statistics
    buf->registerCallback(boost::bind<void>(
        [&](const Observation::ConstPtr& obs,
            const sensor_msgs::ImageConstPtr& img) -> void {
          observationStatsClbk(boost::const_pointer_cast<Observation>(obs),
                               sensor_name + "/obs+image");
        },
        _1,
        _2));

    // Thumbnail
    ROS_INFO_ONCE("Registering 'point-source thumbnail' callback");
    buf->registerCallback(boost::bind<void>(
        [&](const Observation::ConstPtr& obs,
            const sensor_msgs::ImageConstPtr& img) {
          thumbnailClbk(boost::const_pointer_cast<Observation>(obs), img);
        },
        _1,
        _2));
  }
}

// Initialize artifact localization services
void Localizer::initializeServices() {
  // Publish all remaining full artifact reports
  publish_reports_srv_ = nh_.advertiseService(
      "publish_reports", &Localizer::publishReportsClbk, this);
}

Camera::Ptr Localizer::createCameraModel(const std::string& camera_name,
                                         const std::string& info_topic) {
  // Get intrinsics and extrinsics
  ros::Duration timeout(0.5);
  auto topic = camera_name + "/" + info_topic;
  ROS_DEBUG("Subscribing to camera info topic: %s", topic.c_str());
  auto info =
      ros::topic::waitForMessage<sensor_msgs::CameraInfo>(topic, timeout);
  if (!info) {
    ROS_ERROR_THROTTLE(
        60, "Failed to get intrinsics for camera '%s'", camera_name.c_str());
    return nullptr;
  }

  Eigen::Affine3d base_link_T_sensor;
  if (!getTransformEigenFromTf(config_.base_link_frame,
                               info->header.frame_id,
                               ros::Time(0),
                               base_link_T_sensor)) {
    ROS_ERROR("Failed to get extrinsics for camera '%s'", camera_name.c_str());
    return nullptr;
  }

  // Generate camera object
  auto camera = std::make_shared<Camera>(camera_name);
  camera->setIntrinsicsFromCameraInfo(info);
  camera->setExtrinsics(base_link_T_sensor);
  return camera;
}

// Initialize dynamic sensor parameters
void Localizer::setupSensorClbk(const ros::TimerEvent& event) {
  ROS_INFO_ONCE("Setting up sensors");

  // RGB cameras
  for (const auto& camera_name : config_.rgb_cameras) {
    if (hasKey(cameras_, camera_name)) {
      continue;
    }

    auto camera = createCameraModel(camera_name, config_.rgb_info_topic);
    if (camera) {
      cameras_[camera_name] = camera;
      ROS_INFO("Initialized RGB camera '%s'", camera_name.c_str());

      // Initialize RGB camera debug image publishers
      if (!config_.debug_topic.empty()) {
        for (auto const& key : camera->getDebugImageNames()) {
          std::string topic =
              camera_name + "/" + config_.debug_topic + "/" + key;
          debug_image_pubs_[camera_name][key] =
              pnh_.advertise<sensor_msgs::Image>(topic,
                                                 config_.debug_topic_queue_size,
                                                 config_.debug_topic_latch);
        }
      }
    }
  }

  // Thermal cameras
  for (const auto& camera_name : config_.thermal_cameras) {
    if (hasKey(cameras_, camera_name)) {
      continue;
    }

    auto camera = createCameraModel(camera_name, config_.thermal_info_topic);
    if (camera) {
      cameras_[camera_name] = camera;
      ROS_INFO("Initialized thermal camera '%s'", camera_name.c_str());

      // Initialize thermal camera debug image publishers
      if (!config_.debug_topic.empty()) {
        for (auto const& key : camera->getDebugImageNames()) {
          std::string topic =
              camera_name + "/" + config_.debug_topic + "/" + key;
          debug_image_pubs_[camera_name][key] =
              pnh_.advertise<sensor_msgs::Image>(topic,
                                                 config_.debug_topic_queue_size,
                                                 config_.debug_topic_latch);
        }
      }
    }
  }

  // Signal sensors
  for (const auto& sensor_name : config_.signal_sensor_names) {
    if (!hasKey(signal_processors_, sensor_name)) {
      signal_processors_[sensor_name] = std::make_shared<SignalProcessor>(
          config_.signal_configs.at(sensor_name));
      ROS_INFO("Initialized signal processor: '%s'", sensor_name.c_str());
    }
  }
}

void Localizer::publishArtifactClbk(const ros::TimerEvent& event) {
  int num_msgs = 0;
  for (auto& artifact : artifacts_) {
    if (!artifact->isPublished() && artifact->enoughObservationsMade() &&
        artifact->isThumbnailValid()) {
      publishFullReport(artifact);
      artifact->setPublished();
      ++num_msgs;
    }
  }

  if (num_msgs > 0) {
    ROS_INFO("Published %d artifacts", num_msgs);
  }
}

void Localizer::rgbClbk(const sensor_msgs::ImageConstPtr& msg,
                        const std::string& camera_name) {
  rgb_bufs_.at(camera_name)->add(msg);
}

void Localizer::depthClbk(const sensor_msgs::ImageConstPtr& msg,
                          const std::string& camera_name) {
  depth_bufs_.at(camera_name)->add(msg);
}

void Localizer::thermalClbk(const sensor_msgs::ImageConstPtr& msg,
                            const std::string& camera_name) {
  thermal_bufs_.at(camera_name)->add(msg);
}

void Localizer::lidarClbk(const sensor_msgs::PointCloud2ConstPtr& msg) {
  latest_lidar_msg_ = msg;
}

void Localizer::visionDetectionClbk(
    const darknet_ros_msgs::ObjectConstPtr& msg) {
  // Extract observation information
  auto obs = boost::make_shared<Observation>(msg);

  if (!hasKey(cameras_, msg->camera_name)) {
    std::string known_camera_names = "";
    std::string delim = "";
    for (auto& known_camera : cameras_) {
      auto& name = known_camera.first;
      known_camera_names = known_camera_names + delim + name;
      delim = ",";
    }
    ROS_ERROR("Unknown camera name: %s. (Known cameras are: %s)", msg->camera_name.c_str(), known_camera_names.c_str());
    return;
  }
  obs->camera = cameras_.at(msg->camera_name);

  // Process detection
  processDetection(obs);
}

void Localizer::signalDetectionClbk(
    const artifact_msgs::PointSourceDetectionConstPtr& msg,
    const std::string& label,
    const std::string& detection_source) {
  // Extract observation information
  auto obs = boost::make_shared<Observation>(msg);
  obs->detection_source = detection_source;

  if (!hasKey(signal_processors_, detection_source)) {
    ROS_ERROR("Unknown detection source: %s", detection_source.c_str());
    return;
  }
  obs->signal_processor = signal_processors_.at(detection_source);

  obs->strength = obs->signal_processor->filterMovingAverage(msg->strength);
  ROS_DEBUG("Signal strength after smoothing: %.1f (original: %.1f)",
            obs->strength,
            msg->strength);

  obs->signal_processor->getConfidenceFromSignalStrength(obs->strength,
                                                         obs->confidence);

  // Process detection
  processDetection(obs);
}

bool Localizer::processDetection(const Observation::Ptr& obs) {
  // Check if detection quality
  if (!obs->isGoodDetection()) {
    ROS_DEBUG("Detection quality is low: %s (%.1f%%)",
              obs->label.c_str(),
              100.0 * obs->confidence);
    return false;
  }

  // Query pose
  obs->map_frame = config_.map_frame;
  obs->base_link_frame = config_.base_link_frame;

  if (!getTransformEigenFromTf(obs->map_frame,
                               obs->base_link_frame,
                               obs->stamp,
                               obs->map_T_base_link,
                               ros::Duration(config_.tf_timeout))) {
    ROS_ERROR("Failed to obtain pose");
    return false;
  }
  if (obs->camera) {
    obs->map_T_sensor = obs->map_T_base_link * obs->camera->getExtrinsics();
    obs->sensor_frame = obs->camera->getFrame();
  } else {
    obs->map_T_sensor = obs->map_T_base_link;
    obs->sensor_frame = config_.base_link_frame;
  }

  // Reject observation from the same position
  if (!hasMovedEnough(obs->label, obs->map_T_base_link)) {
    ROS_DEBUG_THROTTLE(1, "No movement since previous observation. Dropping");
    return false;
  }

  // Process bearing measurements for reconciliation
  if (obs->camera && obs->vision_msg) {
    obs->camera->getBearingFromBoundingBox(
        obs->vision_msg->box, obs->bearing, obs->bearing_sigma);
    if (obs->isBearingValid()) {
      ROS_DEBUG("Bearing computed for reconciliation");
    }
  }

  // Find the same artifact in the database. If not, create a new one
  if (!reconcileOrCreateArtifact(obs)) {
    ROS_ERROR("Failed to reconcile/create artifact report");
    return false;
  }

  // Put in buffer. This will trigger message filter pipeline
  if (obs->vision_msg) {
    obs_bufs_.at(obs->detection_source)->add(obs);
  } else if (obs->signal_msg) {
    signal_bufs_.at(obs->detection_source)->add(obs);
  } else {
    ROS_ERROR("Discarding unknown observation type");
    return false;
  }

  ROS_INFO("Added new '%s' detection (confidence=%.1f%%) from '%s'",
           obs->label.c_str(),
           100.0 * obs->confidence,
           obs->detection_source.c_str());

  return true;
}

// Count the number of observations
void Localizer::observationStatsClbk(const Observation::Ptr& obs,
                                     const std::string& category) {
  if (!hasKey(message_stats_, category)) {
    message_stats_[category] = 0;
  }
  ++message_stats_[category];

  // Print message statistics
  std::stringstream ss;
  ss << "Observation statistics:";
  for (auto it : message_stats_) {
    ss << std::endl << "  " << it.first << " = " << it.second;
  }
  ROS_INFO_STREAM_THROTTLE(30, ss.str());
}

// Process bearing measurements from bounding box
void Localizer::bearingClbk(const Observation::Ptr& obs) {
  // Check if we have enough data to run this callback
  if (!obs->vision_msg) {
    ROS_WARN("Unable to run bearing callback: not enough data!");
    return;
  }

  // Compute bearing
  obs->camera->getBearingFromBoundingBox(
      obs->vision_msg->box, obs->bearing, obs->bearing_sigma);
  if (!obs->isBearingValid()) {
    ROS_ERROR("Failed to extract bearing");
    return;
  }

  // Add observation
  ROS_DEBUG("Processing new bearing detection for %s",
            obs->artifact->getId().c_str());
  if (obs->artifact->addBearingObservation(*obs)) {
    ROS_DEBUG("Bearing measurements added: %s",
              obs->artifact->getSummaryString().c_str());
  }

  // Publish updated position
  if (obs->artifact->minimalObservationsMade()) {
    publishPositionUpdate(obs->artifact);
  }

  // Debug visualization
  publishRawBearingMeasurement(*obs);
}

void Localizer::lidarRangeClbk(const Observation::Ptr& obs,
                               const sensor_msgs::ImageConstPtr& img) {
  ROS_DEBUG("Running lidar range callback for %s", obs->camera->getName().c_str());

  // Check if we have enough data to run this callback
  if (!latest_lidar_msg_ || !obs->vision_msg) {
    if (!latest_lidar_msg_) {
      ROS_WARN("Unable to run lidar range callback: no lidar message in "
               "observation!");
    }
    if (!obs->vision_msg) {
      ROS_WARN("Unable to run lidar range callback: no vision message in "
               "observation!");
    }
    return;
  }
  ROS_DEBUG("Passed lidar range callback pre-checks for %s", obs->camera->getName().c_str());

  // Transform lidar point cloud to camera frame
  sensor_msgs::PointCloud2Ptr transformed_lidar_msg(
      new sensor_msgs::PointCloud2());
  try {
    geometry_msgs::TransformStamped lidar_to_cam_transform =
        tf_buf_.lookupTransform(obs->vision_msg->header.frame_id,
                                obs->vision_msg->header.stamp,
                                latest_lidar_msg_->header.frame_id,
                                latest_lidar_msg_->header.stamp,
                                config_.odom_frame,
                                ros::Duration(1.0));
    tf2::doTransform(
        *latest_lidar_msg_, *transformed_lidar_msg, lidar_to_cam_transform);
  } catch (tf2::TransformException& ex) {
    ROS_ERROR("Failed to transform LiDAR message to %s frame: %s",
              obs->vision_msg->header.frame_id.c_str(),
              ex.what());
    return;
  }
  // Process raw data to artifact observation
  if (!obs->camera->getRangeFromBoundingBox(obs->vision_msg->box,
                                            transformed_lidar_msg,
                                            *img,
                                            obs->range,
                                            obs->range_sigma)) {
    ROS_ERROR("Failed to extract range from lidar!");
  }

  if (!obs->isRangeValid()) {
    ROS_ERROR("Failed to extract valid range from lidar!");
    return;
  }

  // do the size score
  double range, range_sigma;
  double height, width;
  double size_score = -1.0;
  height = -1.0;
  width = -1.0;
  range = obs->range;
  range_sigma = obs->range_sigma;
  if (!obs->camera->getSizeScore(range,
                                 range_sigma,
                                 obs->vision_msg->box,
                                 height,
                                 width,
                                 size_score)) {
    ROS_WARN("Size Filter unsuccessful");
  }
  obs->addSizeScore(size_score, height, width);

  // Add observation
  if (obs->artifact->addRangeObservation(*obs)) {
    ROS_INFO("New lidar observation added at range %.1f+/-%.1fm from %s: %s",
             obs->range,
             obs->range_sigma,
             obs->camera->getName().c_str(),
             obs->artifact->getSummaryString().c_str());
  }

  // Publish updated position
  if (obs->artifact->minimalObservationsMade()) {
    publishPositionUpdate(obs->artifact);
  }

  // Debug visualization
  publishRawRangeMeasurement(*obs);
  if (!config_.debug_topic.empty() || !config_.debug_dir.empty()) {
    publishLidarDebugImages(*obs);
  }
}

void Localizer::signalClbk(const Observation::Ptr& obs) {
  // Check if we have enough data to run this callback
  if (!obs->signal_msg) {
    ROS_WARN(
        "Unable to run signal callback: no signal message in observation!");
    return;
  }

  // Process raw data to artifact observation
  if (!obs->signal_processor->getRangeFromSignalStrength(
          obs->strength, obs->range, obs->range_sigma)) {
    ROS_ERROR("Failed to estimate range from signal");
    return;
  }

  // Add observation
  ROS_DEBUG("Processing new signal data for %s",
            obs->artifact->getId().c_str());
  if (obs->artifact->addSignalObservation(*obs)) {
    ROS_INFO("New signal observation added from %s: %s",
             obs->detection_source.c_str(),
             obs->artifact->getSummaryString().c_str());
  }

  // Publish updated position
  if (obs->artifact->minimalObservationsMade()) {
    publishPositionUpdate(obs->artifact);
  }
}

// Process range measurements from depth image
void Localizer::depthRangeClbk(const Observation::Ptr& obs,
                               const sensor_msgs::ImageConstPtr& img) {
  // Check if we have enough data to run this callback
  if (!obs->vision_msg) {
    ROS_WARN("Unable to run depth range callback: no vision message in "
             "observation!");
    return;
  }

  // Process raw data to artifact observation
  double range, range_sigma;
  double height, width;
  double size_score = -1.0;
  height = -1.0;
  width = -1.0;

  if (!obs->camera->getRangeFromBoundingBox(
          obs->vision_msg->box, img, obs->range, obs->range_sigma)) {
    ROS_ERROR("Failed to extract range from depth image!");
    return;
  }
  if (!obs->isRangeValid()) {
    ROS_ERROR("Failed to extract valid range from depth image!");
    return;
  }

  range = obs->range;
  range_sigma = obs->range_sigma;
  if (!obs->camera->getSizeScore(range,
                                 range_sigma,
                                 obs->vision_msg->box,
                                 height,
                                 width,
                                 size_score)) {
    ROS_WARN("Size Filter unsuccessful");
  }

  obs->addSizeScore(size_score, height, width);

  // Add observation
  if (obs->artifact->addRangeObservation(*obs)) {
    ROS_INFO("New depth observation added at range %.1f+/-%.1fm from %s: %s",
             obs->range,
             obs->range_sigma,
             obs->camera->getName().c_str(),
             obs->artifact->getSummaryString().c_str());
  }

  // Publish updated position
  if (obs->artifact->minimalObservationsMade()) {
    publishPositionUpdate(obs->artifact);
  }

  // Debug visualization
  publishRawRangeMeasurement(*obs);
}

void Localizer::thumbnailClbk(const Observation::Ptr& obs,
                              const sensor_msgs::ImageConstPtr& img) {
  // Add observation
  ROS_DEBUG("Processing new image/confidence for %s",
            obs->artifact->getId().c_str());
  obs->image = img;
  if (obs->artifact->addImageObservation(*obs)) {
    if (obs->camera) {
      ROS_INFO("New image observation added from %s: %s",
               obs->camera->getName().c_str(),
               obs->artifact->getSummaryString().c_str());
    } else if (!obs->detection_source.empty()) {
      ROS_INFO("New image observation added from %s: %s",
               obs->detection_source.c_str(),
               obs->artifact->getSummaryString().c_str());
    } else {
      ROS_INFO("New image observation added: %s",
               obs->artifact->getSummaryString().c_str());
    }
  }
}

void Localizer::kalmanClbk(const Observation::Ptr& obs,
                           const sensor_msgs::ImageConstPtr& img) {
  ROS_WARN_ONCE("Kalman filter is not supported yet");
}

void Localizer::mslRaptorClbk(const Observation::Ptr& obs,
                              const sensor_msgs::ImageConstPtr& img) {
  ROS_WARN_ONCE("MSL-Raptor is not supported yet");
}

bool Localizer::publishReportsClbk(
    artifact_msgs::PublishReports::Request& req,
    artifact_msgs::PublishReports::Response& res) {
  if (req.require_thumbnail && req.require_enough_observations) {
    ROS_INFO("Publish reports service called: publishing all pending artifact reports with a thumbnail and enough observations.");
  } else if (req.require_thumbnail) {
    ROS_INFO("Publish reports service called: publishing all pending artifact reports with a thumbnail.");
  } else if (req.require_enough_observations) {
    ROS_INFO("Publish reports service called: publishing all pending artifact reports with enough observations.");
  } else {
    ROS_INFO("Publish reports service called: publishing all pending artifact reports.");
  }
  std::vector<std::string> artifact_ids;
  for (auto& artifact : artifacts_) {
    if (!artifact->isPublished()) {
      bool publishable = true;
      if (req.require_thumbnail && !artifact->isThumbnailValid()) {
        publishable = false;
      }
      if (req.require_enough_observations && !artifact->enoughObservationsMade()) {
        publishable = false;
      }
      if (publishable) {
        publishFullReport(artifact);
        artifact_ids.push_back(artifact->getId());
        res.published_artifact_ids.push_back(artifact->getId());
        artifact->setPublished();
      }
    }
  }

  if (artifact_ids.size() > 0) {
    std::string artifact_ids_str = "";
    std::string delim = "";
    for (auto& id : artifact_ids) {
      artifact_ids_str += delim + id;
      delim = ", ";
    }
    ROS_INFO("Publish reports service published %d remaining artifacts: %s",
             artifact_ids.size(),
             artifact_ids_str.c_str());
  } else {
    ROS_INFO("Publish reports service did not publish any remaining reports.");
  }

  return true;
}

bool Localizer::reconcileOrCreateArtifact(const Observation::Ptr& obs) {
  // Already know who you are
  if (obs->artifact) {
    return true;
  }

  // Find corresponding artifact from the database
  if (!artifacts_.empty()) {
    std::vector<double> reconciliation_scores;
    reconciliation_scores.reserve(artifacts_.size());
    for (auto& artifact : artifacts_) {
      reconciliation_scores.push_back(artifact->getReconciliationScore(*obs));
    }

    auto max_score_it = std::max_element(reconciliation_scores.begin(),
                                         reconciliation_scores.end());
    double max_score = *max_score_it;
    double index = max_score_it - reconciliation_scores.begin();
    if (max_score == 0) {
      ROS_WARN("Could not find corresponding entry in the database");
    } else {
      ROS_DEBUG("Reconciled to: %s (Score=%.2f)",
                artifacts_[index]->getId().c_str(),
                max_score);
      obs->artifact = artifacts_[index];
      return true;
    }
  }

  // Check if this is a valid artifact
  if (!hasKey(config_.artifact_configs, obs->label)) {
    ROS_ERROR("Failed to create a report for type %s", obs->label.c_str());
    return false;
  }
  auto artifact_config = config_.artifact_configs.at(obs->label);

  // No matched artifact in the database. Create new artifact
  std::string id_hint;
  if (obs->signal_msg && obs->label != "Gas") {
    id_hint = obs->signal_msg->id;
  }
  auto artifact = std::make_shared<Artifact>(artifact_config, id_hint);

  ROS_INFO("Generated new '%s' artifact with ID=%s",
           artifact->getLabel().c_str(),
           artifact->getId().c_str());

  artifacts_.push_back(artifact);
  obs->artifact = artifact;
  return true;
}

void Localizer::publishFullReport(const Artifact::Ptr& artifact) {
  // update the thumbnail
  if (artifact->createMosaique()) {
    ROS_INFO("Thumbnail created");
  } else {
    ROS_ERROR("Could not create a thumbnail");
  }

  auto msg = artifact->toMessageInMap();
  if (!msg) {
    return;
  }

  // create the scorability
  if (artifact->updateScorability(msg)) {
    ROS_INFO("Scorability updated before publishing artifact!");
  } else {
    ROS_ERROR(
        "Could not updated artifact scorability - will be same as confidence");
    msg->scorability = msg->confidence;
  }

  // publish the artifact
  artifact_pub_.publish(msg);

  // Draw bounding box and publish for debuggnig
  if (thumbnail_pub_.getNumSubscribers()) {
    auto thumbnail = artifact->toAnnotatedImageMessage();
    if (thumbnail) {
      thumbnail_pub_.publish(thumbnail);
    }
  }

  ROS_INFO("Artifact published: %s", artifact->getSummaryString().c_str());
}

void Localizer::publishPositionUpdate(const Artifact::Ptr& artifact) {
  auto msg = artifact->toMessageInBaseLink();
  if (!msg) {
    return;
  }

  msg->thumbnail =
      sensor_msgs::CompressedImage(); // Drop image to reduce traffic
  artifact_update_pub_.publish(msg);

  ROS_DEBUG("Position published: %s", artifact->getSummaryString().c_str());
}

void Localizer::publishRawBearingMeasurement(const Observation& obs) {
  // No need to work if there is no subscribers
  if (raw_meas_pub_.getNumSubscribers() == 0) {
    return;
  }

  // Compose marker message and publish
  auto msg = obs.createBearingMarker();
  if (!msg) {
    return;
  }
  msg->id = bearing_counter_++;
  raw_meas_pub_.publish(msg);
}

void Localizer::publishRawRangeMeasurement(const Observation& obs) {
  // No need to work if there is no subscribers
  if (raw_meas_pub_.getNumSubscribers() == 0) {
    return;
  }

  // Compose marker message and publish
  auto msg = obs.createRangeMarker();
  if (!msg) {
    return;
  }
  msg->id = range_counter_++;
  raw_meas_pub_.publish(msg);
}

void Localizer::publishLidarDebugImages(const Observation& obs) {
  // Publish debug images
  for (auto const& img_name : obs.camera->getDebugImageNames()) {
    std::string cam_name;
    cv::Mat img;
    try {
      cam_name = obs.camera->getName();
      auto pub = debug_image_pubs_[cam_name][img_name];
      img = obs.camera->getDebugImage(img_name);
      auto enc = obs.camera->getDebugImageEncoding(img_name);
      if (pub.getNumSubscribers() > 0) {
        sensor_msgs::ImagePtr msg =
            cv_bridge::CvImage(std_msgs::Header(), enc, img).toImageMsg();
        msg->header.frame_id = obs.camera->getFrame();
        msg->header.stamp = obs.stamp;
        pub.publish(msg);
      }
    } catch (const std::exception& ex) {
      ROS_ERROR("Failed to publish debug image %s for camera %s: %s",
                img_name.c_str(),
                cam_name.c_str(),
                ex.what());
    }

    // Output debug image files to debug directory
    if (!img.empty() && !config_.debug_dir.empty()) {
      std::string img_filename = config_.debug_dir + "/" + cam_name + "_" +
          img_name + "_" + std::to_string(obs.stamp.sec) + ".png";
      try {
        cv::imwrite(img_filename, img);
        ROS_DEBUG("Successfully saved %s debug image for camera %s to file %s",
                  img_name.c_str(),
                  cam_name.c_str(),
                  img_filename.c_str());
      } catch (const std::exception& ex) {
        ROS_ERROR("Failed to save %s debug image for camera %s to file %s: %s",
                  img_name.c_str(),
                  cam_name.c_str(),
                  img_filename.c_str(),
                  ex.what());
      }
    }
  }
}

bool Localizer::getTransformEigenFromTf(const std::string& parent_frame,
                                        const std::string& child_frame,
                                        const ros::Time& stamp,
                                        Eigen::Affine3d& T,
                                        const ros::Duration& timeout) const {
  // Test if TF is available
  if (!tf_buf_.canTransform(parent_frame, child_frame, ros::Time(0))) {
    ROS_ERROR("TF from %s to %s not available",
              parent_frame.c_str(),
              child_frame.c_str());
    return false;
  }

  // Get transform
  geometry_msgs::TransformStamped transform;
  try {
    transform =
        tf_buf_.lookupTransform(parent_frame, child_frame, stamp, timeout);
  } catch (tf2::TransformException& ex) {
    ROS_ERROR("Transform error: %s", ex.what());
    return false;
  }
  T = tf2::transformToEigen(transform);
  return true;
}

bool Localizer::hasMovedEnough(const std::string& label,
                               const Eigen::Affine3d& map_T_base_link) {
  if (!hasKey(last_observation_pose_, label)) {
    last_observation_pose_[label] = map_T_base_link;
    return true;
  }

  double dist = (map_T_base_link.translation() -
                 last_observation_pose_[label].translation())
                    .norm();
  if (dist < config_.min_translation_between_observations) {
    return false;
  }

  last_observation_pose_[label] = map_T_base_link;
  return true;
}
