#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <ros/message_traits.h>
#include <ros/ros.h>

#include <artifact_msgs/PointSourceDetection.h>
#include <artifact_msgs/ScorabilityMetrics.h>
#include <darknet_ros_msgs/Object.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>

#include "artifact_localization/camera.h"
#include "artifact_localization/signal_processor.h"

class Artifact;

// Generic observation class that summaries all possible detection types
struct Observation {
  typedef boost::shared_ptr<Observation> Ptr;
  typedef boost::shared_ptr<Observation const> ConstPtr;

  typedef std::shared_ptr<Artifact> ArtifactPtr;
  typedef std::shared_ptr<Camera> CameraPtr;
  typedef std::shared_ptr<SignalProcessor> SignalProcessorPtr;

  Observation();
  Observation(const artifact_msgs::PointSourceDetectionConstPtr& msg);
  Observation(const darknet_ros_msgs::ObjectConstPtr& msg);

  bool isObservationValid() const {
    return !label.empty() && stamp.isValid();
  }
  bool isGoodDetection() const {
    if (!isObservationValid()) {
      return false;
    }

    if (vision_msg) {
      return camera->isDetectionValid(vision_msg);
    } else if (signal_msg) {
      return signal_processor->isDetectionValid(signal_msg);
    }

    return false;
  }

  bool isRangeValid() const {
    return range > 0 && range_sigma >= 0;
  }
  bool isBearingValid() const {
    return !bearing.array().isNaN().any() && bearing_sigma >= 0;
  }
  bool isImageValid() const {
    return image != NULL;
  }

  void addSizeScore(double confidence, double obs_height, double obs_width) {
    size_confidence = confidence;
    height = obs_height;
    width = obs_width;
  }

  visualization_msgs::MarkerPtr createRangeMarker() const;
  visualization_msgs::MarkerPtr createBearingMarker() const;

  // Detection timestamp
  ros::Time stamp;

  // Frame IDs
  std::string map_frame;
  std::string base_link_frame;
  std::string sensor_frame;

  // Type of artifact (DARPA-defined)
  std::string label;

  // Confidence of detection
  double confidence = -1;
  double yolo_confidence = -1;
  double color_confidence = -1;
  double size_confidence = -1;
  double width = -1;
  double height = -1;

  // Source of detection (e.g., camera_front, wifi)
  std::string detection_source;

  // Signal strength for signal-based artifacts
  double strength;

  // image of the detection
  sensor_msgs::Image rgb_image;

  // Pose of base link and sensor at the time of detection
  Eigen::Affine3d map_T_base_link;
  Eigen::Affine3d map_T_sensor;

  // Sensor model
  CameraPtr camera;
  SignalProcessorPtr signal_processor;

  // Corresponding artifact
  ArtifactPtr artifact;

  // Raw messages
  darknet_ros_msgs::ObjectConstPtr vision_msg;
  artifact_msgs::PointSourceDetectionConstPtr signal_msg;

  // Bearing measurements
  Eigen::Vector3d bearing = Eigen::Vector3d::Constant(NAN);
  double bearing_sigma = -1;

  // Range measurements
  double range = -1;
  double range_sigma = -1;

  // Image measurements
  sensor_msgs::ImageConstPtr image;
};

// Need to implement this to use approx time sync with custom class
namespace ros {
namespace message_traits {

template <>
struct TimeStamp<Observation> {
  static ros::Time* pointer(Observation& m) {
    return &m.stamp;
  }
  static ros::Time const* pointer(const Observation& m) {
    return &m.stamp;
  }
  static ros::Time value(const Observation& m) {
    return m.stamp;
  }
};

} // namespace message_traits
} // namespace ros
