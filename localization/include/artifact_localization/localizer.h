#pragma once

#include <deque>
#include <gtest/gtest.h>
#include <unordered_map>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/pass_through.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <artifact_msgs/PointSourceDetection.h>
#include <darknet_ros_msgs/Object.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <artifact_msgs/PublishReports.h>

#include "artifact_localization/artifact.h"
#include "artifact_localization/camera.h"
#include "artifact_localization/config.h"
#include "artifact_localization/observation.h"
#include "artifact_localization/signal_processor.h"

class ArtifactLocalizationFixtureTest;

class Localizer {
  friend class ArtifactLocalizationFixtureTest;
  FRIEND_TEST(ArtifactLocalizationFixtureTest, cubeBTSignalDetectionClbkTest1);
  FRIEND_TEST(ArtifactLocalizationFixtureTest, cubeVisionDetectionClbkTest1);
  FRIEND_TEST(ArtifactLocalizationFixtureTest, cubeBTReconcileOrCreateTest1);
  FRIEND_TEST(ArtifactLocalizationFixtureTest,
              cubeVisionReconcileOrCreateTest1);
  FRIEND_TEST(ArtifactLocalizationFixtureTest, cubeBTSignalClbkTest1);
  FRIEND_TEST(ArtifactLocalizationFixtureTest, cubeThumbnailClbkTest1);
  FRIEND_TEST(ArtifactLocalizationFixtureTest, cubeBearingClbkTest1);
  FRIEND_TEST(ArtifactLocalizationFixtureTest, cubeBTPublishArtifactClbkTest1);
  FRIEND_TEST(ArtifactLocalizationFixtureTest,
              cubeVisionPublishArtifactClbkTest1);

  typedef artifact_msgs::PointSourceDetection PointSourceDetection;

public:
  // clang-format off
  typedef message_filters::PassThrough<sensor_msgs::Image> ImageBuffer;
  typedef std::shared_ptr<ImageBuffer> ImageBufferPtr;
  typedef std::unordered_map<std::string, ImageBufferPtr> ImageBufferMap;

  typedef message_filters::PassThrough<Observation> ObservationBuffer;
  typedef std::shared_ptr<ObservationBuffer> ObservationBufferPtr;
  typedef std::unordered_map<std::string, ObservationBufferPtr> ObservationBufferMap;

  typedef message_filters::sync_policies::ApproximateTime<Observation, sensor_msgs::Image> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> ObservationImageSync;
  typedef std::shared_ptr<ObservationImageSync> ObservationImageSyncPtr;
  typedef std::unordered_map<std::string, ObservationImageSyncPtr> ObservationImageSyncMap;

  typedef std::unordered_map<std::string, ros::Publisher> PubMap;
  typedef std::unordered_map<std::string, PubMap> CamPubMap;
  // clang-format on

  Localizer(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  ~Localizer();

  void run();

private:
  // Initialization helpers
  void loadParameters();
  void initializePublishers();
  void initializeSubscribers();
  void initializeQueues();
  void initializeServices();
  Camera::Ptr createCameraModel(const std::string& camera_name,
                                const std::string& info_topic);

  // Worker functions (call either synchronously or as a timer callback
  void setupSensorClbk(const ros::TimerEvent& e = ros::TimerEvent());
  void publishArtifactClbk(const ros::TimerEvent& e = ros::TimerEvent());

  // Message callbacks
  void rgbClbk(const sensor_msgs::ImageConstPtr& msg,
               const std::string& camera_name);
  void depthClbk(const sensor_msgs::ImageConstPtr& msg,
                 const std::string& camera_name);
  void thermalClbk(const sensor_msgs::ImageConstPtr& msg,
                   const std::string& camera_name);
  void lidarClbk(const sensor_msgs::PointCloud2ConstPtr& msg);

  void visionDetectionClbk(const darknet_ros_msgs::ObjectConstPtr& msg);
  void signalDetectionClbk(const PointSourceDetection::ConstPtr& msg,
                           const std::string& label,
                           const std::string& detection_source);
  bool processDetection(const Observation::Ptr& obs);

  // Measurement processing callbacks
  void observationStatsClbk(const Observation::Ptr& obs,
                            const std::string& category);
  void bearingClbk(const Observation::Ptr& obs);
  void lidarRangeClbk(const Observation::Ptr& obs,
                      const sensor_msgs::ImageConstPtr& img);
  void lidarRangeClbk(const Observation::Ptr& obs) {
    sensor_msgs::ImagePtr empty_img(new sensor_msgs::Image());
    lidarRangeClbk(obs, empty_img);
  }
  void signalClbk(const Observation::Ptr& obs);
  void depthRangeClbk(const Observation::Ptr& obs,
                      const sensor_msgs::ImageConstPtr& img);
  void thumbnailClbk(const Observation::Ptr& obs,
                     const sensor_msgs::ImageConstPtr& img);
  void kalmanClbk(const Observation::Ptr& obs,
                  const sensor_msgs::ImageConstPtr& img);
  void mslRaptorClbk(const Observation::Ptr& obs,
                     const sensor_msgs::ImageConstPtr& img);

  // Service callbacks
  bool publishReportsClbk(artifact_msgs::PublishReports::Request& req,
                          artifact_msgs::PublishReports::Response& res);

  // Find corresponding artifact
  bool reconcileOrCreateArtifact(const Observation::Ptr& obs);

  // Publish artifacts
  void publishFullReport(const Artifact::Ptr& artifact);
  void publishPositionUpdate(const Artifact::Ptr& artifact);
  void publishRawBearingMeasurement(const Observation& obs);
  void publishRawRangeMeasurement(const Observation& obs);
  void publishLidarDebugImages(const Observation& obs);

  // Transform utils
  bool getTransformEigenFromTf(
      const std::string& parent_frame,
      const std::string& child_frame,
      const ros::Time& stamp,
      Eigen::Affine3d& T,
      const ros::Duration& timeout = ros::Duration(0)) const;

  bool hasMovedEnough(const std::string& label,
                      const Eigen::Affine3d& map_T_base_link);

  // Helper function for std container
  template <typename Key, typename Value>
  bool hasKey(const std::unordered_map<Key, Value>& map, const Key& key) {
    return map.find(key) != map.end();
  }

  // ROS handlers
  ros::NodeHandle nh_, pnh_;
  image_transport::ImageTransport it_;

  // Subscribers
  std::unordered_map<std::string, ros::Subscriber> subs_;
  std::unordered_map<std::string, image_transport::Subscriber> it_subs_;

  tf2_ros::TransformListener tf_listener_;
  tf2_ros::Buffer tf_buf_;

  // Publishers
  ros::Publisher artifact_pub_;
  ros::Publisher artifact_update_pub_;
  ros::Publisher thumbnail_pub_;
  ros::Publisher detection_report_pub_;
  ros::Publisher raw_meas_pub_;
  CamPubMap debug_image_pubs_;

  // Services
  ros::ServiceServer publish_reports_srv_;

  // Message container
  sensor_msgs::PointCloud2ConstPtr latest_lidar_msg_;

  ImageBufferMap rgb_bufs_;
  ImageBufferMap depth_bufs_;
  ImageBufferMap thermal_bufs_;
  ObservationBufferMap obs_bufs_;
  ObservationBufferMap signal_bufs_;
  ObservationImageSyncMap obs_image_bufs_;
  ObservationImageSyncMap obs_depth_bufs_;
  ObservationImageSyncMap signal_image_bufs_;

  std::unordered_map<std::string, int> message_stats_;

  // Artifact container
  std::vector<Artifact::Ptr> artifacts_;

  // Sensor models
  std::unordered_map<std::string, Camera::Ptr> cameras_;
  std::unordered_map<std::string, SignalProcessor::Ptr> signal_processors_;

  // Configurations
  LocalizerConfig config_;

  // Misc
  std::unordered_map<std::string, Eigen::Affine3d> last_observation_pose_;
  int bearing_counter_ = 0;
  int range_counter_ = 0;
};
