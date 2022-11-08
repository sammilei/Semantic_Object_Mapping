/**
 *  @brief Pipeline/integration test cases for artifact_localization
 *
 *  This file contains primarily pipeline/integration tests for
 * artifact_localization.
 */

#include "test_artifact_localization_common.cpp"

TEST_F(ArtifactLocalizationFixtureTest, cubeBTDetectionTest1) {
  ros::Duration timeout(0.5);
  std::string cube_detection_topic = robot_namespace_nodes + "/bt_rssi/cube";
  auto cube_detection_msg =
      ros::topic::waitForMessage<artifact_msgs::PointSourceDetection>(
          cube_detection_topic, timeout);

  EXPECT_GE(cube_detection_msg->header.stamp.sec, 0);
  EXPECT_GE(cube_detection_msg->header.stamp.nsec, 0);
  EXPECT_STREQ(cube_detection_msg->header.frame_id.c_str(), "world");
  EXPECT_STREQ(cube_detection_msg->id.substr(0, 4).c_str(), "Cube");
  EXPECT_FLOAT_EQ(cube_detection_msg->strength, -39.0);
}

TEST_F(ArtifactLocalizationFixtureTest, cubeVisionDetectionTest1) {
  ros::Duration timeout(0.5);
  std::string cube_detection_topic =
      robot_namespace_nodes + "/detected_object/cube";
  auto cube_detection_msg =
      ros::topic::waitForMessage<darknet_ros_msgs::Object>(cube_detection_topic,
                                                           timeout);

  EXPECT_GE(cube_detection_msg->header.stamp.sec, 0);
  EXPECT_GE(cube_detection_msg->header.stamp.nsec, 0);
  EXPECT_STREQ(cube_detection_msg->header.frame_id.c_str(), "world");
  EXPECT_STREQ(cube_detection_msg->box.Class.c_str(), "Cube");
  EXPECT_FLOAT_EQ(cube_detection_msg->box.probability, 1.0);
  EXPECT_FLOAT_EQ(cube_detection_msg->box.yolo_probability, 1.0);
  EXPECT_FLOAT_EQ(cube_detection_msg->box.color_score, 1.0);
}

TEST_F(ArtifactLocalizationFixtureTest, cubeBTSignalDetectionClbkTest1) {
  ros::Duration timeout(0.5);
  std::string cube_detection_topic = robot_namespace_nodes + "/bt_rssi/cube";
  auto cube_detection_msg =
      ros::topic::waitForMessage<artifact_msgs::PointSourceDetection>(
          cube_detection_topic, timeout);

  l_->signalDetectionClbk(cube_detection_msg, "Bluetooth", "bluetooth");
}

TEST_F(ArtifactLocalizationFixtureTest, cubeVisionDetectionClbkTest1) {
  ros::Duration timeout(0.5);
  std::string cube_detection_topic =
      robot_namespace_nodes + "/detected_object/cube";
  auto cube_detection_msg =
      ros::topic::waitForMessage<darknet_ros_msgs::Object>(cube_detection_topic,
                                                           timeout);

  l_->visionDetectionClbk(cube_detection_msg);
}

TEST_F(ArtifactLocalizationFixtureTest, cubeBTObservationTest1) {
  ros::Duration timeout(0.5);
  std::string cube_detection_topic = robot_namespace_nodes + "/bt_rssi/cube";
  auto cube_detection_msg =
      ros::topic::waitForMessage<artifact_msgs::PointSourceDetection>(
          cube_detection_topic, timeout);
  auto cube_obs = boost::make_shared<Observation>(cube_detection_msg);

  EXPECT_STREQ(cube_obs->label.c_str(), "Cube");
}

TEST_F(ArtifactLocalizationFixtureTest, cubeVisionObservationTest1) {
  ros::Duration timeout(0.5);
  std::string cube_detection_topic =
      robot_namespace_nodes + "/detected_object/cube";
  auto cube_detection_msg =
      ros::topic::waitForMessage<darknet_ros_msgs::Object>(cube_detection_topic,
                                                           timeout);
  auto cube_obs = boost::make_shared<Observation>(cube_detection_msg);

  EXPECT_STREQ(cube_obs->label.c_str(), "Cube");
}

TEST_F(ArtifactLocalizationFixtureTest, cubeBTReconcileOrCreateTest1) {
  ros::Duration timeout(0.5);
  std::string cube_detection_topic = robot_namespace_nodes + "/bt_rssi/cube";
  auto cube_detection_msg =
      ros::topic::waitForMessage<artifact_msgs::PointSourceDetection>(
          cube_detection_topic, timeout);
  auto cube_obs = boost::make_shared<Observation>(cube_detection_msg);

  EXPECT_FALSE(cube_obs->artifact);
  l_->reconcileOrCreateArtifact(cube_obs);
  EXPECT_TRUE(cube_obs->artifact);
}

TEST_F(ArtifactLocalizationFixtureTest, cubeVisionReconcileOrCreateTest1) {
  ros::Duration timeout(0.5);
  std::string cube_detection_topic =
      robot_namespace_nodes + "/detected_object/cube";
  auto cube_detection_msg =
      ros::topic::waitForMessage<darknet_ros_msgs::Object>(cube_detection_topic,
                                                           timeout);
  auto cube_obs = boost::make_shared<Observation>(cube_detection_msg);

  EXPECT_FALSE(cube_obs->artifact);
  l_->reconcileOrCreateArtifact(cube_obs);
  EXPECT_TRUE(cube_obs->artifact);
}

TEST_F(ArtifactLocalizationFixtureTest, cubeBTSignalClbkTest1) {
  ros::Duration timeout(0.5);
  std::string cube_detection_topic = robot_namespace_nodes + "/bt_rssi/cube";
  auto cube_detection_msg =
      ros::topic::waitForMessage<artifact_msgs::PointSourceDetection>(
          cube_detection_topic, timeout);
  auto cube_obs = boost::make_shared<Observation>(cube_detection_msg);

  l_->reconcileOrCreateArtifact(cube_obs);

  // Fake checks and calculations from Localizer::signalDetectionClbk()
  cube_obs->signal_processor = l_->signal_processors_.at("bluetooth");
  cube_obs->strength = cube_obs->signal_processor->filterMovingAverage(
      cube_detection_msg->strength);
  cube_obs->signal_processor->getConfidenceFromSignalStrength(
      cube_obs->strength, cube_obs->confidence);

  auto cube_sigma_before = cube_obs->artifact->getLargestSigma();
  l_->signalClbk(cube_obs);
  auto cube_sigma_after = cube_obs->artifact->getLargestSigma();
  EXPECT_LE(cube_sigma_after, cube_sigma_before);
}

TEST_F(ArtifactLocalizationFixtureTest, cubeThumbnailClbkTest1) {
  ros::Duration timeout(0.5);
  std::string cube_detection_topic =
      robot_namespace_nodes + "/detected_object/cube";
  auto cube_detection_msg =
      ros::topic::waitForMessage<darknet_ros_msgs::Object>(cube_detection_topic,
                                                           timeout);
  auto cube_obs = boost::make_shared<Observation>(cube_detection_msg);

  l_->reconcileOrCreateArtifact(cube_obs);

  // Replicate the camera assignment from Localizer::visionDetectionClbk()
  cube_obs->camera = std::make_shared<Camera>(*c_);

  // Create a fake cube image message to pass to Localizer::thumbnailClbk()
  cv::Mat cube_img = cv::Mat(100, 100, CV_32FC3, cv::Scalar(0, 0, 0));
  sensor_msgs::ImagePtr cube_img_msg =
    cv_bridge::CvImage(std_msgs::Header(), "bgr8", cube_img).toImageMsg();

  auto cube_sigma_before = cube_obs->artifact->getLargestSigma();
  l_->thumbnailClbk(cube_obs, cube_img_msg);
  auto cube_sigma_after = cube_obs->artifact->getLargestSigma();
  EXPECT_LE(cube_sigma_after, cube_sigma_before);
}

TEST_F(ArtifactLocalizationFixtureTest, cubeBearingClbkTest1) {
  ros::Duration timeout(0.5);
  std::string cube_detection_topic =
      robot_namespace_nodes + "/detected_object/cube";
  auto cube_detection_msg =
      ros::topic::waitForMessage<darknet_ros_msgs::Object>(cube_detection_topic,
                                                           timeout);
  auto cube_obs = boost::make_shared<Observation>(cube_detection_msg);

  l_->reconcileOrCreateArtifact(cube_obs);

  // Replicate the camera assignment from Localizer::visionDetectionClbk()
  cube_obs->camera = std::make_shared<Camera>(*c_);

  auto cube_sigma_before = cube_obs->artifact->getLargestSigma();
  l_->bearingClbk(cube_obs);
  auto cube_sigma_after = cube_obs->artifact->getLargestSigma();
  EXPECT_LE(cube_sigma_after, cube_sigma_before);
}

TEST_F(ArtifactLocalizationFixtureTest, cubeBTPublishArtifactClbkTest1) {
  ros::Duration timeout(0.5);
  std::string cube_detection_topic = robot_namespace_nodes + "/bt_rssi/cube";
  auto cube_detection_msg =
      ros::topic::waitForMessage<artifact_msgs::PointSourceDetection>(
          cube_detection_topic, timeout);
  auto cube_obs = boost::make_shared<Observation>(cube_detection_msg);

  // Fake checks and calculations from Localizer::signalDetectionClbk()
  cube_obs->signal_processor = l_->signal_processors_.at("bluetooth");
  cube_obs->strength = cube_obs->signal_processor->filterMovingAverage(
      cube_detection_msg->strength);
  cube_obs->signal_processor->getConfidenceFromSignalStrength(
      cube_obs->strength, cube_obs->confidence);

  // Fake color, yolo & size confidence for scorability checks
  cube_obs->color_confidence = 1.0;
  cube_obs->yolo_confidence = 1.0;
  cube_obs->size_confidence = 1.0;

  // Create artifact from cube observation
  l_->reconcileOrCreateArtifact(cube_obs);

  // Fake valid thumbnails to bypass the artifact->isThumbnailValid() check
  // in Localizer::publishArtifactClbk()
  cv::Mat fake_thumbnail = cv::Mat(100, 100, CV_32FC3, cv::Scalar(0, 0, 0));
  sensor_msgs::CompressedImagePtr fake_thumbnail_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", fake_thumbnail)
          .toCompressedImageMsg();
  cube_obs->artifact->thumbnail_ = fake_thumbnail_msg;
  cube_obs->artifact->thumbnail_closest_ = fake_thumbnail_msg;
  cube_obs->artifact->thumbnail_last_ = fake_thumbnail_msg;
  cube_obs->artifact->thumbnail_brightest_ = fake_thumbnail_msg;
  cube_obs->artifact->thumbnail_best_ = fake_thumbnail_msg;
  cube_obs->artifact->thumbnail_less_blurry_ = fake_thumbnail_msg;

  // Update scorability stats
  // TODO: It seems that Artifact::UpdateConfidence() only gets called by
  // Artifact::addBearingObservation(). Should it also be called by, e.g.
  // Artifact::addSignalObservation()?
  cube_obs->artifact->updateConfidence(*cube_obs);

  // Run the observation callback
  l_->signalClbk(cube_obs);

  // Fake covariance & confidence to simulate multiple detections/observations
  // TODO: Perhaps there is a more realistic way to simulate this?
  cube_obs->artifact->covariance_ = Eigen::Matrix3d::Zero();
  cube_obs->artifact->data_.confidence = 1.0;

  // Ensure cube artifact passes each check defined in
  // Localizer::publishArtifactClbk()
  EXPECT_TRUE(!cube_obs->artifact->isPublished());
  EXPECT_TRUE(cube_obs->artifact->isPositionValid());
  EXPECT_TRUE(cube_obs->artifact->minimalObservationsMade());
  EXPECT_LE(cube_obs->artifact->getLargestSigma(),
            cube_obs->artifact->config_.sigma_min);
  EXPECT_GE(cube_obs->artifact->data_.confidence,
            cube_obs->artifact->config_.confidence_max);
  EXPECT_TRUE(cube_obs->artifact->enoughObservationsMade());
  EXPECT_TRUE(cube_obs->artifact->isThumbnailValid());

  // Publish cube artifact
  l_->publishArtifactClbk();
  EXPECT_TRUE(cube_obs->artifact->isPublished());
}

TEST_F(ArtifactLocalizationFixtureTest, cubeVisionPublishArtifactClbkTest1) {
  ros::Duration timeout(0.5);
  std::string cube_detection_topic =
      robot_namespace_nodes + "/detected_object/cube";
  auto cube_detection_msg =
      ros::topic::waitForMessage<darknet_ros_msgs::Object>(cube_detection_topic,
                                                           timeout);
  auto cube_obs = boost::make_shared<Observation>(cube_detection_msg);

  // Replicate the camera assignment from Localizer::visionDetectionClbk()
  cube_obs->camera = std::make_shared<Camera>(*c_);

  // Create artifact from cube observation
  l_->reconcileOrCreateArtifact(cube_obs);

  // Fake valid thumbnails to bypass the artifact->isThumbnailValid() check
  // in Localizer::publishArtifactClbk()
  cv::Mat fake_thumbnail = cv::Mat(100, 100, CV_32FC3, cv::Scalar(0, 0, 0));
  sensor_msgs::CompressedImagePtr fake_thumbnail_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", fake_thumbnail)
          .toCompressedImageMsg();
  cube_obs->artifact->thumbnail_ = fake_thumbnail_msg;
  cube_obs->artifact->thumbnail_closest_ = fake_thumbnail_msg;
  cube_obs->artifact->thumbnail_last_ = fake_thumbnail_msg;
  cube_obs->artifact->thumbnail_brightest_ = fake_thumbnail_msg;
  cube_obs->artifact->thumbnail_best_ = fake_thumbnail_msg;
  cube_obs->artifact->thumbnail_less_blurry_ = fake_thumbnail_msg;

  // Update scorability stats
  // TODO: It seems that Artifact::UpdateConfidence() only gets called by
  // Artifact::addBearingObservation(). Should it also be called by, e.g.
  // Artifact::addSignalObservation()?
  cube_obs->artifact->updateConfidence(*cube_obs);

  // Run the observation callback
  l_->bearingClbk(cube_obs);

  // Fake covariance & confidence to simulate multiple
  // detections/observations
  // TODO: Perhaps there is a more realistic way to simulate this?
  cube_obs->artifact->covariance_ = Eigen::Matrix3d::Zero();
  cube_obs->artifact->data_.confidence = 1.0;

  // Ensure cube artifact passes each check defined in
  // Localizer::publishArtifactClbk()
  EXPECT_TRUE(!cube_obs->artifact->isPublished());
  EXPECT_TRUE(cube_obs->artifact->isPositionValid());
  EXPECT_TRUE(cube_obs->artifact->minimalObservationsMade());
  EXPECT_LE(cube_obs->artifact->getLargestSigma(),
            cube_obs->artifact->config_.sigma_min);
  EXPECT_GE(cube_obs->artifact->data_.confidence,
            cube_obs->artifact->config_.confidence_max);
  EXPECT_TRUE(cube_obs->artifact->enoughObservationsMade());
  EXPECT_TRUE(cube_obs->artifact->isThumbnailValid());

  // Publish cube artifact
  l_->publishArtifactClbk();
  EXPECT_TRUE(cube_obs->artifact->isPublished());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_artifact_localization_pipeline");
  return RUN_ALL_TESTS();
}
