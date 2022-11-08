/**
 *  @brief Unit test cases for artifact_localization
 *
 *  This file contains primarily unit tests for artifact_localization.
 */

#include "test_artifact_localization_common.cpp"

TEST_F(ArtifactLocalizationFixtureTest, getRangeFromBoundingBoxTest1) {
  // Create bounding box
  darknet_ros_msgs::BoundingBox bbox;
  bbox.Class = "Survivor";
  bbox.xmin = 162;
  bbox.ymin = 70;
  bbox.xmax = 262;
  bbox.ymax = 170;

  // Create lidar message with points inside bounding box
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
  PointCloud::Ptr lidar_pc(new PointCloud);
  lidar_pc->points.push_back(pcl::PointXYZ(0.0, 0.0, 1.0));
  lidar_pc->points.push_back(pcl::PointXYZ(0.0, 0.1, 1.0));
  lidar_pc->points.push_back(pcl::PointXYZ(0.1, 0.0, 1.0));
  lidar_pc->points.push_back(pcl::PointXYZ(0.1, 0.1, 1.0));
  sensor_msgs::PointCloud2Ptr lidar_msg(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*lidar_pc, *lidar_msg);
  lidar_msg->header.frame_id = robot_namespace_frames + "/velodyne";

  double range;
  double range_sigma;
  sensor_msgs::ImagePtr empty_img(new sensor_msgs::Image());
  EXPECT_TRUE(c_->getRangeFromBoundingBox(
      bbox, lidar_msg, *empty_img, range, range_sigma));
  EXPECT_FLOAT_EQ(range, 1.0);
  EXPECT_FLOAT_EQ(range_sigma, 0.8);
}

TEST_F(ArtifactLocalizationFixtureTest, getRangeFromBoundingBoxTest2) {
  // Create bounding box
  darknet_ros_msgs::BoundingBox bbox;
  bbox.Class = "Survivor";
  bbox.xmin = 162;
  bbox.ymin = 70;
  bbox.xmax = 262;
  bbox.ymax = 170;

  // Create lidar message with points inside bounding box
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
  PointCloud::Ptr lidar_pc(new PointCloud);
  lidar_pc->points.push_back(pcl::PointXYZ(0.0, 0.0, 1.0));
  lidar_pc->points.push_back(pcl::PointXYZ(0.0, 0.1, 1.0));
  lidar_pc->points.push_back(pcl::PointXYZ(0.1, 0.0, 1.0));
  lidar_pc->points.push_back(pcl::PointXYZ(0.1, 0.1, 1.0));
  sensor_msgs::PointCloud2Ptr lidar_msg(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*lidar_pc, *lidar_msg);
  lidar_msg->header.frame_id = robot_namespace_frames + "/velodyne";

  c_->config_.foreground_depth_percentiles.close_percentile = 0.0;
  c_->config_.foreground_depth_percentiles.far_percentile = 1.0;

  double range;
  double range_sigma;
  sensor_msgs::ImagePtr empty_img(new sensor_msgs::Image());
  EXPECT_TRUE(c_->getRangeFromBoundingBox(
      bbox, lidar_msg, *empty_img, range, range_sigma));
  EXPECT_FLOAT_EQ(range, 1.0);
  EXPECT_FLOAT_EQ(range_sigma, 0.8);
}

TEST_F(ArtifactLocalizationFixtureTest, getRangeFromBoundingBoxTest3) {
  // Create bounding box
  darknet_ros_msgs::BoundingBox bbox;
  bbox.Class = "Survivor";
  bbox.xmin = 162;
  bbox.ymin = 70;
  bbox.xmax = 262;
  bbox.ymax = 170;

  // Create lidar message with points inside bounding box at different distances
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
  PointCloud::Ptr lidar_pc(new PointCloud);
  lidar_pc->points.push_back(pcl::PointXYZ(0.0, 0.0, 1.0));
  lidar_pc->points.push_back(pcl::PointXYZ(0.0, 0.1, 2.0));
  lidar_pc->points.push_back(pcl::PointXYZ(0.1, 0.0, 3.0));
  lidar_pc->points.push_back(pcl::PointXYZ(0.1, 0.1, 4.0));
  sensor_msgs::PointCloud2Ptr lidar_msg(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*lidar_pc, *lidar_msg);
  lidar_msg->header.frame_id = robot_namespace_frames + "/velodyne";

  c_->config_.foreground_depth_percentiles.close_percentile = 0.0;
  c_->config_.foreground_depth_percentiles.far_percentile = 1.0;
  c_->config_.foreground_depth_nn_dist_to_median = 2.0;

  double range;
  double range_sigma;
  sensor_msgs::ImagePtr empty_img(new sensor_msgs::Image());
  EXPECT_TRUE(c_->getRangeFromBoundingBox(
      bbox, lidar_msg, *empty_img, range, range_sigma));
  // Foreground percentiles: [0.0, 1.0]
  // Depth points in [0.0, 1.0] percentile range: [1.0, 2.0, 3.0, 4.0]
  // Median of points in [0.0, 1.0] percentile range: 2.5
  // Foreground points = Nearest neighbours within 2.0m radius of 2.5:
  // [1.0, 2.0, 3.0, 4.0] Survivor percentiles: [0.15, 0.3] Closest foreground
  // points in [0.15, 0.3] percentile range: [1.0, 2.0] range = Mean of closest
  // foreground points in [0.15, 0.3] percentile range: 1.5 range_sigma = coeff0
  // + coeff1 * range = 0.4 + 0.4 * 2.5 = 1.0
  EXPECT_FLOAT_EQ(range, 1.5);
  EXPECT_FLOAT_EQ(range_sigma, 1.0);
}

TEST_F(ArtifactLocalizationFixtureTest, getRangeFromBoundingBoxTest4) {
  // Create bounding box
  darknet_ros_msgs::BoundingBox bbox;
  bbox.Class = "Survivor";
  bbox.xmin = 162;
  bbox.ymin = 70;
  bbox.xmax = 262;
  bbox.ymax = 170;

  // Create lidar message with points inside bounding box at different distances
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
  PointCloud::Ptr lidar_pc(new PointCloud);
  lidar_pc->points.push_back(pcl::PointXYZ(0.0, 0.0, 1.0));
  lidar_pc->points.push_back(pcl::PointXYZ(0.0, 0.1, 2.0));
  lidar_pc->points.push_back(pcl::PointXYZ(0.1, 0.0, 3.0));
  lidar_pc->points.push_back(pcl::PointXYZ(0.1, 0.1, 4.0));
  sensor_msgs::PointCloud2Ptr lidar_msg(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*lidar_pc, *lidar_msg);
  lidar_msg->header.frame_id = robot_namespace_frames + "/velodyne";

  c_->config_.foreground_depth_percentiles.close_percentile = 0.0;
  c_->config_.foreground_depth_percentiles.far_percentile = 1.0;
  c_->config_.foreground_depth_nn_dist_to_median = 0.5;

  double range;
  double range_sigma;
  sensor_msgs::ImagePtr empty_img(new sensor_msgs::Image());
  EXPECT_TRUE(c_->getRangeFromBoundingBox(
      bbox, lidar_msg, *empty_img, range, range_sigma));
  // Foreground percentiles: [0.0, 1.0]
  // Depth points in [0.0, 1.0] percentile range: [1.0, 2.0, 3.0, 4.0]
  // Median of points in [0.0, 1.0] percentile range: 2.5
  // Foreground points = Nearest neighbours within 0.5m radius of 2.5:
  // [2.0, 3.0] Survivor percentiles: [0.15, 0.3] Closest foreground points in
  // [0.15, 0.3] percentile range: [2.0] range = Mean of closest foreground
  // points in [0.15, 0.3] percentile range: 2.0 range_sigma = coeff0 + coeff1 *
  // range = 0.4 + 0.4 * 2.0 = 1.2
  EXPECT_FLOAT_EQ(range, 2.0);
  EXPECT_FLOAT_EQ(range_sigma, 1.2);
}

TEST_F(ArtifactLocalizationFixtureTest, getRangeFromBoundingBoxTest5) {
  // Create bounding box
  darknet_ros_msgs::BoundingBox bbox;
  bbox.Class = "Survivor";
  bbox.xmin = 162;
  bbox.ymin = 70;
  bbox.xmax = 262;
  bbox.ymax = 170;

  // Create lidar message with points inside bounding box at different distances
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
  PointCloud::Ptr lidar_pc(new PointCloud);
  lidar_pc->points.push_back(pcl::PointXYZ(0.0, 0.0, 1.0));
  lidar_pc->points.push_back(pcl::PointXYZ(0.0, 0.1, 2.0));
  lidar_pc->points.push_back(pcl::PointXYZ(0.1, 0.0, 3.0));
  lidar_pc->points.push_back(pcl::PointXYZ(0.1, 0.1, 4.0));
  sensor_msgs::PointCloud2Ptr lidar_msg(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*lidar_pc, *lidar_msg);
  lidar_msg->header.frame_id = robot_namespace_frames + "/velodyne";

  c_->config_.foreground_depth_percentiles.close_percentile = 0.0;
  c_->config_.foreground_depth_percentiles.far_percentile = 1.0;
  c_->config_.foreground_depth_nn_dist_to_median = 0.5;

  double range;
  double range_sigma;
  sensor_msgs::ImagePtr empty_img(new sensor_msgs::Image());
  EXPECT_TRUE(c_->getRangeFromBoundingBox(
      bbox, lidar_msg, *empty_img, range, range_sigma));
  // Foreground percentiles: [0.0, 1.0]
  // Depth points in [0.0, 1.0] percentile range: [1.0, 2.0, 3.0, 4.0]
  // Median of points in [0.0, 1.0] percentile range: 2.5
  // Foreground points = Nearest neighbours within 0.5m radius of 2.5:
  // [2.0, 3.0] Survivor percentiles: [0.15, 0.3] Closest foreground points in
  // [0.15, 0.3] percentile range: [2.0] range = Mean of closest foreground
  // points in [0.15, 0.3] percentile range: 2.0 range_sigma = coeff0 + coeff1 *
  // range = 0.4 + 0.4 * 2.0 = 1.2
  EXPECT_FLOAT_EQ(range, 2.0);
  EXPECT_FLOAT_EQ(range_sigma, 1.2);
}

TEST_F(ArtifactLocalizationFixtureTest, getRangeFromBoundingBoxTest6) {
  // Create bounding box
  darknet_ros_msgs::BoundingBox bbox;
  bbox.Class = "Survivor";
  bbox.xmin = 162;
  bbox.ymin = 70;
  bbox.xmax = 262;
  bbox.ymax = 170;

  // Create lidar message with points both inside & outside bounding box
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
  PointCloud::Ptr lidar_pc(new PointCloud);
  lidar_pc->points.push_back(pcl::PointXYZ(10.0, 0.0, 1.0));
  lidar_pc->points.push_back(pcl::PointXYZ(0.0, 10.0, 1.0));
  lidar_pc->points.push_back(pcl::PointXYZ(10.0, 10.0, 1.0));
  lidar_pc->points.push_back(pcl::PointXYZ(-10.0, 0.0, 1.0));
  lidar_pc->points.push_back(pcl::PointXYZ(0.0, -10.0, 1.0));
  lidar_pc->points.push_back(pcl::PointXYZ(-10.0, -10.0, 1.0));
  lidar_pc->points.push_back(pcl::PointXYZ(0.0, 0.0, 1.0));
  lidar_pc->points.push_back(pcl::PointXYZ(0.0, 0.1, 2.0));
  lidar_pc->points.push_back(pcl::PointXYZ(0.1, 0.0, 3.0));
  lidar_pc->points.push_back(pcl::PointXYZ(0.1, 0.1, 4.0));
  sensor_msgs::PointCloud2Ptr lidar_msg(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*lidar_pc, *lidar_msg);
  lidar_msg->header.frame_id = robot_namespace_frames + "/velodyne";

  c_->config_.foreground_depth_percentiles.close_percentile = 0.0;
  c_->config_.foreground_depth_percentiles.far_percentile = 0.75;
  c_->config_.foreground_depth_nn_dist_to_median = 0.5;

  double range;
  double range_sigma;
  sensor_msgs::ImagePtr empty_img(new sensor_msgs::Image());
  EXPECT_TRUE(c_->getRangeFromBoundingBox(
      bbox, lidar_msg, *empty_img, range, range_sigma));
  // Depth points inside bounding box: [1.0, 2.0, 3.0, 4.0]
  // Foreground percentiles: [0.0, 0.75]
  // Depth points in [0.0, 0.75] percentile range: [1.0, 2.0, 3.0]
  // Median of points in [0.0, 0.75] percentile range: 2.0
  // Foreground points = Nearest neighbours within 0.5m radius of 2.0: [2.0]
  // Survivor percentiles: [0.15, 0.3]
  // Closest foreground points in [0.15, 0.3] percentile range: [2.0]
  // range = Mean of closest foreground points in [0.15, 0.3] percentile
  // range: 2.0 range_sigma = coeff0 + coeff1 * range = 0.4 + 0.4 * 2.0 = 1.2
  EXPECT_FLOAT_EQ(range, 2.0);
  EXPECT_FLOAT_EQ(range_sigma, 1.2);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_artifact_localization_unit");
  return RUN_ALL_TESTS();
}
