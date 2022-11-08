/**
 *  @brief Test cases for artifact
 */

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <artifact_msgs/Artifact.h>

#include "artifact_localization/artifact.h"
#include "artifact_localization/observation.h"

TEST(ArtifactTest, testBearingMeasurements) {
  ros::Time::init();

  // Artifact position is at (1, 1, 0)
  // Obtain bearing measurements from (0, 0, 0) and (1, 0, 0)
  ArtifactConfig config;
  config.name = "Backpack";
  config.abbreviation = "ba";
  config.robot_name = "husky1";
  config.map_frame = "husky1/map";
  config.base_link_frame = "husky1/base_link";
  auto artifact = boost::make_shared<Artifact>(config);

  Observation obs1;
  obs1.stamp = ros::Time::now();
  obs1.bearing = Eigen::Vector3d(1, 1, 0).normalized();
  obs1.bearing_sigma = 0.2;
  obs1.map_T_base_link = Eigen::Affine3d::Identity();
  obs1.map_T_sensor = obs1.map_T_base_link;

  Observation obs2;
  obs2.stamp = ros::Time::now();
  obs2.bearing = Eigen::Vector3d(0, 1, 0).normalized();
  obs2.bearing_sigma = 0.2;
  obs2.map_T_base_link = Eigen::Affine3d::Identity();
  obs2.map_T_base_link.translation() << 1, 0, 0;
  obs2.map_T_sensor = obs2.map_T_base_link;

  artifact->addBearingObservation(obs1);
  artifact->addBearingObservation(obs2);

  // Get correct position estimate
  EXPECT_TRUE(artifact->isPositionValid());

  auto position = artifact->getPositionInMap();
  double eps = 0.2;
  EXPECT_NEAR(position.x(), 1, eps);
  EXPECT_NEAR(position.y(), 1, eps);
  EXPECT_NEAR(position.z(), 0, eps);

  // Get valid covariance
  auto cov = artifact->getPositionInMap();
  EXPECT_FALSE(cov.array().isNaN().any());

  // Can generate message
  auto msg = artifact->toMessageInMap();
  EXPECT_FALSE(msg->header.stamp.isZero());
  EXPECT_EQ(msg->header.frame_id, config.map_frame);
  EXPECT_FALSE(msg->point.header.stamp.isZero());
  EXPECT_EQ(msg->point.header.frame_id, config.map_frame);
  EXPECT_NEAR(msg->point.point.x, 1, eps);
  EXPECT_NEAR(msg->point.point.y, 1, eps);
  EXPECT_NEAR(msg->point.point.z, 0, eps);
  EXPECT_EQ(msg->name, config.robot_name);
  EXPECT_EQ(msg->label, config.name);

  msg = artifact->toMessageInBaseLink();
  EXPECT_FALSE(msg->header.stamp.isZero());
  EXPECT_EQ(msg->header.frame_id, config.base_link_frame);
  EXPECT_FALSE(msg->point.header.stamp.isZero());
  EXPECT_EQ(msg->point.header.frame_id, config.base_link_frame);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_artifact");
  return RUN_ALL_TESTS();
}
