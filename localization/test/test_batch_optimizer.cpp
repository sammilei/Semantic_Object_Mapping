#include "artifact_localization/batch_optimizer.h"
#include <gtest/gtest.h>

class LocalizationTest : public ::testing::Test {
protected:
  virtual void SetUp() {
    std::srand(0);

    // True position of artifact
    actual_position_ << 0.5, 0.5, 0;
  }

  double simulateRange(const Eigen::Affine3d& pose) {
    return (actual_position_ - pose.translation()).norm();
  }

  Eigen::Vector3d simulateBearing(const Eigen::Affine3d& pose) {
    return actual_position_ - pose.translation();
  }

  double computeError(const Eigen::Vector3d position) {
    return (actual_position_ - position).norm();
  }

  bool isNaN(const Eigen::MatrixXd& mat) {
    return mat.array().isNaN().any();
  }

  BatchOptimizer optimizer_;
  Eigen::Vector3d actual_position_;
};

// By default, position and covariance should be nan
TEST_F(LocalizationTest, testConstructor) {
  auto position = optimizer_.getPosition();
  EXPECT_TRUE(isNaN(position));

  auto cov = optimizer_.getCovariance();
  EXPECT_TRUE(isNaN(cov));
}

// Takes range measurements from N random positions
TEST_F(LocalizationTest, testRangeFactor) {
  Eigen::Affine3d pose;
  pose.linear().setIdentity();
  for (int i = 0; i < 10; ++i) {
    pose.translation().setRandom();
    double range = simulateRange(pose);
    double noise = 0.1;
    optimizer_.addRangeMeasurement(range, noise, ros::Time(10 + i), pose);
  }

  // Compute position and covariance
  bool result = optimizer_.optimize();
  EXPECT_TRUE(result);

  auto position = optimizer_.getPosition();
  EXPECT_LT(computeError(position), 0.1);

  auto cov = optimizer_.getCovariance();
  EXPECT_FALSE(isNaN(cov));
}

TEST_F(LocalizationTest, testRangeFactorDegenerateCase) {
  // Takes range measurements from 2 random positions (degenerate case)
  Eigen::Affine3d pose;
  pose.linear().setIdentity();
  for (int i = 0; i < 2; ++i) {
    pose.translation().setRandom();
    double range = simulateRange(pose);
    double noise = 0.1;
    optimizer_.addRangeMeasurement(range, noise, ros::Time(10 + i), pose);
  }

  // Compute position and covariance
  bool result = optimizer_.optimize();
  EXPECT_FALSE(result);

  auto position = optimizer_.getPosition();
  EXPECT_TRUE(isNaN(position));

  auto cov = optimizer_.getCovariance();
  EXPECT_TRUE(isNaN(cov));
}

TEST_F(LocalizationTest, testBearingFactor) {
  // Takes range measurements from N random positions
  Eigen::Affine3d pose;
  pose.linear().setIdentity();
  for (int i = 0; i < 10; ++i) {
    pose.translation().setRandom();
    auto bearing = simulateBearing(pose);
    double noise = 0.1;
    optimizer_.addBearingMeasurement(bearing, noise, ros::Time(10 + i), pose);
  }

  // Compute position and covariance
  bool result = optimizer_.optimize();
  EXPECT_TRUE(result);

  auto position = optimizer_.getPosition();
  EXPECT_LT(computeError(position), 0.1);

  auto cov = optimizer_.getCovariance();
  EXPECT_FALSE(isNaN(cov));
}

TEST_F(LocalizationTest, testBearingFactorDegenerate) {
  // Takes range measurements from 1 random position (degenerate)
  Eigen::Affine3d pose;
  pose.linear().setIdentity();
  for (int i = 0; i < 1; ++i) {
    pose.translation().setRandom();
    auto bearing = simulateBearing(pose);
    double noise = 0.1;
    optimizer_.addBearingMeasurement(bearing, noise, ros::Time(10 + i), pose);
  }

  // Compute position and covariance
  bool result = optimizer_.optimize();
  EXPECT_FALSE(result);

  auto position = optimizer_.getPosition();
  EXPECT_TRUE(isNaN(position));

  auto cov = optimizer_.getCovariance();
  EXPECT_TRUE(isNaN(cov));
}

TEST_F(LocalizationTest, testBearingRangeFactor) {
  // Takes range measurements from N random positions
  Eigen::Affine3d pose;
  pose.linear().setIdentity();
  for (int i = 0; i < 10; ++i) {
    pose.translation().setRandom();
    auto bearing = simulateBearing(pose);
    auto range = simulateRange(pose);
    double bearing_noise = 0.1;
    double range_noise = 0.1;
    optimizer_.addBearingRangeMeasurement(
        bearing, range, bearing_noise, range_noise, ros::Time(10 + i), pose);
  }

  // Compute position and covariance
  bool result = optimizer_.optimize();
  EXPECT_TRUE(result);

  auto position = optimizer_.getPosition();
  EXPECT_LT(computeError(position), 0.1);

  auto cov = optimizer_.getCovariance();
  EXPECT_FALSE(isNaN(cov));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_batch_optimizer");
  return RUN_ALL_TESTS();
}