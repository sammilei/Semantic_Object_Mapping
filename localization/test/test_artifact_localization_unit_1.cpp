/**
 *  @brief Unit test cases for artifact_localization
 *
 *  This file contains primarily unit tests for artifact_localization.
 */

#include "test_artifact_localization_common.cpp"

TEST_F(ArtifactLocalizationFixtureTest, doubleTruncatedMeanTest1) {
  std::vector<double> vec;
  double close_percentile = 0.0;
  double far_percentile = 0.05;
  double truncated_mean;
  EXPECT_FALSE(c_->doubleTruncatedMean(
      vec, close_percentile, far_percentile, truncated_mean));
}

TEST_F(ArtifactLocalizationFixtureTest, doubleTruncatedMeanTest2) {
  std::vector<double> vec;
  vec.push_back(0.1);
  vec.push_back(0.2);
  vec.push_back(0.3);
  vec.push_back(0.4);
  vec.push_back(0.5);
  double close_percentile = 0.0;
  double far_percentile = 0.05;
  double truncated_mean;
  EXPECT_TRUE(c_->doubleTruncatedMean(
      vec, close_percentile, far_percentile, truncated_mean));
  EXPECT_FLOAT_EQ(truncated_mean, 0.1);
}

TEST_F(ArtifactLocalizationFixtureTest, doubleTruncatedMeanTest3) {
  std::vector<double> vec;
  vec.push_back(0.1);
  vec.push_back(0.2);
  vec.push_back(0.3);
  vec.push_back(0.4);
  vec.push_back(0.5);
  double close_percentile = 0.0;
  double far_percentile = 0.5;
  double truncated_mean;
  EXPECT_TRUE(c_->doubleTruncatedMean(
      vec, close_percentile, far_percentile, truncated_mean));
  EXPECT_FLOAT_EQ(truncated_mean, 0.2);
}

TEST_F(ArtifactLocalizationFixtureTest, doubleTruncatedMeanTest4) {
  std::vector<double> vec;
  vec.push_back(0.1);
  vec.push_back(0.2);
  vec.push_back(0.3);
  vec.push_back(0.4);
  vec.push_back(0.5);
  double close_percentile = 0.0;
  double far_percentile = 1.0;
  double truncated_mean;
  EXPECT_TRUE(c_->doubleTruncatedMean(
      vec, close_percentile, far_percentile, truncated_mean));
  EXPECT_FLOAT_EQ(truncated_mean, 0.3);
}

TEST_F(ArtifactLocalizationFixtureTest, doubleTruncatedMeanTest5) {
  std::vector<double> vec;
  vec.push_back(1.0);
  vec.push_back(2.0);
  vec.push_back(3.0);
  vec.push_back(4.0);
  double close_percentile = 0.0;
  double far_percentile = 1.0;
  double truncated_mean;
  EXPECT_TRUE(c_->doubleTruncatedMean(
      vec, close_percentile, far_percentile, truncated_mean));
  EXPECT_FLOAT_EQ(truncated_mean, 2.5);
}

TEST_F(ArtifactLocalizationFixtureTest, doubleTruncatedMedianTest1) {
  std::vector<double> vec;
  double close_percentile = 0.0;
  double far_percentile = 0.05;
  double truncated_median;
  EXPECT_FALSE(c_->doubleTruncatedMedian(
      vec, close_percentile, far_percentile, truncated_median));
}

TEST_F(ArtifactLocalizationFixtureTest, doubleTruncatedMedianTest2) {
  std::vector<double> vec;
  vec.push_back(0.1);
  vec.push_back(0.2);
  vec.push_back(0.3);
  vec.push_back(0.4);
  vec.push_back(0.5);
  double close_percentile = 0.0;
  double far_percentile = 0.05;
  double truncated_median;
  EXPECT_TRUE(c_->doubleTruncatedMedian(
      vec, close_percentile, far_percentile, truncated_median));
  EXPECT_FLOAT_EQ(truncated_median, 0.1);
}

TEST_F(ArtifactLocalizationFixtureTest, doubleTruncatedMedianTest3) {
  std::vector<double> vec;
  vec.push_back(0.1);
  vec.push_back(0.2);
  vec.push_back(0.3);
  vec.push_back(0.4);
  vec.push_back(0.5);
  double close_percentile = 0.0;
  double far_percentile = 0.5;
  double truncated_median;
  EXPECT_TRUE(c_->doubleTruncatedMedian(
      vec, close_percentile, far_percentile, truncated_median));
  EXPECT_FLOAT_EQ(truncated_median, 0.2);
}

TEST_F(ArtifactLocalizationFixtureTest, doubleTruncatedMedianTest4) {
  std::vector<double> vec;
  vec.push_back(0.1);
  vec.push_back(0.2);
  vec.push_back(0.3);
  vec.push_back(0.4);
  vec.push_back(0.5);
  double close_percentile = 0.0;
  double far_percentile = 1.0;
  double truncated_median;
  EXPECT_TRUE(c_->doubleTruncatedMedian(
      vec, close_percentile, far_percentile, truncated_median));
  EXPECT_FLOAT_EQ(truncated_median, 0.3);
}

TEST_F(ArtifactLocalizationFixtureTest, doubleTruncatedMedianTest5) {
  std::vector<double> vec;
  vec.push_back(1.0);
  vec.push_back(1.0);
  vec.push_back(1.0);
  vec.push_back(1.0);
  depth_percentile percentiles = c_->config_.foreground_depth_percentiles;
  double close_percentile = percentiles.close_percentile;
  double far_percentile = percentiles.far_percentile;
  double truncated_median;
  EXPECT_TRUE(c_->doubleTruncatedMedian(
      vec, close_percentile, far_percentile, truncated_median));
  EXPECT_FLOAT_EQ(truncated_median, 1.0);
}

TEST_F(ArtifactLocalizationFixtureTest, doubleNearestNeighborsTest1) {
  std::vector<double> vec;
  double query = 0.0;
  double radius = 0.5;
  std::vector<double> nearest_neighbors;
  EXPECT_FALSE(
      c_->doubleNearestNeighbors(vec, query, radius, nearest_neighbors));
}

TEST_F(ArtifactLocalizationFixtureTest, doubleNearestNeighborsTest2) {
  std::vector<double> vec;
  vec.push_back(0.1);
  double query = 0.1;
  double radius = 0.5;
  std::vector<double> nearest_neighbors;
  EXPECT_TRUE(
      c_->doubleNearestNeighbors(vec, query, radius, nearest_neighbors));
  EXPECT_EQ(nearest_neighbors.size(), 1);
  EXPECT_FLOAT_EQ(nearest_neighbors[0], 0.1);
}

TEST_F(ArtifactLocalizationFixtureTest, doubleNearestNeighborsTest3) {
  std::vector<double> vec;
  vec.push_back(0.1);
  vec.push_back(0.2);
  vec.push_back(0.3);
  vec.push_back(0.4);
  vec.push_back(0.5);
  double query = 0.25;
  double radius = 0.1;
  std::vector<double> nearest_neighbors;
  EXPECT_TRUE(
      c_->doubleNearestNeighbors(vec, query, radius, nearest_neighbors));
  EXPECT_EQ(nearest_neighbors.size(), 2);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_artifact_localization_unit");
  return RUN_ALL_TESTS();
}
