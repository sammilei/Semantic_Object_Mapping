/**
 *  @brief Test cases for talker class
 *
 *  This file shows an example usage of gtest.
 */

#include <gtest/gtest.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "artifact_reconciliation/artifact_reconciliation.h"
#include <sensor_msgs/Image.h>

// OpenCV2.
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#ifdef TEST_IMAGES_PATH
std::string testImagesPath_ = TEST_IMAGES_PATH;
#else
#error Path of test images is not defined in CMakeLists.txt.
#endif

class ArtifactReconciliationFixtureTest : public ::testing::Test {
protected:
  virtual void SetUp() {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    ros::param::set("~/association_radius", 5.0);
    ros::param::set("~/netvlad_desc_dim", 32);
    ros::param::set("~/netvlad_threshold", 0.5);
    ros::param::set("~/netvlad_association_radius", 15.0);

    ar = new ArtifactReconciliation(nh, pnh);

    // // Load test images
    std::string pathToTestImage = testImagesPath_ + "/test_image_1.jpg";
    cv_bridge::CvImagePtr cv_ptr1(new cv_bridge::CvImage);
    cv_ptr1->image = cv::imread(pathToTestImage, CV_LOAD_IMAGE_COLOR);
    cv_ptr1->encoding = sensor_msgs::image_encodings::RGB8;
    image1 = *(cv_ptr1->toCompressedImageMsg());

    pathToTestImage = testImagesPath_ + "/test_image_2.jpg";
    cv_bridge::CvImagePtr cv_ptr2(new cv_bridge::CvImage);
    cv_ptr2->image = cv::imread(pathToTestImage, CV_LOAD_IMAGE_COLOR);
    cv_ptr2->encoding = sensor_msgs::image_encodings::RGB8;
    image2 = *(cv_ptr2->toCompressedImageMsg());
    sleep(5);
  }

  virtual void TearDown() {}
  ArtifactReconciliation* ar;
  sensor_msgs::CompressedImage image1;
  sensor_msgs::CompressedImage image2;
};



TEST_F(ArtifactReconciliationFixtureTest, hasParentTrue) {
  ArtifactWithDesc child;
  child.id = "asdf";
  child.parent_id = child.id;
  child.label = "backpack";
  child.point.point.x = 0.0;
  child.point.point.y = 0.0;
  child.point.point.z = 0.0;
  ArtifactWithDesc parent = child;
  parent.id = "qwer";
  parent.parent_id = "qwer";

  ar->artifacts_.push_back(parent);

  bool result = ar->hasParent(child);
  EXPECT_TRUE(result);
  EXPECT_EQ(child.parent_id, parent.id);
}

TEST_F(ArtifactReconciliationFixtureTest, hasParentFalseTooFarSameCategory) {
  ArtifactWithDesc child;
  child.id = "asdf";
  child.parent_id = child.id;
  child.label = "backpack";
  child.point.point.x = 0.0;
  child.point.point.y = 0.0;
  child.point.point.z = 0.0;
  ArtifactWithDesc parent = child;
  parent.id = "qwer";
  parent.parent_id = "qwer";
  parent.point.point.x = 6.0;
  parent.point.point.y = 0.0;
  parent.point.point.z = 0.0;

  ar->artifacts_.push_back(parent);

  bool result = ar->hasParent(child);
  EXPECT_FALSE(result);
}

TEST_F(ArtifactReconciliationFixtureTest, hasParentFalseCloseDiffCategory) {
  ArtifactWithDesc child;
  child.id = "asdf";
  child.parent_id = child.id;
  child.label = "backpack";
  child.point.point.x = 0.0;
  child.point.point.y = 0.0;
  child.point.point.z = 0.0;
  ArtifactWithDesc parent = child;
  parent.id = "qwer";
  parent.parent_id = "qwer";
  child.label = "survivor";

  ar->artifacts_.push_back(parent);

  bool result = ar->hasParent(child);
  EXPECT_FALSE(result);
}

TEST_F(ArtifactReconciliationFixtureTest, FindOriginalParent) {
  ArtifactWithDesc child;
  child.id = "asdf";
  child.parent_id = "qwer";
  child.label = "backpack";
  child.point.point.x = 0.0;
  child.point.point.y = 0.0;
  child.point.point.z = 0.0;
  ArtifactWithDesc parent = child;
  parent.id = "qwer";
  parent.parent_id = "zxcv";
  ArtifactWithDesc grandparent = parent;
  grandparent.id = "zxcv";
  grandparent.parent_id = "zxcv";

  ar->artifacts_.push_back(grandparent);
  ar->artifacts_.push_back(parent);

  ar->findOriginalParent(child.parent_id);
  EXPECT_EQ(child.parent_id, grandparent.id);
}

TEST_F(ArtifactReconciliationFixtureTest, InRangeSameCategoryDifferentRobot) {
  ArtifactWithDesc child;
  child.id = "asdf";
  child.parent_id = child.id;
  child.name = "husky2";
  child.label = "backpack";
  child.point.point.x = 0.0;
  child.point.point.y = 0.0;
  child.point.point.z = 0.0;
  ArtifactWithDesc parent = child;
  parent.id = "qwer";
  parent.name = "husky1";
  parent.parent_id = "qwer";

  ar->artifacts_.push_back(parent);

  bool result = ar->hasParent(child);
  EXPECT_TRUE(result);
  EXPECT_EQ(child.parent_id, parent.id);
}

/*
TEST_F(ArtifactReconciliationFixtureTest, hasParentTrueWithNetvlad) {
  artifact_msgs::Artifact child;
  child.id = "asdfeg_color";
  child.parent_id = child.id;
  child.label = "backpack";
  child.point.point.x = 0.0;
  child.point.point.y = 0.0;
  child.point.point.z = 0.0;
  child.detection_source = "camera_front";
  child.thumbnail = image1;
  artifact_msgs::Artifact parent = child;
  parent.id = "qwerty_color";
  parent.parent_id = "qwerty_color";
  parent.detection_source = "camera_left";
  ar->unreconciledArtifactClbk(parent);
  ar->unreconciledArtifactClbk(child);

  EXPECT_EQ(ar->artifacts_.back().parent_id, parent.id);
}

TEST_F(ArtifactReconciliationFixtureTest, hasParentFalseTooFarNetvlad) {
  artifact_msgs::Artifact child;
  child.id = "asd_color";
  child.parent_id = child.id;
  child.label = "backpack";
  child.point.point.x = 0.0;
  child.point.point.y = 0.0;
  child.point.point.z = 0.0;
  child.thumbnail = image1;
  child.detection_source = "color";
  artifact_msgs::Artifact parent = child;
  parent.id = "qwer_color";
  parent.parent_id = "qwer_color";
  parent.thumbnail = image2;
  parent.detection_source = "color";

  ar->unreconciledArtifactClbk(parent);
  ar->unreconciledArtifactClbk(child);

  EXPECT_EQ(ar->artifacts_.back().parent_id, ar->artifacts_.back().id);
}

TEST_F(ArtifactReconciliationFixtureTest,
       hasParentFalseTooFarDistValidNetvlad) {
  artifact_msgs::Artifact child;
  child.id = "asdf_color";
  child.parent_id = child.id;
  child.label = "backpack";
  child.point.point.x = 0.0;
  child.point.point.y = 0.0;
  child.point.point.z = 0.0;
  child.thumbnail = image1;
  child.detection_source = "camera_front";
  artifact_msgs::Artifact parent = child;
  parent.id = "qwer_color";
  parent.parent_id = "qwer_color";
  parent.point.point.x = 1000.0;
  parent.point.point.y = 1000.0;
  parent.point.point.z = 1000.0;
  parent.detection_source = "camera_left";

  ar->unreconciledArtifactClbk(parent);
  ar->unreconciledArtifactClbk(child);

  EXPECT_EQ(ar->artifacts_.back().parent_id, ar->artifacts_.back().id);
}
*/

TEST_F(ArtifactReconciliationFixtureTest,
       hasParentTrueOneColorMissingThumbnail) {
  artifact_msgs::Artifact child;
  child.id = "asdf_color";
  child.parent_id = child.id;
  child.label = "backpack";
  child.point.point.x = 0.0;
  child.point.point.y = 0.0;
  child.point.point.z = 0.0;
  child.detection_source = "camera_front";
  artifact_msgs::Artifact parent = child;
  parent.id = "qwer_color";
  parent.parent_id = "qwer_color";
  parent.thumbnail = image1;
  parent.detection_source = "camera_left";

  ar->unreconciledArtifactClbk(parent);
  ar->unreconciledArtifactClbk(child);

  EXPECT_EQ(ar->artifacts_.back().parent_id, parent.id);
}

TEST_F(ArtifactReconciliationFixtureTest, hasParentTrueOneWithThermalImage) {
  artifact_msgs::Artifact child;
  child.id = "asdf_color";
  child.parent_id = child.id;
  child.label = "backpack";
  child.point.point.x = 0.0;
  child.point.point.y = 0.0;
  child.point.point.z = 0.0;
  child.thumbnail = image1;
  child.detection_source = "camera_front";

  artifact_msgs::Artifact parent = child;
  parent.id = "qwer_thermal";
  parent.parent_id = "qwer_thermal";
  parent.thumbnail = image2;
  parent.detection_source = "boson";

  ar->unreconciledArtifactClbk(parent);
  ar->unreconciledArtifactClbk(child);

  EXPECT_EQ(ar->artifacts_.back().parent_id, parent.id);
}

TEST_F(ArtifactReconciliationFixtureTest, sortDetectionConfidencesDescending) {
  std::vector<darknet_ros_msgs::Object> dets;
  darknet_ros_msgs::Object det;
  float prob1 = 0.9;
  float prob2 = 0.5;
  det.box.probability = prob1;
  dets.push_back(det);
  det.box.probability = prob2;
  dets.push_back(det);
  sort(dets.begin(), dets.end(), ar->compare_confidence);

  EXPECT_FLOAT_EQ(dets[0].box.probability, prob1);
}

TEST_F(ArtifactReconciliationFixtureTest, redetectConfidenceBase) {
  artifact_msgs::Artifact art;
  art.id = "asdf_color";
  art.parent_id = art.id;
  art.label = "backpack";
  art.point.point.x = 0.0;
  art.point.point.y = 0.0;
  art.point.point.z = 0.0;
  art.thumbnail = image1;
  art.detection_source = "camera_front";
  art.confidence_base = 0.0;

  ar->unreconciledArtifactClbk(art);

  // Base now uses the same confidence field
  // EXPECT_EQ(ar->artifacts_.back().confidence_base, 1.0);
  EXPECT_EQ(ar->artifacts_.back().confidence_base, 1.0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_artifact_reconciliation");
  return RUN_ALL_TESTS();
}
