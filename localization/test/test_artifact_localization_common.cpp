/**
 *  @brief Test cases for talker class
 *
 *  This file shows an example usage of gtest.
 */

#include <gtest/gtest.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <pcl_ros/point_cloud.h>

#include "artifact_localization/artifact.h"
#include "artifact_localization/camera.h"
#include "artifact_localization/localizer.h"

class ArtifactLocalizationFixtureTest : public ::testing::Test {
protected:
  virtual void SetUp() {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    robot_namespace_nodes = "/husky1";
    robot_namespace_frames = "husky1";

    tf_buf_ = new tf2_ros::Buffer(ros::Duration(120));
    tf_listener_ = new tf2_ros::TransformListener(*tf_buf_);
    tf_buf_->setUsingDedicatedThread(true);

    l_ = new Localizer(nh, pnh);

    ros::Duration timeout(0.5);
    std::string camera_name = "camera_front";
    std::string camera_info_topic =
        robot_namespace_nodes + "/" + camera_name + "/camera_info";
    auto info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(
        camera_info_topic, timeout);

    Eigen::Affine3d base_link_T_sensor;
    if (!l_->getTransformEigenFromTf(robot_namespace_frames + "/base_link",
                                     info->header.frame_id,
                                     ros::Time(0),
                                     base_link_T_sensor)) {
      ROS_ERROR("Failed to get extrinsics for camera '%s'",
                camera_name.c_str());
    }

    c_ = new Camera(camera_name);
    c_->setIntrinsicsFromCameraInfo(info);
    c_->setExtrinsics(base_link_T_sensor);
    a_ = new Artifact(ArtifactConfig(), "");

    l_->initializePublishers();
    l_->initializeSubscribers();
    l_->initializeQueues();
    l_->setupSensorClbk();
  }

  virtual void TearDown() {}

  tf2_ros::TransformListener* tf_listener_;
  tf2_ros::Buffer* tf_buf_;

  Localizer* l_;
  Camera* c_;
  Artifact* a_;
  std::string robot_namespace_nodes;
  std::string robot_namespace_frames;
};
