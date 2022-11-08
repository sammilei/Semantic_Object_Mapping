/**
 *  @brief Simple talker class
 */

#pragma once

#include "ros/ros.h"
#include <algorithm>
#include <artifact_msgs/Artifact.h>
#include <artifact_msgs/GetNetvladDesc.h>
#include <darknet_ros_msgs/Detect.h>
#include <fstream>
#include <iostream>
#include <queue>
#include <ros/console.h>
#include <string>
#include <string_view>
#include <vector>

struct ArtifactWithDesc : artifact_msgs::Artifact {
  ArtifactWithDesc();
  ArtifactWithDesc(artifact_msgs::Artifact const& a);
  ~ArtifactWithDesc();
  float* visual_desc_;
};

class ArtifactReconciliation {
public:
  ArtifactReconciliation(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  ~ArtifactReconciliation();
  void unreconciledArtifactClbk(const artifact_msgs::Artifact& msg);
  // netvladServiceQuery()
  void run();
  void shortenId(std::string& id);
  float spatial_distance(const geometry_msgs::PointStamped& x1,
                         const geometry_msgs::PointStamped& x2);
  float descriptor_distance(const float* desc1, const float* desc2);
  bool hasParent(ArtifactWithDesc& a);
  void findOriginalParent(std::string& parent_id);
  static bool compare_confidence(const darknet_ros_msgs::Object& det1,
                                 const darknet_ros_msgs::Object& det2);
  void redetect_image(ArtifactWithDesc& art);
  ros::NodeHandle nh_, pnh_;
  std::vector<std::shared_ptr<ros::Subscriber>> unreconciled_artifact_subs_;
  ros::ServiceClient get_netvlad_desc_srv_;
  ros::ServiceClient redetection_client_rgb_, redetection_client_therm_;
  ros::Publisher reconciled_artifact_pub_;
  std::vector<ArtifactWithDesc> artifacts_;
  std::vector<std::string> robot_names_;
  float association_radius_;
  float netvlad_threshold_;
  float netvlad_association_radius_;
  std::string results_file_;
  bool analyse_results_;
  std::string hotspot_prefix_;
  int netvlad_desc_dim_;
};
