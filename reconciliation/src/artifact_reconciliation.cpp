#include <artifact_reconciliation/artifact_reconciliation.h>

ArtifactWithDesc::ArtifactWithDesc(artifact_msgs::Artifact const& a)
  : artifact_msgs::Artifact(a), visual_desc_(nullptr) {}

ArtifactWithDesc::ArtifactWithDesc()
  : artifact_msgs::Artifact(), visual_desc_(nullptr) {}

ArtifactWithDesc::~ArtifactWithDesc() {}

ArtifactReconciliation::ArtifactReconciliation(const ros::NodeHandle& nh,
                                               const ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh) {
  pnh_.getParam("/robots", robot_names_);
  pnh_.getParam("association_radius", association_radius_);
  pnh_.getParam("hotspot_prefix", hotspot_prefix_);
  pnh_.getParam("netvlad_desc_dim", netvlad_desc_dim_);
  pnh_.getParam("netvlad_threshold", netvlad_threshold_);
  pnh_.getParam("netvlad_association_radius", netvlad_association_radius_);
  pnh_.getParam("results_file", results_file_);
  pnh_.getParam("analyse_results", analyse_results_);

  for (const auto& n : robot_names_) {
    ros::Subscriber sub =
        nh_.subscribe("/" + n + "/artifact",
                      10000,
                      &ArtifactReconciliation::unreconciledArtifactClbk,
                      this);
    std::shared_ptr<ros::Subscriber> sub_ptr =
        std::make_shared<ros::Subscriber>(sub);
    unreconciled_artifact_subs_.push_back(sub_ptr);
  }

  get_netvlad_desc_srv_ =
      nh_.serviceClient<artifact_msgs::GetNetvladDesc>("get_netvlad_desc");

  reconciled_artifact_pub_ =
      nh_.advertise<artifact_msgs::Artifact>("reconciled_artifact", 1);

  redetection_client_rgb_ =
      nh_.serviceClient<darknet_ros_msgs::Detect>("detection_service_rgb");

  redetection_client_therm_ =
      nh_.serviceClient<darknet_ros_msgs::Detect>("detection_service_therm");
}

ArtifactReconciliation::~ArtifactReconciliation() {}

void ArtifactReconciliation::run() {
  ros::Rate r(5);
  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
}

void ArtifactReconciliation::unreconciledArtifactClbk(
    const artifact_msgs::Artifact& msg) {
  ArtifactWithDesc candidate(msg);

  // Store visual descriptor if color detection
  if (candidate.detection_source == "color") {
    artifact_msgs::GetNetvladDesc srv;
    srv.request.im = candidate.thumbnail;
    get_netvlad_desc_srv_.call(srv);
    if (srv.response.descriptor.size() == netvlad_desc_dim_) {
      candidate.visual_desc_ = new float[netvlad_desc_dim_];
      copy(srv.response.descriptor.begin(),
           srv.response.descriptor.end(),
           candidate.visual_desc_);
    }
  } else {
    ROS_WARN("No visual descriptor for detection source '%s'",
             candidate.detection_source.c_str());
  }

  ROS_INFO_STREAM("Received: " << candidate.label << " ID: " << candidate.id
                               << " at " << candidate.point.point.x << " "
                               << candidate.point.point.y << " "
                               << candidate.point.point.z << " from "
                               << candidate.name);

  // see if redetecting with base station YOLO improves the confidence
  redetect_image(candidate);

  if (hasParent(candidate)) {
    findOriginalParent(candidate.parent_id);
  }
  // write the reconciliation in a csv
  if (analyse_results_) {
    std::ofstream outfile;
    outfile.open(results_file_,
                 std::ios_base::app); // append instead of overwrite
    outfile << candidate.id + "," + candidate.parent_id + "\n";
  }
  if (candidate.id != candidate.parent_id) {
    ROS_INFO_STREAM("  " << candidate.id << " (" << candidate.label
                         << ") has been linked to its original parent: "
                         << candidate.parent_id);
  } else {
    ROS_INFO_STREAM("  " << candidate.id << " (" << candidate.label
                         << ") is unique");
  }
  artifact_msgs::Artifact* to_pub = &candidate;
  reconciled_artifact_pub_.publish(*to_pub);
  artifacts_.push_back(candidate);
}

void ArtifactReconciliation::redetect_image(ArtifactWithDesc& art) {
  if (!redetection_client_rgb_.exists()) {
    ROS_WARN_STREAM("Base redetection service not found - Not redetecting for "
                    "reconciled artifact");
    return;
  }
  // Check whether a visual artifact
  if (art.detection_source == "wifi" || art.detection_source == "gas") {
    return;
  }

  // Thumbnail redetection using Full YOLO
  darknet_ros_msgs::Detect srv;

  // populate service request with thumbnail
  srv.request.image = art.thumbnail;

  bool service_call_result;

  // previous code when using uncompressed thumbnails
  // if (art.thumbnail.encoding == "bgr8") {
  //  // color
  //  service_call_result = redetection_client_rgb_.call(srv);
  //} else if (art.thumbnail.encoding == "mono8") {
  //  // thermal
  //  service_call_result = redetection_client_therm_.call(srv);
  //} else
  //  return;

  if (art.detection_source.rfind("camera_", 0) == 0) {
    service_call_result = redetection_client_rgb_.call(srv);
  } else if (art.detection_source == "boson") {
    // thermal
    service_call_result = redetection_client_therm_.call(srv);
  } else
    return;

  if (service_call_result) {
    std::vector<darknet_ros_msgs::Object> dets = srv.response.detections;
    // sort by descending confidence if more than 1 detection box
    if (dets.empty()) {
      ROS_INFO_STREAM("Base station did not redetect any object");
    } else {
      sort(dets.begin(), dets.end(), compare_confidence);

      ROS_INFO_STREAM("Robot and base station YOLO confidences: "
                      << art.confidence << " -> "
                      << dets[0].box.yolo_probability);
      // get the highest confidence box
      art.confidence_base = dets[0].box.yolo_probability;
      // Do something here if we want to update the scorability based on the
      // base station confidence ! i.e. if base station doesn't find artifact or
      // if BS confidence much smaller than yolo conf, reduce the scorability.
      // Take into account the nbr of observations as well (if high number of
      // observation the BS should have less impact)
    }
  } else {
    ROS_ERROR("Image redetection service timed out");
  }
}

bool ArtifactReconciliation::compare_confidence(
    const darknet_ros_msgs::Object& det1,
    const darknet_ros_msgs::Object& det2) {
  return det1.box.probability > det2.box.probability;
}

float ArtifactReconciliation::descriptor_distance(const float* desc1,
                                                  const float* desc2) {
  float d = 0.0;
  for (unsigned int i = 0; i < netvlad_desc_dim_; i++) {
    d += pow(desc1[i] - desc2[i], 2);
  }
  return sqrt(d);
}

float ArtifactReconciliation::spatial_distance(
    const geometry_msgs::PointStamped& x1,
    const geometry_msgs::PointStamped& x2) {
  return sqrt(pow(x1.point.x - x2.point.x, 2) +
              pow(x1.point.y - x2.point.y, 2) +
              pow(x1.point.z - x2.point.z, 2));
}

bool ArtifactReconciliation::hasParent(ArtifactWithDesc& a) {
  ROS_ASSERT_MSG(a.parent_id == a.id ||
                     a.id.substr(0, hotspot_prefix_.length()) !=
                         hotspot_prefix_,
                 "parent_id must equal id. Do not call this function unless "
                 "artifact is newly received");
  // TODO: Handle properly the case of a new artifact linking two non
  // previously linked artifacts (finding several no currently related valid
  // parents)

  float closest_desc_dist = netvlad_threshold_;
  float closest_spatial_dist = association_radius_;
  float spatial_dist_to_comp, desc_dist_to_comp;
  for (auto it = artifacts_.rbegin(); it != artifacts_.rend(); ++it) {
    // Look for best visual match within radius if possible, or rely only on
    // location if no visual descriptors

    if ((a.label == it->label) && a.visual_desc_ && it->visual_desc_) {
      desc_dist_to_comp = descriptor_distance(a.visual_desc_, it->visual_desc_);
      if ((desc_dist_to_comp < closest_desc_dist) &&
          (spatial_distance(a.point, it->point) <
           netvlad_association_radius_)) {
        closest_desc_dist = desc_dist_to_comp;
        // Found a close enough visual descriptor within association radius
        a.parent_id = it->id;
      }
    } else if ((closest_desc_dist >= netvlad_threshold_) &&
               (a.label == it->label)) {
      spatial_dist_to_comp = spatial_distance(a.point, it->point);
      if (spatial_dist_to_comp < closest_spatial_dist) {
        closest_spatial_dist = spatial_dist_to_comp;
        // Found an instance not yet matched visually within radius
        a.parent_id = it->id;
      }
    }
  }

  return ((closest_desc_dist < netvlad_threshold_) ||
          (closest_spatial_dist < association_radius_));
}

void ArtifactReconciliation::findOriginalParent(std::string& parent_id) {
  // this function traverses the list of artifacts from most recent to earliest
  // and works out the earliest related artifact
  for (auto it = artifacts_.rbegin(); it != artifacts_.rend(); ++it) {
    if (it->id == parent_id) {
      parent_id = it->parent_id;
    }
  }
}
