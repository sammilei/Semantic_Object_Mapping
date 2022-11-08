#include "artifact_localization/observation.h"

Observation::Observation() {}

Observation::Observation(
    const artifact_msgs::PointSourceDetectionConstPtr& msg) {
  stamp = msg->header.stamp;
  if (msg->id.empty() || msg->id.substr(0, 3) == "CO2") {
    label = "Gas";
  } else if (msg->id.substr(0, 2) == "Ph") {
    label = "Cell Phone";
  } else if (msg->id.substr(0, 2) == "Cu") {
    label = "Cube";
  } else {
    ROS_ERROR("Unrecognized point source detection type: %s",
              msg->id.c_str());
    label = "";
  }
  signal_msg = msg;
}

Observation::Observation(const darknet_ros_msgs::ObjectConstPtr& msg) {
  stamp = msg->header.stamp;
  label = msg->box.Class;
  confidence = msg->box.probability;
  yolo_confidence = msg->box.yolo_probability;
  color_confidence = msg->box.color_score;
  detection_source = msg->camera_name;
  vision_msg = msg;
}

visualization_msgs::MarkerPtr Observation::createRangeMarker() const {
  if (!isRangeValid()) {
    return NULL;
  }

  auto msg = boost::make_shared<visualization_msgs::Marker>();
  msg->header.stamp = stamp;
  msg->header.frame_id = map_frame;
  msg->ns = "range";
  msg->type = visualization_msgs::Marker::LINE_STRIP;

  msg->scale.x = 0.05;

  msg->color.r = 0.0;
  msg->color.g = 0.5;
  msg->color.b = 0.5;
  msg->color.a = 1.0;

  msg->pose.position.x = map_T_sensor(0, 3);
  msg->pose.position.y = map_T_sensor(1, 3);
  msg->pose.position.z = map_T_sensor(2, 3);
  msg->pose.orientation.w = 1.0;

  int n = 50;
  msg->points.reserve(n);
  for (int i = 0; i <= n; ++i) {
    geometry_msgs::Point pt;
    pt.x = range * std::cos(2.0 * M_PI * i / n);
    pt.y = range * std::sin(2.0 * M_PI * i / n);
    msg->points.push_back(pt);
  }

  return msg;
}

visualization_msgs::MarkerPtr Observation::createBearingMarker() const {
  if (!isBearingValid()) {
    return NULL;
  }

  auto msg = boost::make_shared<visualization_msgs::Marker>();
  msg->header.stamp = stamp;
  msg->header.frame_id = map_frame;
  msg->ns = "bearing";
  msg->type = visualization_msgs::Marker::LINE_LIST;
  msg->pose.orientation.w = 1.0;
  msg->scale.x = 0.03; // Line width

  msg->color.r = 0.0;
  msg->color.g = 1.0;
  msg->color.b = 0.0;
  msg->color.a = 1.0;

  msg->points.resize(2);
  const double length = 10.0;
  auto bearing_in_map = map_T_sensor * (length * bearing);
  msg->points[0].x = map_T_sensor(0, 3);
  msg->points[0].y = map_T_sensor(1, 3);
  msg->points[0].z = map_T_sensor(2, 3);
  msg->points[1].x = bearing_in_map(0);
  msg->points[1].y = bearing_in_map(1);
  msg->points[1].z = bearing_in_map(2);

  return msg;
}
