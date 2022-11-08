#include "artifact_localization/localizer.h"

int main(int argc, char** argv) {
  // Initialize ROS
  ros::init(argc, argv, "artifact_localization");

  // Initialize public and private node handles
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // Initialize localizer class
  Localizer localizer(nh, pnh);
  localizer.run();

  return 0;
}