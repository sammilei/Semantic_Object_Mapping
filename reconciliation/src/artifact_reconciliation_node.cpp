/**
 *  @brief ArtifactReconciliation node
 */

#include <artifact_reconciliation/artifact_reconciliation.h>

int main(int argc, char** argv) {
  // Initialize ROS
  ros::init(argc, argv, "artifact_reconciliation");

  // Initialize public and private node handles
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // Initialize classical filter class
  ArtifactReconciliation artifact_reconciliation(nh, pnh);
  artifact_reconciliation.run();

  return 0;
}