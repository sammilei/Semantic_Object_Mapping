#include <edgetpu_detection/edgetpu_detector.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "edgetpu_detection");
  ros::NodeHandle node_handle_public, node_handle("~");
  coral::EdgeTpuDetector edgeTpuDetector(node_handle_public, node_handle);

  ros::Rate r(30); // 0.1 hz
  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
