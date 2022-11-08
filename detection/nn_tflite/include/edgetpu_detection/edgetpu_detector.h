/*
 * edgetpu_detector.h
 * Adapted from YoloObjectDetector.h, Marko Bjelonic, ETH Zurich, Robotic
 * Systems Lab
 */

#pragma once

// c++
#include <chrono>
#include <cmath>
#include <iostream>
#include <pthread.h>
#include <string>
#include <thread>
#include <vector>

// ROS
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>

// OpenCv
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

// darknet_ros_msgs
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>
#include <darknet_ros_msgs/Detect.h>
#include <darknet_ros_msgs/FullDetection.h>
#include <darknet_ros_msgs/Object.h>
#include <darknet_ros_msgs/ObjectCount.h>

#include "ros/ros.h"
#include <algorithm>
#include <fstream>
#include <map>
#include <memory>
#include <ostream>
#include <regex>

#include "edgetpu.h"
#include "model_utils.h"
#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/model.h"

namespace coral {

class EdgeTpuDetector {
public:
  /*!
   * Constructor.
   */
  explicit EdgeTpuDetector(ros::NodeHandle nh, ros::NodeHandle nh_private);

  /*!
   * Destructor.
   */
  ~EdgeTpuDetector();

private:
  /*!
   * Reads a text file with all detection labels.
   * @return map of class id to label.
   */
  std::map<int, std::string> readLabelsFile(const std::string& label_path);

  /*!
   * Initialize the ROS connections.
   */
  void init();

  /*!
   * Convert an image from a ROS message into the format for inference.
   */
  std::vector<uint8_t> decodeImageMsg(const sensor_msgs::ImageConstPtr& msg);

  /*!
   * Callback of camera.
   * @param[in] msg image pointer.
   */
  void cameraCallback(const sensor_msgs::ImageConstPtr& msg);

  /*!
   * Publishes the detection image.
   * @return true if successful.
   */
  bool
  publishDetectionImage(const sensor_msgs::ImageConstPtr& image_msg,
                        const darknet_ros_msgs::BoundingBoxes& bounding_boxes);

  //! ROS node handle.
  ros::NodeHandle node_handle_, node_handle_private_;

  //! Advertise and subscribe to image topics.
  image_transport::ImageTransport image_transport_;

  //! ROS subscriber and publisher.
  std::vector<image_transport::Subscriber> image_subscribers_;
  ros::Publisher object_publisher_;
  //ros::Publisher bounding_boxes_publisher_;
  ros::Publisher detected_object_publisher_;

  //! Detected objects.
  darknet_ros_msgs::BoundingBoxes bounding_boxes_results_;
  darknet_ros_msgs::Object detected_object_result_;
  std::vector<darknet_ros_msgs::Object> object_messages_;

  //! Camera related parameters.
  int frame_width_;
  int frame_height_;

  //! Publisher of the bounding box image.
  ros::Publisher detection_image_publisher_;

  // Tensorflow interpreter
  std::unique_ptr<tflite::Interpreter> interpreter_;
  std::unique_ptr<tflite::FlatBufferModel> model_;
  std::shared_ptr<edgetpu::EdgeTpuContext> edgetpu_context_;
  std::map<int, std::string> labels_map_;
  int detection_image_width_;
  int detection_image_height_;
  int detection_image_channels_;
  cv::Size detection_image_size_;
  float detection_threshold_;

  bool detectServer(darknet_ros_msgs::Detect::Request& req,
                    darknet_ros_msgs::Detect::Response& res);

  void publishBoundingBox(const std::vector<darknet_ros_msgs::Object>& bounding_boxes,
                          const sensor_msgs::ImageConstPtr& image_msg);

  darknet_ros_msgs::FullDetection
  getFullDetect(const sensor_msgs::ImageConstPtr& image_msg,
                const darknet_ros_msgs::Object& bounding_box);

  ros::ServiceServer detection_service_;

  ros::Duration detection_service_timeout_;
  ros::Time detection_service_start_;

  bool detection_service_running_ = false;
};

} /* namespace coral*/
