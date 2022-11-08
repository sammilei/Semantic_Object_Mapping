#include "edgetpu_detection/edgetpu_detector.h"
#include <ros/console.h>

namespace coral {
char* labels;
char* models;
char* data;
char** detectionNames;
EdgeTpuDetector::EdgeTpuDetector(ros::NodeHandle nh, ros::NodeHandle nh_private)
  : node_handle_(nh),
    node_handle_private_(nh_private),
    image_transport_(node_handle_) {
  ROS_INFO("[EdgeTpuDetector] Node started.");

  init();
}

EdgeTpuDetector::~EdgeTpuDetector() {}

std::map<int, std::string> readLabelsFile(const std::string& label_path) {
  std::map<int, std::string> ret;
  std::ifstream label_file(label_path);
  if (!label_file.good()) {
    std::cerr << "label file: " << labels << " doesn't exist." << std::endl;
    std::abort();
  }

  for (std::string line; std::getline(label_file, line);) {
    std::istringstream ss(line);
    int id;
    ss >> id;
    line = std::regex_replace(line, std::regex("^ +[0-9]+ +"), "");
    ret.emplace(id, line);
  }
  return ret;
}

void EdgeTpuDetector::init() {
  ROS_INFO("[EdgeTpuDetector] init().");
  std::string labels_path, models_path, labels_file, model_file;

  // Check if EdgeTPU found
  edgetpu_context_ = edgetpu::EdgeTpuManager::GetSingleton()->OpenDevice();

  // Get paths
  node_handle_private_.param(
      "config_path", labels_path, std::string("/default"));
  node_handle_private_.param(
      "weights_path", models_path, std::string("/default"));

  // Path to classes file.
  node_handle_private_.param("detection_model/classes_names/file_name",
                             labels_file,
                             std::string("imagenet_labels.txt"));
  node_handle_private_.param(
      "detection_model/threshold", detection_threshold_, 0.3f);
  labels_path += "/" + labels_file;
  labels = new char[labels_path.length() + 1];
  strcpy(labels, labels_path.c_str());

  // Read labels.
  labels_map_ = coral::readLabelsFile(labels);

  // Build interpreter.
  if (!edgetpu_context_) {
    // Path to models file.
    node_handle_private_.param(
        "detection_model/cpu_tflite_model/file_name",
        model_file,
        std::string("mobilenet_v1_1.0_224_quant.tflite"));
    models_path += "/" + model_file;
    models = new char[models_path.length() + 1];
    strcpy(models, models_path.c_str());

    std::ifstream in_model_file(models);
    if (!in_model_file.good()) {
      std::cerr << "models file: " << models << " doesn't exist." << std::endl;
      std::abort();
    }

    // Read model.
    model_ = tflite::FlatBufferModel::BuildFromFile(models);
    if (model_ == nullptr) {
      std::cerr << "Fail to build FlatBufferModel from file: " << models
                << std::endl;
      std::abort();
    }

    interpreter_ = std::move(coral::buildInterpreter(*model_));
    ROS_WARN("[EdgeTpuDetector] Built interpreter, NOT USING EDGE TPU.");
  } else {
    // Path to models file.
    node_handle_private_.param(
        "detection_model/edgetpu_tflite_model/file_name",
        model_file,
        std::string("mobilenet_v1_1.0_224_quant_edgetpu.tflite"));
    models_path += "/" + model_file;
    models = new char[models_path.length() + 1];
    strcpy(models, models_path.c_str());
    // Read model.
    model_ = tflite::FlatBufferModel::BuildFromFile(models);
    if (model_ == nullptr) {
      std::cerr << "Fail to build FlatBufferModel from file: " << models
                << std::endl;
      std::abort();
    }
    interpreter_ = std::move(
        coral::buildEdgeTpuInterpreter(*model_, edgetpu_context_.get()));
    ROS_INFO("[EdgeTpuDetector] Built interpreter, using EdgeTPU.");
  }

  const auto& required_shape = coral::getInputShape(*interpreter_, 0);
  detection_image_width_ = required_shape[0];
  detection_image_height_ = required_shape[1];
  detection_image_channels_ = required_shape[2];
  detection_image_size_ =
      cv::Size(detection_image_width_, detection_image_height_);

  // Initialize publisher and subscriber.
  std::vector<std::string> camera_topics_names;
  int camera_queue_size;
  std::string object_detector_topic_name;
  int object_detector_queue_size;
  bool object_detector_latch;
  std::string bounding_boxes_topic_name;
  int bounding_boxes_queue_size;
  bool bounding_boxes_latch;
  std::string detection_image_topic_name;
  int detection_image_queue_size;
  bool detection_image_latch;
  std::string detected_object_topic_name;
  int detected_object_queue_size;
  bool detected_object_latch;
  float service_call_timeout;
  std::string redetection_service_topic_name;

  std::vector<std::string> default_topic_name{"/camera/image_raw"};
  node_handle_private_.param(
      "subscribers/camera_topics", camera_topics_names, default_topic_name);
  node_handle_private_.param("subscribers/queue_size", camera_queue_size, 1);
  node_handle_private_.param("publishers/object_detector/topic",
                             object_detector_topic_name,
                             std::string("found_object"));
  node_handle_private_.param(
      "publishers/object_detector/queue_size", object_detector_queue_size, 1);
  node_handle_private_.param(
      "publishers/object_detector/latch", object_detector_latch, false);
  node_handle_private_.param("publishers/bounding_boxes/topic",
                             bounding_boxes_topic_name,
                             std::string("bounding_boxes"));
  node_handle_private_.param(
      "publishers/bounding_boxes/queue_size", bounding_boxes_queue_size, 1);
  node_handle_private_.param(
      "publishers/bounding_boxes/latch", bounding_boxes_latch, false);
  node_handle_private_.param("publishers/detection_image/topic",
                             detection_image_topic_name,
                             std::string("detection_image"));
  node_handle_private_.param(
      "publishers/detection_image/queue_size", detection_image_queue_size, 1);
  node_handle_private_.param(
      "publishers/detection_image/latch", detection_image_latch, true);

  node_handle_private_.param("publishers/detected_object/topic",
                             detected_object_topic_name,
                             std::string("detected_object"));
  node_handle_private_.param(
      "publishers/detected_object/queue_size", detected_object_queue_size, 10);
  node_handle_private_.param(
      "publishers/detected_object/latch", detected_object_latch, true);

  node_handle_private_.param(
      "service_call_timeout", service_call_timeout, 10.0f);
  node_handle_private_.param("services/redetection/topic",
                             redetection_service_topic_name,
                             std::string("detection_service_rgb"));

  for (std::string s : camera_topics_names) {
    image_subscribers_.push_back(image_transport_.subscribe(
        s, camera_queue_size, &EdgeTpuDetector::cameraCallback, this));
  }

  object_publisher_ = node_handle_.advertise<darknet_ros_msgs::ObjectCount>(
      object_detector_topic_name,
      object_detector_queue_size,
      object_detector_latch);
  // bounding_boxes_publisher_ =
  //     node_handle_.advertise<darknet_ros_msgs::BoundingBoxes>(
  //         bounding_boxes_topic_name,
  //         bounding_boxes_queue_size,
  //         bounding_boxes_latch);
  detection_image_publisher_ =
      node_handle_.advertise<sensor_msgs::Image>(detection_image_topic_name,
                                                 detection_image_queue_size,
                                                 detection_image_latch);
  detected_object_publisher_ =
      node_handle_.advertise<darknet_ros_msgs::FullDetection>(
          detected_object_topic_name,
          detected_object_queue_size,
          detected_object_latch);

  detection_service_ = node_handle_.advertiseService(
      redetection_service_topic_name, &EdgeTpuDetector::detectServer, this);

  detection_service_timeout_ = ros::Duration(service_call_timeout);
}

// Decoding with opencv, with resizing
std::vector<uint8_t>
EdgeTpuDetector::decodeImageMsg(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr image_in;

  try {
    image_in = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    std::abort();
  }
  cv::Mat image_for_detection(detection_image_size_, CV_8UC3);
  resize(image_in->image,
         image_for_detection,
         detection_image_size_,
         0,
         0,
         cv::INTER_AREA);

  std::vector<uint8_t> output;
  if (image_for_detection.isContinuous()) {
    // array.assign((float*)mat.datastart, (float*)mat.dataend); // <- has
    // problems for sub-matrix like mat = big_mat.row(i)
    output.assign((uint8_t*)image_for_detection.data,
                  (uint8_t*)image_for_detection.data +
                      image_for_detection.total() *
                          image_for_detection.channels());
  } else {
    for (int i = 0; i < image_for_detection.rows; ++i) {
      output.insert(output.end(),
                    image_for_detection.ptr<uint8_t>(i),
                    image_for_detection.ptr<uint8_t>(i) +
                        image_for_detection.cols *
                            image_for_detection.channels());
    }
  }

  return output;
}

bool EdgeTpuDetector::detectServer(darknet_ros_msgs::Detect::Request& req,
                                   darknet_ros_msgs::Detect::Response& res) {
  sensor_msgs::ImageConstPtr msg =
      boost::make_shared<sensor_msgs::Image>(req.image);
  // route image through camera callback
  cameraCallback(msg);
  detection_service_running_ = true;
  detection_service_start_ = ros::Time::now();
  // need to wait for thread to return
  while ((ros::Time::now() - detection_service_start_) <
         detection_service_timeout_) {
    if (!detection_service_running_) { // i.e. if threads are finished
      res.detections = object_messages_;
      object_messages_.clear();
      return true;
    }
  }
  ROS_ERROR_STREAM("Service called timed out...");
  return false;
}

void EdgeTpuDetector::cameraCallback(
    const sensor_msgs::ImageConstPtr& image_msg) {
  ROS_DEBUG("[EdgeTpuDetector] USB image received.");
  // auto time_start = std::chrono::high_resolution_clock::now();
  const std::vector<uint8_t>& input = decodeImageMsg(image_msg);
  int frame_width = image_msg->width;
  int frame_height = image_msg->height;

  ros::Time detect_start = ros::Time::now();
  const std::vector<coral::BoundingBox> results =
      coral::runInference(input, interpreter_.get());
  std::cout << "detection takes " << 1/(ros::Time::now().toSec() - detect_start.toSec()) << "fps.\n";

  darknet_ros_msgs::ObjectCount obj_count_msg;
  obj_count_msg.header.stamp = image_msg->header.stamp;
  obj_count_msg.header.frame_id = "detection";
  obj_count_msg.count = results.size();
  object_publisher_.publish(obj_count_msg);

  darknet_ros_msgs::BoundingBox bounding_box;
  darknet_ros_msgs::BoundingBoxes bounding_boxes;
  darknet_ros_msgs::Object detected_object;
  std::vector<darknet_ros_msgs::Object> detected_objects;

  std::string camera_frame = image_msg->header.frame_id;
  camera_frame.erase(0, camera_frame.find("/") + 1);
  std::string camera_name = camera_frame.substr(0, camera_frame.find("/"));

  // when there are detections
  if (results.size() > 0) {
    bool to_publish_detection =
        false; // only publish when the detection conf is above conf threshold
    detected_object.header.frame_id = camera_frame;
    detected_object.camera_name = camera_name;
    detected_object.header.stamp = image_msg->header.stamp;

    bounding_boxes.header.stamp = image_msg->header.stamp;
    bounding_boxes.header.frame_id = "detection";
    bounding_boxes.image_header = image_msg->header;

    for (coral::BoundingBox bb : results) {
      // Ignore low confidence
      if (bb.score < detection_threshold_) {
        continue;
      }
      bounding_box.Class = labels_map_[bb.label];
      bounding_box.probability = bb.score;
      bounding_box.xmin = bb.xmin * frame_width;
      bounding_box.ymin = bb.ymin * frame_height;
      bounding_box.xmax = bb.xmax * frame_width;
      bounding_box.ymax = bb.ymax * frame_height;
      bounding_boxes.bounding_boxes.push_back(bounding_box);
      detected_object.box = bounding_box;
      object_messages_.push_back(detected_object);
      detected_objects.push_back(detected_object);
      to_publish_detection = true;
    }
    detection_service_running_ = false;

    if (to_publish_detection) {
      publishBoundingBox(detected_objects, image_msg);
      // bounding_boxes_publisher_.publish(bounding_boxes);
      if (!publishDetectionImage(image_msg, bounding_boxes)) {
        ROS_DEBUG("Detection image has not been broadcasted.");
      }
      to_publish_detection = false;
    }
  }

  // auto time_in_cb_ms =
  // ((float)(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now()
  // - time_start)).count())/1000; float frequency = 1000/time_in_cb_ms;
  // ROS_INFO("[EdgeTpuDetector] Time spent in callback: %fms", time_in_cb_ms);
  // ROS_INFO("[EdgeTpuDetector] Corresponding frequency: %fHz", frequency);
}

darknet_ros_msgs::FullDetection
EdgeTpuDetector::getFullDetect(const sensor_msgs::ImageConstPtr& image_msg,
                               const darknet_ros_msgs::Object& bounding_box) {
  cv_bridge::CvImagePtr image_cv;

  try {
    image_cv =
        cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::RGB8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception in getFulDetect: %s", e.what());
    std::abort();
  }

  darknet_ros_msgs::FullDetection full_det;
  full_det.bbox = bounding_box; // put the bounding box in the message
  cv_bridge::CvImage cvImage;
  cvImage.header.stamp = image_msg->header.stamp;
  cvImage.header.frame_id = "detection_image";
  cvImage.encoding = sensor_msgs::image_encodings::BGR8;
  cvImage.image = image_cv->image;
  full_det.rgb_img = *cvImage.toImageMsg();
  std::string name = ros::this_node::getNamespace();
  std::string name_robot = name.substr(1); // remove "/" before robot name
  full_det.robot_name = name_robot;
  return full_det;
}

void EdgeTpuDetector::publishBoundingBox(
    const std::vector<darknet_ros_msgs::Object>& bounding_boxes,
    const sensor_msgs::ImageConstPtr& image_msg) {
  // remove overlapping boxes
  bool unique = true;
  bool* handled;
  int size = bounding_boxes.size();
  handled = new bool[size];
  for (int i = 0; i < size; i++) {
    handled[i] = false;
  }

  if (size == 1) {
    if (bounding_boxes[0].box.Class != "husky" &&
        bounding_boxes[0].box.Class != "cellphone") {
      darknet_ros_msgs::FullDetection full_det;
      full_det = getFullDetect(image_msg, bounding_boxes[0]);
      detected_object_publisher_.publish(full_det);
      // detected_object_publisher_.publish(bounding_boxes[0]);
      ROS_INFO("Publishing %s with confidence %f",
               bounding_boxes[0].box.Class.c_str(),
               bounding_boxes[0].box.probability);
    }
    return;
  }
  for (int i = 0; i < size; i++) {
    for (int j = 0; j < size; j++) {
      // only publish the outer box
      if (!handled[i] &&
          bounding_boxes[i].box.xmin > bounding_boxes[j].box.xmin &&
          bounding_boxes[i].box.ymin > bounding_boxes[j].box.ymin &&
          bounding_boxes[i].box.xmax < bounding_boxes[j].box.xmax &&
          bounding_boxes[i].box.ymax < bounding_boxes[j].box.ymax) {
        if (bounding_boxes[i].box.probability >=
            bounding_boxes[j].box.probability) {
          // merge image and bbox
          darknet_ros_msgs::FullDetection full_det;
          full_det = getFullDetect(image_msg, bounding_boxes[i]);
          detected_object_publisher_.publish(full_det);
          // detected_object_publisher_.publish(bounding_boxes[i]);
          ROS_INFO("publishing %s %s with conf %f",
                   "outer",
                   bounding_boxes[i].box.Class.c_str(),
                   bounding_boxes[i].box.probability);
          ROS_INFO("skippinnnnng %s %s with conf %f",
                   "inner",
                   bounding_boxes[j].box.Class.c_str(),
                   bounding_boxes[j].box.probability);
        } else if (!handled[j]) {
          darknet_ros_msgs::FullDetection full_det;
          full_det = getFullDetect(image_msg, bounding_boxes[i]);
          detected_object_publisher_.publish(full_det);
          // detected_object_publisher_.publish(bounding_boxes[j]);
          ROS_INFO("publishing %s %s with conf %f",
                   "inner",
                   bounding_boxes[j].box.Class.c_str(),
                   bounding_boxes[j].box.probability);
          ROS_INFO("skippinnnnng %s %s with conf %f",
                   "outer",
                   bounding_boxes[i].box.Class.c_str(),
                   bounding_boxes[i].box.probability);
        }
        handled[i] = true;
        handled[j] = true;
        ROS_INFO("making i:%d and j:%d true", i, j);
      }
    }
    if (!handled[i]) {
      handled[i] = true;
      darknet_ros_msgs::FullDetection full_det;
      full_det = getFullDetect(image_msg, bounding_boxes[i]);
      detected_object_publisher_.publish(full_det);
      // detected_object_publisher_.publish(bounding_boxes[i]);
      ROS_INFO("unique i:%d", i);
      ROS_INFO("publishing unique %s with conf %f",
               bounding_boxes[i].box.Class.c_str(),
               bounding_boxes[i].box.probability);
    }
  }
}

bool EdgeTpuDetector::publishDetectionImage(
    const sensor_msgs::ImageConstPtr& image_msg,
    const darknet_ros_msgs::BoundingBoxes& bounding_boxes) {
  if (detection_image_publisher_.getNumSubscribers() < 1)
    return false;

  cv_bridge::CvImagePtr image_cv;

  try {
    image_cv =
        cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::RGB8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception in publishDetectionImage: %s", e.what());
    std::abort();
  }

  cv::Point top_left, bottom_right;
  for (darknet_ros_msgs::BoundingBox bb : bounding_boxes.bounding_boxes) {
    top_left.x = bb.xmin;
    top_left.y = bb.ymin;
    bottom_right.x = bb.xmax;
    bottom_right.y = bb.ymax;
    std::stringstream stream;
    stream << std::fixed << std::setprecision(0) << bb.Class.c_str() << ' '
           << bb.probability * 100 << '%';

    cv::rectangle(
        image_cv->image, top_left, bottom_right, cv::Scalar(0, 255, 0));
    cv::Rect text_rect = cv::Rect(top_left.x, bottom_right.y - 25, 105, 20);
    cv::rectangle(image_cv->image,
                  text_rect,
                  cv::Scalar(255, 255, 255),
                  cv::FILLED); // for the text
    cv::putText(image_cv->image,
                stream.str(),
                cv::Point(top_left.x, bottom_right.y - 10),
                0,
                0.5,
                cv::Scalar(0, 255, 0),
                1.5);
  }
  cv_bridge::CvImage cvImage;
  cvImage.header.stamp = image_msg->header.stamp;
  cvImage.header.frame_id = "detection_image";
  cvImage.encoding = sensor_msgs::image_encodings::RGB8;
  cvImage.image = image_cv->image;
  detection_image_publisher_.publish(*cvImage.toImageMsg());
  ROS_DEBUG("Detection image has been published.");
  return true;
}

} /* namespace coral*/
