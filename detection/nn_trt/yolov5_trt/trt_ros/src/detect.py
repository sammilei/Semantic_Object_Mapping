#!/usr/bin/env python

from __future__ import division

print(" ======= for debugging =======")
import cv_bridge
print(cv_bridge.__file__)
print(cv_bridge.boost.__file__)
print(" ======= end of debugging =======")

# Python imports
import numpy as np
import scipy.io as sio
import os, sys, cv2, time
from PIL import Image

# ROS imports
import rospy
import std_msgs.msg
from rospkg import RosPack
from std_msgs.msg import UInt8
from sensor_msgs.msg import Image as ros_Image
from sensor_msgs.msg import CompressedImage as ros_CompressedImage
from geometry_msgs.msg import Polygon, Point32
from cv_bridge import CvBridge, CvBridgeError

package = RosPack()
package_path = package.get_path('trt_ros')

from model import *

# Subt imports
from darknet_ros_msgs.msg import FullDetection, Object, BoundingBox, ObjectCount


# Detector manager class for YOLO
class Detector_manager():
    def __init__(self):
        # Load weights parameter
        namespace = rospy.get_name()
        engine_name = rospy.get_param(namespace + "/yolo_trt_model/serialized_file/name")
        self.engine_path = os.path.join(package_path, 'network_config', engine_name)
        rospy.loginfo("Found weights, loading %s", self.engine_path)

        if not os.path.isfile(self.engine_path):
            raise IOError(('{:s} not found.').format(self.engine_path))

        # get the robot name
        self.robot_name = rospy.get_param('~robot_name')

        # Load image parameter and confidence threshold
        self.class_labels = rospy.get_param(namespace + "/yolo_trt_model/detection_classes/names")
        conf_thresh_list = rospy.get_param(namespace + "/yolo_trt_model/detect_thresh/value")
        self.iou_thresh = rospy.get_param(namespace + "/yolo_trt_model/IoU_thresh/value", 0.4)
        self.conf_thresh_artifacts = {}

        rospy.loginfo("artifact specific confidence threshold")
        for ind, one_cls in enumerate(self.class_labels):
            self.conf_thresh_artifacts[one_cls] = conf_thresh_list[ind]

        for key, val in self.conf_thresh_artifacts.items():
            rospy.loginfo("{}: {}".format(key, val))

        # Load subscribers topics
        self.image_topics = rospy.get_param(namespace + '/subscribers/camera_topics/name')
        self.camera_queue_size = rospy.get_param(namespace + "/subscribers/camera_topics/queue_size", 20)

        # Load publishers topics
        self.detection_image_topic_name = rospy.get_param(namespace + "/publishers/detection_image/topic", "detection_image")
        self.detection_image_queue_size = rospy.get_param(namespace + "/publishers/detection_image/queue_size", 1)
        self.detection_image_latch = rospy.get_param(namespace + "/publishers/detection_image/latch", True) 
        self.detected_object_topic_name = rospy.get_param(namespace + "/publishers/detected_object/topic", "detected_object_raw")
        self.detected_object_queue_size = rospy.get_param(namespace + "/publishers/detected_object/queue_size", 20)
        self.detected_object_latch = rospy.get_param(namespace + "/publishers/detected_object/latch", True) 

        # Load net
        self.model = TRTmodel(self.engine_path, self.iou_thresh, min(self.conf_thresh_artifacts.values()))
        # if not torch.cuda.is_available():
            # raise IOError('CUDA not found.')
        
        self.network_img_height, self.network_img_width = self.model.get_dim()
        rospy.loginfo("Neural network loaded: {}, {}".format(self.network_img_width, self.network_img_height))

        # self.inferThread = inferThread(self.model) # start a thread

        # Set up batch
        self.batch_size = self.model.batch_size
        self.batch_ros_img = []
        self.batch_processed = []
        self.batch_header = []
        self.batch_camera_name = []

        # param for img scaling
        self.scale = 0
        self.new_h = 0
        self.new_w = 0
        self.offset_h = 0
        self.offset_w = 0

        self.detection_lock = False

        # other parameter
        self.classes_colors = {}
        color = [(0, 255, 0), (0, 255, 255), (255, 0, 127), (255, 125, 255), (255, 255, 0), (127, 0, 255), (127, 127, 255)]
        for i in range(len(self.class_labels)):
            self.classes_colors[self.class_labels[i]] = color[i]
        self.detection_image_rescale = rospy.get_param(namespace + "publishers/detection_image/resize_scale", 0.5)
        self.to_publish_detection_image = rospy.get_param(namespace + "publishers/detection_image/enable", True)

        # Define publishers
        # detection image
        self.detection_image_pub = {}
        self.detection_image_compressed_pub = {}
        if self.to_publish_detection_image:
            for image_topic in self.image_topics:
                camera = image_topic.split('/')[0]
                detection_image_topic = camera + '/' + self.detection_image_topic_name
                self.detection_image_pub[camera] = rospy.Publisher(detection_image_topic, ros_Image, queue_size=1)
                self.detection_image_compressed_pub[camera] = rospy.Publisher(detection_image_topic + "/compressed", ros_CompressedImage, queue_size=1)
            
        self.detection_count_debug_pub = rospy.Publisher(namespace + "/detection_cnt_per_img", ObjectCount, queue_size=1)
        self.object_detected_pub = rospy.Publisher(self.detected_object_topic_name, FullDetection, queue_size=5)

        # Define subscribters
        self.subscribers = []
        for image_topic in self.image_topics:
            subscribe = True
            camera = image_topic.split('/')[0]
            if camera == "camera_front":
                cam_sub = rospy.Subscriber(image_topic, ros_Image, self.frontRgbCallback, queue_size = self.camera_queue_size)
            elif camera == "camera_right":
                cam_sub = rospy.Subscriber(image_topic, ros_Image, self.rightRgbCallback, queue_size = self.camera_queue_size)
            elif camera == "camera_left":
                cam_sub = rospy.Subscriber(image_topic, ros_Image, self.leftRgbCallback, queue_size = self.camera_queue_size)
            elif camera == "camera_up":
                cam_sub = rospy.Subscriber(image_topic, ros_Image, self.upRgbCallback, queue_size = self.camera_queue_size)
            elif camera == "camera_rear":
                cam_sub = rospy.Subscriber(image_topic, ros_Image, self.rearRgbCallback, queue_size = self.camera_queue_size)
            elif camera == "boson":
                cam_sub = rospy.Subscriber(image_topic, ros_Image, self.thermalCallback, queue_size = self.camera_queue_size)
            else:
                rospy.logerr("camera name unexpected, cannot process the yolov5 detection")
                subscribe = False
            if subscribe:
                self.subscribers.append(cam_sub)
        
        rospy.loginfo("Launched node for object detection")
        print("num of subscribers: {}".format(len(self.subscribers)))
        print("model batch size: {}".format(self.batch_size))
        print("class name: {}".format(self.class_labels))

        # load cv_bridge
        self.bridge = CvBridge()
        print("cv_bridge is loaded?", self.bridge != None)

        rospy.loginfo("detection image is published: {}".format(self.to_publish_detection_image))
        # Spin
        rospy.spin()


    def frontRgbCallback(self, image_data):
        self.imageFunction(image_data, "camera_front")

    def rightRgbCallback(self, image_data):
        self.imageFunction(image_data, "camera_right")

    def leftRgbCallback(self, image_data):
        self.imageFunction(image_data, "camera_left")

    def upRgbCallback(self, image_data):
        self.imageFunction(image_data, "camera_up")

    def rearRgbCallback(self, image_data):
        self.imageFunction(image_data, "camera_rear")

    def thermalCallback(self, image_data):
        self.imageFunction(image_data, "boson")

    def imageFunction(self, image_data, camera_name_image):
        # Convert the image to OpenCV
        if len(self.batch_ros_img) < self.batch_size:
            input_img, self.scale, self.new_h, self.new_w = self.preProcess(image_data)

            self.offset_w = (self.network_img_width - self.new_w)//2
            self.offset_h = (self.network_img_height - self.new_h)//2

            self.batch_ros_img.append(image_data)
            self.batch_processed.append(input_img)
            self.batch_header.append(image_data.header)
            self.batch_camera_name.append(camera_name_image)
        elif not self.detection_lock:
            self.detection_lock = True
            #print ("batch is full as {}, detect now".format(self.batch_size))
            # Get detections from network
            batch = self.batch_processed[:self.batch_size]
            try:
                batch = np.concatenate(batch, 0)
                detections = self.model.run(batch)

                # Parse detections
                for i, detection in enumerate(detections):
                    detection_results = []
                    camera_name = self.batch_camera_name[i]
                    if len(detection) > 0:
                        for det in detection:
                            # Get xmin, ymin, xmax, ymax, confidence and class
                            xmin_orig, xmax_orig, ymin_orig, ymax_orig, conf, det_class = self.processDetectionResult(det)

                            xmin_orig, xmax_orig, ymin_orig, ymax_orig = self.validBbox(xmin_orig, xmax_orig, ymin_orig,
                                                                                        ymax_orig,
                                                                                        self.batch_ros_img[i].height,
                                                                                        self.batch_ros_img[i].width)

                            # Populate darknet message
                            if conf > self.conf_thresh_artifacts[self.class_labels[int(det_class)]]:
                                detected_obj = self.publishDetection(self.batch_ros_img[i], xmin_orig, xmax_orig, ymin_orig,
                                                                    ymax_orig, conf, det_class, i, camera_name)

                                # Append in overall detection message
                                detection_results.append(detected_obj)

                    # show detection rate using a counter
                    detection_count_debug = ObjectCount()
                    detection_count_debug.header = self.batch_header[i]
                    detection_count_debug.count = len(detection_results)
                    self.detection_count_debug_pub.publish(detection_count_debug)

                    # Visualize images with detection only
                    if (self.to_publish_detection_image and len(detection_results) > 0):
                        self.publishDetectionImage(detection_results, self.batch_ros_img[i], camera_name)
            except Exception as e:
                print("batch lock conflict: {}".format(e))

            self.batch_ros_img = []
            self.batch_processed = []
            self.batch_header = []
            self.batch_camera_name = []
            self.detection_lock = False

    def validBbox(self, xmin_orig, xmax_orig, ymin_orig, ymax_orig, img_height, img_width):
        if (xmin_orig<0 or ymin_orig<0 or xmax_orig>img_width or ymax_orig>img_height):
            rospy.loginfo("bounding box not coherent - clipping to image size")
        xmin_orig = min(max(xmin_orig, 0), img_width - 2)
        ymin_orig = min(max(ymin_orig, 0), img_height - 2)
        xmax_orig = max(1, min(xmax_orig, img_width - 1))
        ymax_orig = max(1, min(ymax_orig, img_height - 1))

        if xmin_orig>xmax_orig:
            rospy.logerr("xmax smaller than xmin")
        if ymin_orig>ymax_orig:
            rospy.logerr("ymax smaller than ymin")
        return xmin_orig, xmax_orig, ymin_orig, ymax_orig


    def preProcess(self, ros_img):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_img, 'rgb8')#desired_encoding="passthrough")
            if ros_img.encoding == 'rgb8': #for raw input, decompressed:rgb8 by default
                print("this is rgb8 in the if statement")
                cv_image = cv2.cvtColor(cv_image,cv2.COLOR_BGR2BRG)
            img_in = np.array(Image.fromarray(cv_image))

            resized, scale, (new_h, new_w) = resize_img_no_PIL(img_in, (self.network_img_height, self.network_img_width))

            input_img = np.transpose(resized, (2, 0, 1)).astype(np.float32)  # HWC -> CHW
            input_img = np.expand_dims(input_img, axis=0)/255.0
            input_img = np.array(input_img)
            return input_img, scale, new_h, new_w

        except CvBridgeError as e:
            print(e)
            return None



    def processDetectionResult(self, det):
         # Get xmin, ymin, xmax, ymax, confidence and class
        xmin, ymin, xmax, ymax, conf, det_class = det
        xmin = int(xmin.item())
        ymin = int(ymin.item())
        xmax = int(xmax.item())
        ymax = int(ymax.item())
        conf = conf.item()
        det_class = det_class.item()

        # rescale the bbox
        xmin_orig = (xmin - self.offset_w) // self.scale
        xmax_orig = (xmax - self.offset_w) // self.scale
        ymin_orig = (ymin - self.offset_h) // self.scale
        ymax_orig = (ymax - self.offset_h) // self.scale
        return xmin_orig, xmax_orig, ymin_orig, ymax_orig, conf, det_class


    def publishDetection(self, image_data, xmin_orig, xmax_orig, ymin_orig, ymax_orig, conf, det_class, index_in_batch, camera_name):
        detected_obj = Object()
        detected_obj.header = self.batch_header[index_in_batch]
        detected_obj.camera_name = camera_name
        detected_obj.box.xmin = xmin_orig
        detected_obj.box.xmax = xmax_orig
        detected_obj.box.ymin = ymin_orig
        detected_obj.box.ymax = ymax_orig
        detected_obj.box.yolo_probability = conf
        detected_obj.box.color_score = -1
        detected_obj.box.Class = self.class_labels[int(det_class)]

        rospy.loginfo("{} detected: {} at {} in bbox: ({}, {} {},{})".format(camera_name, self.class_labels[int(det_class)], str(conf)[:4], xmin_orig, xmax_orig, ymin_orig, ymax_orig))

        full_detection = FullDetection()
        full_detection.bbox = detected_obj
        full_detection.rgb_img = image_data
        full_detection.robot_name = self.robot_name

        self.object_detected_pub.publish(full_detection)
        return detected_obj


    def publishDetectionImage(self, bounding_boxes, imgIn, camera_name):
        cv_image = self.bridge.imgmsg_to_cv2(imgIn, "rgb8")
        imgOut = cv_image.copy()
        font = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 0.5
        thickness = 2
        for index in range(len(bounding_boxes)):
            x_p1 = bounding_boxes[index].box.xmin
            y_p1 = bounding_boxes[index].box.ymin
            x_p3 = bounding_boxes[index].box.xmax
            y_p3 = bounding_boxes[index].box.ymax
            label = bounding_boxes[index].box.Class
            confidence = bounding_boxes[index].box.yolo_probability

            # Find class color
            if label in self.classes_colors.keys():
                color = self.classes_colors[label]
            else:
                # Generate a new color if first time seen this label
                color = np.random.randint(0,255,3)

                self.classes_colors[label] = color

            # Create rectangle
            cv2.rectangle(imgOut, (int(x_p1), int(y_p1)), (int(x_p3), int(y_p3)), (int(color[0]),int(color[1]),int(color[2])),thickness)
            text = ('{:s}:{:.0f}%').format(label[:2],confidence*100)
            cv2.putText(imgOut, text, (int(x_p1), int(y_p1+20)), font, fontScale, (255,255,255), thickness ,cv2.LINE_AA)

        # Publish visualization image
        imgOut = cv2.resize(imgOut, (int(imgOut.shape[1] * self.detection_image_rescale), int(imgOut.shape[0] * self.detection_image_rescale)), interpolation=cv2.INTER_LINEAR)
        imgOut = cv2.cvtColor(imgOut, cv2.COLOR_BGR2RGB)
        # image_msg = self.bridge.cv2_to_imgmsg(imgOut, "rgb8")
        image_msg = self.bridge.cv2_to_imgmsg(imgOut)
        compressed_image_msg = self.bridge.cv2_to_compressed_imgmsg(imgOut)
        self.detection_image_pub[camera_name].publish(image_msg)
        self.detection_image_compressed_pub[camera_name].publish(compressed_image_msg)


if __name__=="__main__":
    # Initialize node
    rospy.init_node("trt_ros")
    now = rospy.get_rostime()
    rospy.loginfo("Current time: %i %i", now.secs, now.nsecs)
    # Define detector object
    dm = Detector_manager()
