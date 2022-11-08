#!/usr/bin/env python

from __future__ import division
import cv2
import numpy as np
import os

import cv_bridge
from cv_bridge import CvBridge

import rospy
import sys
import yaml
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo, Image
from darknet_ros_msgs.msg import Object, FullDetection


def crop_img(img_cropped, lab1):
    h, w = img_cropped.shape[:2]

    if lab1 == 'fire_extinguisher':  # remove the top and bottom
        h_5 = max(1, int(0.05 * h))
        h_15 = max(1, int(0.15 * h))
        w_5 = max(1, int(0.05 * w))
        img_cropped = img_cropped[h_15:-h_5, w_5:-w_5]
    elif lab1 == 'backpack':
        h_5 = max(1, int(0.2 * h))
        w_5 = max(1, int(0.2 * w))
        img_cropped = img_cropped[h_5:-h_5, w_5:-w_5]
    elif lab1 == 'rope':
        h_5 = max(1, int(0.2 * h))
        w_5 = max(1, int(0.2 * w))
        img_cropped = img_cropped[h_5:-h_5, w_5:-w_5]
    elif lab1 == 'drill':
        h_5 = max(1, int(0.05 * h))
        h_20 = max(1, int(0.2 * h))
        w_5 = max(1, int(0.05 * w))
        img_cropped = img_cropped[h_5:-h_20, w_5:-w_5]
    return img_cropped


def color_seg_simple(bbox, label, hsv_min, hsv_max, min_h, min_w):

    # get rid of borders in the image
    img_cropped = bbox
    h_orig, w_orig = img_cropped.shape[:2]
    img_cropped = crop_img(img_cropped, label)

    h1, w1 = img_cropped.shape[:2]
    total_pixels = h1 * w1

    if h_orig < min_h or w_orig < min_w:
        percent = -1
        rospy.loginfo("Bounding box too small for color filter")
    else:
        hsv_img = cv2.cvtColor(img_cropped, cv2.COLOR_BGR2HSV)

        if label == 'rope':
            percent = blue_filtering_efficient(hsv_img, hsv_min, hsv_max, total_pixels)
            percent = min(100, 2.5 * percent)
        elif label == 'backpack':
            percent = red_filtering_efficient(hsv_img, hsv_min, hsv_max, total_pixels)
            percent = min(100, 1.4 * percent)
        elif label == 'fire_extinguisher':
            percent = red_filtering_efficient(hsv_img, hsv_min, hsv_max, total_pixels)
            percent = min(100, 3 * percent)
        elif label == 'drill':
            percent = red_filtering_efficient(hsv_img, hsv_min, hsv_max, total_pixels)
            percent = min(100, 2.5 * percent)
        else:
            percent = -1

    return percent


# do filtering for blue (or any other color that doesn't pass through 0 of H
def blue_filtering_efficient(hsv_img, hsv_low, hsv_high, total_pixels):
    mask = cv2.inRange(hsv_img, hsv_low, hsv_high)
    col_pixels = np.sum(np.sum(mask > 0))
    col_all = total_pixels
    percent = int(col_pixels / col_all * 100)
    return percent


def red_filtering_efficient(hsv_img, hsv_low, hsv_high, total_pixels):
    hMin, sMin, vMin = hsv_low
    hMax, sMax, vMax = hsv_high

    lower_red = np.array([hMin, sMin, vMin])
    middle_redb = np.array([179, sMax, vMax])
    middle_reda = np.array([0, sMin, vMin])
    upper_red = np.array([hMax, sMax, vMax])

    red_maska = cv2.inRange(hsv_img, middle_reda, upper_red)
    red_maskb = cv2.inRange(hsv_img, lower_red, middle_redb)
    mask = cv2.bitwise_or(red_maska, red_maskb)

    col_pixels = np.sum(np.sum(mask > 0))
    col_all = total_pixels
    percent = int((col_pixels / col_all) * 100)
    return percent


def combine_color_conf(col_score, yolo_probability):
    if col_score == -1:
        return yolo_probability
    else:
        #return (col_score / 100 + yolo_probability) / 2
        return col_score / 100 * yolo_probability


class ColorFilter:

    def __init__(self):
        print('Artifact detection filter')

        # arguments from launch file
        self.robot_name = rospy.get_param('~robot_name')
        self.use_color_filter = rospy.get_param('~use_color_filter')
        cfg_file = rospy.get_param('~config_file')

        # initializing other stuff
        self.bridge = cv_bridge.CvBridge()
        self.img_count = 0
        self.prev_cord = {'camera_front': [],
                          'camera_right': [],
                          'camera_left': [],
                          'camera_rear': [],
                          'camera_up': [],
                          'camera_top': []}

        self.prev_timestamp = {'camera_front': None,
                          'camera_right': None,
                          'camera_left': None,
                          'camera_rear': None,
                          'camera_up': None,
                          'camera_top': None}

        self.gazebo_keywords = {
            "Backpack": ["backpack"],
            "Rope": ["rope"],
            "Helmet": ["helmet"],
            "Survivor": ["rescue_randy"],
            "Fire Extinguisher": ["fire_extinguisher"],
            "Drill": ["drill"],
            "Cell Phone": ["cell phone"],
            "Cube": ["cube"]
        }
        # read config file
        with open(cfg_file , "r") as ymlfile:
            self.cfg = yaml.load(ymlfile)

        # subscribers rgb
        self.detection_subs_object = {}
        self.detection_pubs_object = {}
        topic_object_sub = "/" + self.robot_name + "/" + self.cfg['color_filter']['subscribers']['rgb_full_detection_topic']
        topic_object_pub = "/" + self.robot_name + "/" + self.cfg['color_filter']['publishers']['rgb_detection_topic']
        self.detection_subs_object[self.robot_name] = rospy.Subscriber(topic_object_sub, FullDetection, self.detection_object_listener_clbk)
        self.detection_pubs_object[self.robot_name] = rospy.Publisher(topic_object_pub, Object)
        self.all_detection_objects = []

        # subscribers thermal
        self.detection_subs_object_thermal = {}
        self.detection_pubs_object_thermal = {}
        topic_object_sub_thermal = "/" + self.robot_name + "/" + self.cfg['color_filter']['subscribers']['thermal_full_detection_topic']
        topic_object_pub_thermal = "/" + self.robot_name + "/" + self.cfg['color_filter']['publishers']['thermal_detection_topic']
        self.detection_subs_object_thermal[self.robot_name] = rospy.Subscriber(topic_object_sub_thermal, FullDetection,
                                                                               self.thermal_detection_object_listener_clbk)
        self.detection_pubs_object_thermal[self.robot_name] = rospy.Publisher(topic_object_pub_thermal, Object)

        # saving studd for debuging
        self.save_path = self.cfg['color_filter']['debug_features']['save_path']
        self.save_path_images = os.path.join(self.save_path, 'images')
        if not os.path.isdir(self.save_path):
            os.mkdir(self.save_path)
        if not os.path.isdir(self.save_path_images):
            os.mkdir(self.save_path_images)



    def get_hsv_thresh(self, label):
        if label == 'rope':
            hsv_min = np.array(self.cfg['color_filter']['filter_thresh']['BLUE_HSV_LOWER'])
            hsv_max = np.array(self.cfg['color_filter']['filter_thresh']['BLUE_HSV_UPPER'])
        elif label == 'backpack':
            hsv_min = np.array(self.cfg['color_filter']['filter_thresh']['RED_HSV_LOWER_BACKPACK'])
            hsv_max = np.array(self.cfg['color_filter']['filter_thresh']['RED_HSV_UPPER_BACKPACK'])
        elif label == 'fire_extinguisher':
            hsv_min = np.array(self.cfg['color_filter']['filter_thresh']['RED_HSV_LOWER_FIRE'])
            hsv_max = np.array(self.cfg['color_filter']['filter_thresh']['RED_HSV_UPPER_FIRE'])
        elif label == 'drill':
            hsv_min = np.array(self.cfg['color_filter']['filter_thresh']['RED_HSV_LOWER_DRILL'])
            hsv_max = np.array(self.cfg['color_filter']['filter_thresh']['RED_HSV_UPPER_DRILL'])
        else:
            hsv_min = None
            hsv_max = None

        min_h = self.cfg['color_filter']['filter_thresh']['MIN_H']
        min_w = self.cfg['color_filter']['filter_thresh']['MIN_W']

        return hsv_min, hsv_max, min_h, min_w


    def thermal_detection_object_listener_clbk(self, data):
        object_to_publish = data.bbox
        robot_name = data.robot_name
        if robot_name != self.robot_name:
            sys.exit('robot name do not match')
        object_to_publish.box.probability = data.bbox.box.yolo_probability
        self.detection_pubs_object_thermal[robot_name].publish(object_to_publish)


    def detection_object_listener_clbk(self, data):
        #print('entering the object callback')
        #print('camera', data.bbox.camera_name)
        #print('name', data.robot_name)
        #print('width', data.rgb_img.width, 'height', data.rgb_img.height)

        #get the image and the label
        label = self.gazebo_keywords[data.bbox.box.Class][0]
        img_cv = self.bridge.imgmsg_to_cv2(data.rgb_img, "bgr8")
        xmin = data.bbox.box.xmin
        ymin = data.bbox.box.ymin
        xmax = data.bbox.box.xmax
        ymax = data.bbox.box.ymax
        bb_scaling = 1.0/4.0
        data.bbox.box.xmin = int(data.bbox.box.xmin*bb_scaling)
        data.bbox.box.xmax = int(data.bbox.box.xmax*bb_scaling)
        data.bbox.box.ymin = int(data.bbox.box.ymin*bb_scaling)
        data.bbox.box.ymax = int(data.bbox.box.ymax*bb_scaling)
        camera_name = data.bbox.camera_name
        robot_name = data.robot_name
        timestamp = data.rgb_img.header.stamp
        if robot_name != self.robot_name:
            sys.exit('robot name do not match')
        if self.prev_timestamp[camera_name] is None:
            self.prev_timestamp[camera_name] = timestamp
        if self.prev_timestamp[camera_name] != timestamp:
            if len(self.prev_cord[camera_name]) > 10:
                self.prev_cord[camera_name].pop(0)
        new_cords = (xmin, xmax, ymin, ymax)
        if new_cords not in self.prev_cord[camera_name]:  # for an unknown reason I get multiple times the same detection, so just ignore
                                        # this fix still does not solve the problem happening when 2 objects seen
                                        # at the same time (it could be on 2 different cameras)
            self.prev_cord[camera_name].append(new_cords)
            # do the color filter
            if self.use_color_filter:
                hsv_min, hsv_max, min_h, min_w = self.get_hsv_thresh(label)
                col_score = color_seg_simple(img_cv[ymin:ymax, xmin:xmax], label, hsv_min, hsv_max, min_h, min_w)

                # draw the bbox on top of image
                if self.cfg['color_filter']['debug_features']['draw_and_save']:  # this is a debug feature
                    col = (0, 255, 0)
                    thickness = 1
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    fontScale = 0.3
                    color = (0, 255, 0)
                    start_point = (xmin, ymin)
                    end_point = (xmax, ymax)
                    cv2.rectangle(img_cv, start_point, end_point, col, thickness)
                    cv2.putText(img_cv, str(col_score) + ' - color', (xmin, ymax + 10), font, fontScale, color)
                    combined_score = combine_color_conf(col_score, data.bbox.box.yolo_probability)
                    cv2.putText(img_cv, str(combined_score) + ' - combined', (xmin, ymax + 30), font, fontScale, color)
                    save_path_img = os.path.join(self.save_path_images, str(self.img_count) + '.jpg')
                    self.img_count += 1
                    cv2.imwrite(save_path_img, img_cv)
                    print('save')
                    print(self.img_count)
                    # TODO save label file
                    # TODO save in detection report file
                    # NOTE: doing so will lead to multiple of very similar detections when to robot has
                    # not or almost not moved. By doing the same in the artifact_localisation node we can
                    # keep only the significantly different detections

                # overwrite the probability with the combination of color score and yolo confidence here
                # could change the object message in order to keep both info - maybe later
                object_to_publish = data.bbox
                combined_score = combine_color_conf(col_score, data.bbox.box.yolo_probability)
                object_to_publish.box.probability = combined_score
                object_to_publish.box.color_score = col_score
            else:
                object_to_publish = data.bbox
                object_to_publish.box.probability = data.bbox.box.yolo_probability
            self.detection_pubs_object[robot_name].publish(object_to_publish)

    # def publish(self):
    #     print('publishing')
    #     for o in self.all_detection_objects:
    #         print('published')
    #         robot_name = o[0]
    #         object_to_publish = o[1]
    #         self.detection_pubs_object[robot_name].publish(object_to_publish)
    #     self.all_detection_objects = []


def main():
    rospy.init_node('artifact_detection_filters')
    colfilter = ColorFilter()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        #colfilter.publish()
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
