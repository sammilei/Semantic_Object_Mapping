#!/usr/bin/env python

import sys
import rospy
import argparse
import importlib
from std_msgs.msg import Header


def main(args):
    rospy.init_node("dummy_artifact_detection_pub")

    # Parse args
    artifact = args.artifact
    artifact_id = args.artifact + "Artifact01"
    topic = args.topic
    topic_type = args.type
    frame_id = args.frame
    rate = args.rate
    signal_strength = args.signal_strength
    bounding_box = args.bounding_box
    probability = args.probability
    yolo_probability = args.yolo_probability
    color_score = args.color_score
    camera_name = args.camera_name

    # Dynamically import message class
    try:
        msg_module_name, msg_class_name = topic_type.split("/")
        msg_module = importlib.import_module(msg_module_name + ".msg")
        msg_class = getattr(msg_module, msg_class_name)
    except Exception as e:
        raise TypeError("Unsupported message type: {}. {}".format(topic_type, e))

    # Publish message
    pub = rospy.Publisher(topic, msg_class, queue_size=10)
    r = rospy.Rate(rate)
    while not rospy.is_shutdown():
        if msg_module_name == "artifact_msgs" and msg_class_name == "PointSourceDetection":
            msg = msg_class(
                header=Header(frame_id=frame_id, stamp=rospy.Time.now()),
                id=artifact_id,
                strength=signal_strength)
        elif msg_module_name == "darknet_ros_msgs" and msg_class_name == "Object":
            from darknet_ros_msgs.msg import BoundingBox
            msg = msg_class(
                header=Header(frame_id=frame_id, stamp=rospy.Time.now()),
                box=BoundingBox(Class=artifact,
                                probability=probability,
                                yolo_probability=yolo_probability,
                                color_score=color_score,
                                xmin=int(bounding_box[0]),
                                ymin=int(bounding_box[1]),
                                xmax=int(bounding_box[2]),
                                ymax=int(bounding_box[3])),
                camera_name=camera_name)
        else:
            raise TypeError("Unsupported message type: {}".format(topic_type))
        pub.publish(msg)
        r.sleep()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Dummy artifact detection publisher.")
    parser.add_argument("-a", "--artifact", default="Cube", type=str)
    parser.add_argument("-to", "--topic", default="bt_rssi", type=str)
    parser.add_argument("-ty", "--type", default="artifact_msgs/PointSourceDetection", type=str)
    parser.add_argument("-f", "--frame", default="world", type=str)
    parser.add_argument("-r", "--rate", default=10, type=int)
    parser.add_argument("-s", "--signal-strength", default=-39.0, type=float)
    parser.add_argument("-bb", "--bounding-box", default=[100, 100, 200, 200], nargs="+")
    parser.add_argument("-p", "--probability", default=1.0, type=float)
    parser.add_argument("-yp", "--yolo-probability", default=1.0, type=float)
    parser.add_argument("-cs", "--color-score", default=1.0, type=float)
    parser.add_argument("-cn", "--camera-name", default="camera_front", type=str)
    args, _ = parser.parse_known_args()

    main(args)
