#!/usr/bin/env python

import rospy
from pose_graph_msgs.msg import KeyValue, PoseGraph
from artifact_msgs.msg import PointSourceDetection, KeyValueID
#from artifact_msgs.msg import PointSourceDetectionKey

key_value_pub = None
detection = None
latest_key = None

CO2_MIN_THRESH = 1500 # Mega cavern Final, ambient is < 1200, DARPA max is 3000
CO2_MAX_THRESH = 3000 # Mega cavern Final, ambient is < 1200, DARPA max is 3000

def format_key(key):
    c = chr(key >> 56)
    n = key & 0x00FFFFFF
    return "{}{}".format(c, n)


def detection_cb(msg):
    global detection, latest_key
    # gas: only report within range
    if 'CO2' in msg.id and msg.strength < CO2_MIN_THRESH:
        return
    if 'CO2' in msg.id and msg.strength > CO2_MAX_THRESH:
        msg.strength = CO2_MAX_THRESH
    if detection:  # only replace if the new one is stronger
        if msg.strength > detection.strength:
            detection = msg
    else:
        detection = msg
    # Publish with last known pose graph key

        # Reset key so we only publish one for a key
        # latest_key = None


def pose_graph_cb(msg):
    global detection, latest_key

    # Only update for odom nodes
    nodes = []
    for node in msg.nodes:
        if node.ID in ["key_frame", "odom_node"]:
            nodes.append(node)

    if not nodes:
        return

    # Check timeout
    if detection and rospy.Time.now() - detection.header.stamp > rospy.Duration(30):
        detection = None
        rospy.logwarn("Detection timeout")

    # No measurement available
    if detection is None:
        rospy.logwarn(
            "No measurement available for key %s",
            format_key(msg.nodes[0].key),
        )
        return

    # Use the latest measurement
    # TODO: Consider proper time sync
    out_msg = KeyValueID()
    out_msg.key = node.key
    out_msg.value = detection.strength
    out_msg.ID  = detection.id
    key_value_pub.publish(out_msg)
    rospy.loginfo(
        "Value added for key %s: %.2f", format_key(out_msg.key), out_msg.value
    )
    detection = None # Clean up to avoid publishing old signal in the next key scan

    # Store key for future detection message
    # latest_key = node.key


def main():
    global key_value_pub

    rospy.init_node("append_key")
    key_value_pub = rospy.Publisher("~output", KeyValueID, queue_size=10)
    strength_sub = rospy.Subscriber(
        "~input", PointSourceDetection, detection_cb, queue_size=1
    )
    pose_graph_sub = rospy.Subscriber(
        "lamp/pose_graph_incremental", PoseGraph, pose_graph_cb, queue_size=10
    )

    rospy.spin()


if __name__ == "__main__":
    main()
