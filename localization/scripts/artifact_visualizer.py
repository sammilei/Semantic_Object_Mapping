#!/usr/bin/env python
"""Convert artifact messages to visualizable forms."""

import rospy
import numpy as np
from tf import transformations as tfm

from artifact_msgs.msg import Artifact
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray


class ArtifactVisualizer(object):
    def __init__(self):
        self.artifact_count = {}
        self.min_number_to_publish = 2
        self.odom_pub = rospy.Publisher("~odom", Odometry, queue_size=1, latch=True)
        self.pose_pub = rospy.Publisher("~pose", PoseStamped, queue_size=1, latch=True)
        self.pose_cov_pub = rospy.Publisher(
            "~pose_with_covariance", PoseWithCovarianceStamped, queue_size=1, latch=True
        )
        self.markers_pub = rospy.Publisher(
            "~markers", MarkerArray, queue_size=1, latch=True
        )
        self.artifact_sub = rospy.Subscriber(
            "~artifact", Artifact, self.artifact_cb, queue_size=1
        )

    def artifact_cb(self, msg):
        # Check minimum number of reports before publishing
        self.artifact_count.setdefault(msg.id, 0)
        self.artifact_count[msg.id] += 1
        if self.artifact_count[msg.id] < self.min_number_to_publish:
            return

        # Publish messages
        self.publish_odom(msg)
        self.publish_pose(msg)
        self.publish_pose_with_covariance(msg)
        self.publish_markers(msg)

    def publish_odom(self, msg):
        if self.odom_pub.get_num_connections() == 0:
            return

        out = Odometry()
        out.header = msg.point.header
        out.pose.pose.position = msg.point.point
        out.pose.pose.orientation.w = 1
        out.pose.covariance = self._to_full_covariance(msg.covariance)
        self.odom_pub.publish(out)

    def publish_pose(self, msg):
        if self.pose_pub.get_num_connections() == 0:
            return

        out = PoseStamped()
        out.header = msg.point.header
        out.pose.position = msg.point.point
        out.pose.orientation.w = 1
        self.pose_pub.publish(out)

    def publish_pose_with_covariance(self, msg):
        if self.pose_cov_pub.get_num_connections() == 0:
            return

        out = PoseWithCovarianceStamped()
        out.header = msg.point.header
        out.pose.pose.position = msg.point.point
        out.pose.pose.orientation.w = 1
        out.pose.covariance = self._to_full_covariance(msg.covariance)
        self.pose_cov_pub.publish(out)

    def publish_markers(self, msg):
        if self.markers_pub.get_num_connections() == 0:
            return

        out = MarkerArray()
        out.markers.append(self._generate_covariance_marker(msg))
        out.markers.append(self._generate_measurement_marker(msg))
        out.markers.append(self._generate_label_marker(msg))
        self.markers_pub.publish(out)

    def _to_full_covariance(self, cov3x3):
        cov = [0] * 36
        cov[:3] = cov3x3[:3]
        cov[6:9] = cov3x3[3:6]
        cov[12:15] = cov3x3[6:]
        return cov

    def _generate_covariance_marker(self, msg):
        marker = Marker()
        marker.action = Marker.ADD
        marker.type = Marker.SPHERE
        marker.ns = "covariance"
        marker.id = int(hash(msg.id) % 1e8)

        marker.header = msg.point.header
        marker.pose.position = msg.point.point
        marker.pose.orientation.w = 1

        marker.color.r = 204.0 / 255.0
        marker.color.g = 51.0 / 255.0
        marker.color.b = 204.0 / 255.0
        marker.color.a = 0.5

        # Compute eigen values
        s, v = np.linalg.eig(np.array(msg.covariance).reshape(3, 3))
        v[:, 0] /= np.linalg.norm(v[:, 0])
        v[:, 1] /= np.linalg.norm(v[:, 1])
        v[:, 2] /= np.linalg.norm(v[:, 2])

        if np.cross(v[:, 0], v[:, 1]).dot(v[:, 2]) < 0:
            # Make it right-handed
            v = v[:, [1, 0, 2]]
            s[0], s[1] = s[1], s[0]

        # Set ellipse scale
        k = 2.296  # 68.26%
        # k = 11.82  # 99.74%
        marker.scale.x = k * np.sqrt(s[0])
        marker.scale.y = k * np.sqrt(s[1])
        marker.scale.z = k * np.sqrt(s[2])

        # Set ellipse orientation
        rot = np.eye(4)
        rot[:3, :3] = v
        quat = tfm.quaternion_from_matrix(rot)
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]
        return marker

    def _generate_measurement_marker(self, msg):
        marker = Marker()
        marker.action = Marker.ADD
        marker.type = Marker.LINE_LIST
        marker.ns = "measurement"
        marker.id = int(hash(msg.id) % 1e8)

        marker.header = msg.point.header
        marker.pose.orientation.w = 1
        for obs_point in msg.observation_points:
            marker.points.append(obs_point)
            marker.points.append(msg.point.point)

        marker.color.r = 1
        marker.color.g = 0.8
        marker.color.b = 0
        marker.color.a = 1
        marker.scale.x = 0.03

        return marker

    def _generate_label_marker(self, msg):
        marker = Marker()
        marker.action = Marker.ADD
        marker.type = Marker.TEXT_VIEW_FACING
        marker.ns = "label"
        marker.id = int(hash(msg.id) % 1e8)
        marker.text = "{} ({}%)".format(msg.label, int(100 * msg.confidence))

        marker.header = msg.point.header
        marker.pose.position = msg.point.point
        marker.pose.orientation.w = 1

        marker.color.r = 1
        marker.color.g = 1
        marker.color.b = 1
        marker.color.a = 1
        marker.scale.z = 0.8

        return marker


def main():
    rospy.init_node("artifact_visualizer")
    viz = ArtifactVisualizer()
    rospy.spin()


if __name__ == "__main__":
    main()
