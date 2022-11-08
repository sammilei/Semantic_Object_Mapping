#!/usr/bin/env python
"""Generate artifact ground truth file from gazebo topic."""

import pprint
import json

import rospy
from gazebo_msgs.msg import ModelStates


def main():
    rospy.init_node("artifact_util")
    try:
        filename = rospy.myargv()[1]
    except:
        filename = None

    # Artifact specification
    gazebo_keywords = {
        "Cell Phone": ["phone"],
        "Backpack": ["backpack"],
        "Rope": ["rope"],
        "Helmet": ["helmet"],
        "Survivor": ["rescue_randy"],
    }

    def get_label(name):
        for label, keywords in gazebo_keywords.iteritems():
            for keyword in keywords:
                if keyword in name:
                    return label
        return None

    # Listen to gazebo model states
    msg = rospy.wait_for_message("/gazebo/model_states", ModelStates)

    # Parse artifact models
    artifacts = []
    for i, name in enumerate(msg.name):
        label = get_label(name)
        if label:
            artifacts.append(
                {
                    "type": label,
                    "x": msg.pose[i].position.x,
                    "y": msg.pose[i].position.y,
                    "z": msg.pose[i].position.z,
                    "id": name,
                }
            )

    # Write to output
    if filename:
        data = {
            "artifacts": artifacts,
        }
        with open(filename, "w") as f:
            print("Writing result to {}".format(filename))
            json.dump(data, f)
    else:
        pprint.pprint(artifacts)


if __name__ == "__main__":
    main()
