#!/usr/bin/env python
"""Surrogate for DARPA Scoring Algorithm/Manager"""

import sys
import time
import json
import yaml
import uuid
import math
import datetime
import pytz
from scipy.spatial import distance

import rospy


class ScoringSystem(object):
    """ Scoring Manager and Algorithm for each run

        Attributes:
        - ground_truth_dataset: (dict) contains a list of known artifacts
                                and their respective location (x,y,z).
                                It also includes their status (found, not_found)
        - reported_artifacts:   (list) list of reports, with their status, time, etc
    """

    def __init__(self, ground_truth_dataset=None, team='costar'):
        self.ground_truth_dataset = ground_truth_dataset
        self.reported_artifacts = []
        self.allowable_error_range = 5.0  # meters
        self.team = team
        self.run_id = 1
        self.run_status = None
        self.run_start_time = None

    def start_run(self):
        self.run_status = 'started'
        self.run_start_time = time.time()

    def reset_run(self):
        self.reported_artifacts = []
        self.run_id = 1
        self.start_run()

    def get_run_clock(self):
        return float(format(time.time() - self.run_start_time, '.1f'))

    def get_score(self):
        """Compute the score based on the artifacts found in the
           ground_truth_dataset.
        """
        # simple scoring algorithm: couting items scored/found
        # TODO: implement time-based scoring algorithm
        return self.run_counting_based_scoring_algorithm()

    def get_remaining_reports(self):
        """Compute how many score reports a team has remaining
        """
        remaining_reports = 0
        for artifact in self.ground_truth_dataset['artifacts']:
            if 'found' not in artifact.keys() or not artifact['found']:
                remaining_reports += 1
        return remaining_reports

    def report_artifact(self, artifact):
        """Check artifact against ground_truth_dataset and return
           1) score, 2) and a dict with the following info:
            a) wehther the score changed, b) report status
           (i.e., "scored", "admin stop", "run not started",
           "report limit exceeded", "time limit exceeded"),
           c) submitted time, d) the id of the reported
        """
        score_change = 0
        report_status = 'scored'
        reported_location = (artifact["x"], artifact["y"], artifact["z"])

        # get the absolute time of the report submission. ISO 8601 combined date and time format
        utc_now = pytz.utc.localize(datetime.datetime.utcnow())
        pst_now = utc_now.astimezone(pytz.timezone("America/Los_Angeles"))
        submitted_datetime = pst_now.isoformat()

        # check dataset to see if this artifact was correctly found/detected
        for truth_artifact in self.ground_truth_dataset['artifacts']:
            # check if the type matches
            if 'found' not in truth_artifact.keys() \
                    and artifact['type'].lower() == truth_artifact["type"].lower():
                # check position/point
                truth_location = (truth_artifact["x"],
                                  truth_artifact["y"],
                                  truth_artifact["z"])
                error_range = distance.euclidean(
                    truth_location, reported_location)
                # if the point is in the allowable error range, then scored and
                #  and mark truth artifact as found
                if error_range <= self.allowable_error_range:
                    score_change = 1
                    truth_artifact['found'] = True
                    rospy.loginfo('(HOORAY!) Artifact %s found (error=%.2f[m]). '
                                  'Compored to ground truth artifact id=%s',
                                  artifact['type'], error_range, truth_artifact['id'])
                else:
                    rospy.logwarn('(X) Artifact %s found outside the allowable error range (error=%.2f[m]). '
                                  'Compored to ground truth artifact id=%s',
                                  artifact['type'], error_range, truth_artifact['id'])

        report_id = len(self.reported_artifacts) + 1
        result = {'uri': 'http://localhost:8080/api/artifact_reports/{}'.format(report_id),
                  'id': report_id,
                  'submitted_datetime': submitted_datetime,
                  'run_clock': self.get_run_clock(),
                  'team': self.team,
                  'run': self.run_id,
                  'report_status': report_status,
                  'score_change': score_change}
        result.update(artifact)
        self.reported_artifacts.append(result)

        score = self.get_score()

        return score, result

    # SCORING ALGORITHMS
    def run_counting_based_scoring_algorithm(self):
        """Simple scoring algorithm: counts the number of artifacts
           found/reported correctly
        """
        score = 0
        for reported_artifact in self.reported_artifacts:
            if reported_artifact['score_change'] == 1:
                score += 1
        return int(score)
