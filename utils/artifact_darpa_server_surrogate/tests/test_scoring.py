#!/usr/bin/env python

import json
import unittest
import requests

import rospy
import rostest


class ScoringTest(unittest.TestCase):

    def setUp(self):
        rospy.loginfo("Wait for server to come up")
        rospy.wait_for_service('/darpa/server/get_loggers')
        rospy.loginfo("Server ready")

    def get_headers(self):
        return {
            'Content-Type': 'application/json',
            'Authorization': 'Bearer subttesttoken123'
        }

    def submit(self, data, headers=None):
        headers = headers or self.get_headers()
        url = 'http://localhost:8000/api/artifact_reports/'
        return requests.post(url, headers=headers, data=json.dumps(data))

    def test_handle_valid_report(self):
        data = {
            'type': 'Survivor',
            'x': 10.0,
            'y': 20.0,
            'z': 0.0,
        }
        result = self.submit(data).json()
        self.assertEquals(result['report_status'], 'scored')
        self.assertEquals(result['score_change'], 1)

    def test_handle_invalid_report(self):
        # Ground truth: Cell Phone (5, 0, 0)
        data = [{'type': 'Cell Phone', 'x': 11.0, 'y': 0.0, 'z': 0.0},
                {'type': 'Cell Phone', 'x': 5.0, 'y': -50.0, 'z': 0.0},
                {'type': 'Cell Phone', 'x': 5.0, 'y': 0.0, 'z': -6.0}]

        for d in data:
            r = requests.post('http://localhost:8000/api/artifact_reports/',
                              headers=self.get_headers(),
                              data=json.dumps(d))
            result = r.json()
            self.assertEquals(result['report_status'], 'scored')
            self.assertEquals(result['score_change'], 0)

    def test_should_not_accept_duplicate_reports(self):
        data = {
            'type': 'Backpack',
            'x': 10.0,
            'y': -10.0,
            'z': 0.0,
        }
        result = self.submit(data).json()
        self.assertEquals(result['report_status'], 'scored')
        self.assertEquals(result['score_change'], 1)

        # Should not score second submission
        result = self.submit(data).json()
        self.assertEquals(result['report_status'], 'scored')
        self.assertEquals(result['score_change'], 0)

    def test_should_reject_invalid_token(self):
        data = {
            'type': 'Backpack',
            'x': 0,
            'y': 0,
            'z': 0,
        }
        headers = self.get_headers()
        headers['Authorization'] = 'Bearer invalidtoken'
        response = self.submit(data, headers)
        self.assertFalse(response.ok)

    def test_should_reject_empty_token(self):
        data = {
            'type': 'Backpack',
            'x': 0,
            'y': 0,
            'z': 0,
        }
        headers = self.get_headers()
        del headers['Authorization']
        response = self.submit(data, headers)
        self.assertFalse(response.ok)


if __name__ == '__main__':
    rostest.rosrun('darpa_server_surrogate', 'test_scoring', ScoringTest)
