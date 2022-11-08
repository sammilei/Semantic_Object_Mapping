#!/usr/bin/env python
"""Surrogate for DARPA Command Post Server."""

import os
import rospy
import json
import httplib
import threading
from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer

from darpa_scoring_system.scoring_system import ScoringSystem


AUTHENTICATION_TOKEN = None
CURRENT_TEAM = None


# This class will handle any incoming request from the browser
class SurrogateRequestHandler(BaseHTTPRequestHandler):
    """ DARPA Server Surrogate Handler for the GET and POST requests.
    """

    def is_token_authorized(self, token):
        """ Check authentication token """
        if token == AUTHENTICATION_TOKEN:
            return True
        return False

    def set_json_response(self, status_code, content):
        """ Set GET json response with specified status code and content """
        self.send_response(status_code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', len(content))
        self.send_header('Access-Control-Allow-Origin', '*')
        # self.send_header('Access-Control-Allow-Method',
        #                  'GET,HEAD,OPTIONS,POST,PUT')
        # self.send_header('Access-Control-Allow-Headers',
        #                  'Origin,X-Requested-With,Content-Type,Accept,Authorization')
        self.end_headers()

        # Send the html message
        self.wfile.write(content)

    def do_OPTIONS(self):
        self.send_response(200, "ok")
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Method',
                         'GET,HEAD,OPTIONS,POST,PUT')
        self.send_header('Access-Control-Allow-Headers',
                         'Origin,X-Requested-With,Content-Type,Accept,Authorization')

    def do_GET(self):
        """ Handle GET requests """
        # TEST GET: $ curl -i -X GET -H 'Authorization: Bearer subttesttoken123' http://localhost:8080/api/status
        rospy.logdebug("GET: %s", self.path)

        if self.path == '/api/status/':
            # check authentication token
            if 'Authorization' in self.headers:
                token = self.headers.getheader('Authorization').split()[1]
            else:
                token = ''

            if not self.is_token_authorized(token):
                response_content = 'Authentication token does not match expected token for this run (or was not provided).'
                self.set_json_response(status_code=httplib.UNAUTHORIZED,
                                       content=response_content)
                return

            # TODO: get status
            response_content_dict = {'score': self.server.scoring_system.get_score(),
                                     'run_clock': self.server.scoring_system.get_run_clock(),
                                     'remaining_reports': self.server.scoring_system.get_remaining_reports(),
                                     'current_team': CURRENT_TEAM}
            response_content = json.dumps(response_content_dict)
            self.set_json_response(status_code=httplib.OK,
                                   content=response_content)

        else:
            self.send_response(httplib.OK)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            # Send the html message
            self.wfile.write("Welcome to the DARPA Command Post Server Surrogate!")
        return

    def do_POST(self):
        """ Handle POST requests """
        # TEST POST: $ curl -i -X POST -H 'Authorization: Bearer subttesttoken123' -H 'Content-Type: application/json' -H 'Content-Length: 44' -d '{"x": 19.5, "y": 0, "z": 0, "type": "Drill"}' http://localhost:8080/api/artifact_reports

        rospy.logdebug("POST: %s", self.path)

        if self.path == '/api/artifact_reports/':

            # check authentication token
            if 'Authorization' in self.headers:
                token = self.headers.getheader('Authorization').split()[1]
            else:
                token = ''

            if not self.is_token_authorized(token):
                response_content = 'Unauthorized. Authentication token does not match expected token for this run (or was not provided).'
                self.set_json_response(status_code=httplib.UNAUTHORIZED,
                                       content=response_content)
                return

            # check cntent type. refuse to receive non-json content
            ctype = self.headers.getheader('Content-Type')
            if ctype != 'application/json':
                response_content = 'Wrong content type. Only JSON content is accepted'
                self.set_json_response(
                    status_code=httplib.UNPROCESSABLE_ENTITY, content=response_content)
                return

            # read the message and convert it into a python dictionary
            request_content_dict = {}
            try:
                length = int(self.headers.getheader('Content-Length'))
                request_content_dict = json.loads(self.rfile.read(length))
                rospy.loginfo('Artifact Report received: %s', request_content_dict)
            except Exception as e:
                response_content = 'Bad Request. JSON/CBOR parsing failed.'
                self.set_json_response(status_code=httplib.BAD_REQUEST,
                                       content=response_content)
                return

            # TODO: process report
            score, response_content_dict = \
                self.server.scoring_system.report_artifact(request_content_dict)
            rospy.loginfo('Current Score: %d', score)

            # send the message back
            response_content = json.dumps(response_content_dict)
            self.set_json_response(status_code=httplib.OK,
                                   content=response_content)

        elif self.path == '/api/reset/':
            # reset the run (clock and score)
            # $ curl -i -X POST -H 'Authorization: Bearer subttesttoken123' -H 'Content-Type: application/json' -H 'Content-Length: 2' -d '{}' http://localhost:8080/api/reset/
            self.server.scoring_system.reset_run()
            response_content = 'Run reset (including clock and score)!'
            self.set_json_response(status_code=httplib.OK,
                                   content=response_content)
            return


def main():
    global AUTHENTICATION_TOKEN, CURRENT_TEAM

    rospy.init_node('darpa_server_surrogate', disable_signals=True)

    hostname = rospy.get_param('~hostname', '0.0.0.0')
    port = rospy.get_param('~port', 8080)
    AUTHENTICATION_TOKEN = rospy.get_param('~token', 'subttesttoken123')
    CURRENT_TEAM = rospy.get_param('~team', 'costar')
    dataset_file = rospy.get_param('~dataset_file')

    darpa_scoring_system = None
    try:
        with open(dataset_file) as f:
            data = json.load(f)
    except IOError:
        rospy.logwarn("Dataset file not found: %s", dataset_file)
        data = {"artifacts": []}

    darpa_scoring_system = ScoringSystem(ground_truth_dataset=data)
    darpa_scoring_system.start_run()

    # Create a web server and define the handler to manage the
    # incoming request
    server = HTTPServer((hostname, port), SurrogateRequestHandler)
    server.scoring_system = darpa_scoring_system

    rospy.loginfo('Started httpserver on %s:%d', hostname, port)

    # Wait forever for incoming http requests
    server.serve_forever()

    # Gracefully shutdown ROS
    rospy.signal_shutdown("Shutdown requested")


if __name__ == '__main__':
    main()
