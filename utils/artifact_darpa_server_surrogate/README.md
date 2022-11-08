
DARPA Command Post Server Surrogate
====================

The Surrogate for the DARPA Command Post Server is an HTTP server that provides the same GET and POST functionality for artifact reporting, status check and mapping and telemetry reporting.


## Installation

### 1. Clone this repository into ROS workspace
```
git clone https://gitlab.robotics.caltech.edu/naisr/artifact/artifact_darpa_server_surrogate.git
```

### 2. Build
```
catkin build
```

## Execute

Start the web server
```
roslaunch darpa_server_surrogate server.launch
```
and access to `http://localhost:8080` with your favorite browser to check it if is up and running.


## Testing

You can test GET ('/api/status') and POST ('/api/artifact_reports') through command line using curl. As an example, run the following command from= a terminal to get status:
```
$ curl -i -X GET -H 'Authorization: Bearer subttesttoken123' http://localhost:8080/api/status/
```
The token can be changed if necessary.

In order to post an artifact report and get the results, use the following command example from a terminal:

```
$ curl -i -X POST -H 'Authorization: Bearer subttesttoken123' -H 'Content-Type: application/json' -d '{"x": 19.5, "y": 0, "z": 0, "type": "Drill"}' http://localhost:8080/api/artifact_reports/
```
