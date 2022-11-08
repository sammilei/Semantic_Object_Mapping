#!/usr/bin/env python
"""Utility functions for generating and evaluating data about artifact localization performance."""

import bson
import pandas as pd
import subprocess
import os
import math
import yaml
import copy
import json
import rospkg
import csv
import numpy as np
import rospy
import rosbag
import datetime
from tf import transformations as tfs
np.set_printoptions(precision=3, suppress=True)
rospack = rospkg.RosPack()


class CommonUtils(object):
    def __init__(self, params):
        self.params = params
        self.datasets_path = self.params['datasets_path']
        self.artifact_placement = self.get_artifact_placement()

    def get_artifact_placement(self):
        """Return dictionary of {placement id: artifact type}"""
        with open(self.datasets_path + self.params['artifact_placement_file'], 'r') as infile:
            return yaml.safe_load(infile)

    def get_artifact_type_by_placement_id(self, placement_id):
        """Return string of artifact type corresponding to a placement ID"""
        # print 'art type by pid', self.artifact_placement[placement_id]
        return self.artifact_placement[placement_id]

    def get_surveyed_coords_ts(self):
        """Return dictionary of {placement id: [x,y,z]} in the ts frame"""
        with open(self.datasets_path + self.params['surveyed_gt_file'], 'r') as infile:
            return yaml.safe_load(infile)

    def get_placement_id_from_coord_w(self, coord_w):
        coord_ts_calc = tfs.translation_from_matrix(tfs.concatenate_matrices(
            tfs.inverse_matrix(T), tfs.translation_matrix(coord_w)))  # TODO check
        for id, coord_ts_orig in self.get_surveyed_coords_ts().items():
            # threshold for numerical error
            if self.distance_two_points(coord_ts_calc, coord_ts_orig) < 0.1:
                return id

    def get_ts_w_transformation(self):
        """Get transformation between ts and w frames in the form of a 4x4 homogeneous matrix"""
        # Read relative pose from file
        filename = os.path.expanduser('~/.ros/gt_w_transformation.yaml')
        print 'Try to load calibration data from:', filename
        if os.path.exists(filename):
            with open(filename) as f:
                data = yaml.load(f)
        else:
            data = {
                'position': {
                    'x': 0,
                    'y': 0,
                    'z': 0,
                },
                'orientation': {
                    'x': 0,
                    'y': 0,
                    'z': 0,
                    'w': 1,
                },
            }

        trans = tfs.translation_matrix(
            [data['position']['x'], data['position']['y'], data['position']['z']])
        rot = tfs.quaternion_matrix([data['orientation']['x'], data['orientation']
                                     ['y'], data['orientation']['z'], data['orientation']['w']])
        T = tfs.concatenate_matrices(trans, rot)
        return T

    def distance_two_points(self, pt1, pt2):
        """Returns Euclidean distance between two points pt1=[x1,y1,z1] and pt2=[x2,y2,z2]"""
        return math.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2 + (pt1[2] - pt2[2])**2)

    def timestamp_double_from_rospy_time(self, rospy_time):
        return rospy_time.secs + rospy_time.nsecs / 1e9

    def transform_ts_to_w(self, points):
        """Returns dictionary of {artifact type: [x,y,z]}, points transformed from ts to w frame"""
        points_transformed = {}
        T = self.get_ts_w_transformation()
        for artifact_type, coord_ts in points.items():
            pos = tfs.translation_from_matrix(tfs.concatenate_matrices(
                T, tfs.translation_matrix(coord_ts)))
            points_transformed.update(
                {artifact_type: [pos[0], pos[1], pos[2]]})
        return points_transformed

    def get_ts_coords_of_placed_artifacts(self):
        """Returns dictionary of {artifact type: [x,y,z]}, expressed in ts frame"""
        placed_artifacts = self.artifact_placement
        surveyed_gt = self.get_surveyed_coords_ts()
        ts_coords = {}
        for placement_id, artifact_type in placed_artifacts.items():
            ts_coords.update({artifact_type: surveyed_gt[placement_id]})
        return ts_coords

    def split_underscore(self, text):
        return text.split('_')[0]

    def get_key(self, my_dict, val):
        for key, value in my_dict.items():
            if val == value: 
                return key


class GenerateGtJson(CommonUtils):
    def __init__(self, params):
        CommonUtils.__init__(self, params)
        self.write_to_json(self.generate_json_data(
            self.transform_ts_to_w(self.get_ts_coords_of_placed_artifacts())))

    def generate_json_data(self, transformed_points):
        to_append_template = {"id": 0, "type": 0, "x": 0, "y": 0, "z": 0}
        to_append_list = []
        json_data = {"artifacts": []}
        id = 0  # does not correlate with placement id. Unique, ascending from 0
        # Populate list of artifact metadata
        for artifact_type, point in transformed_points.items():
            to_append = copy.deepcopy(to_append_template)
            to_append["id"] = self.get_key(self.artifact_placement, artifact_type)
            to_append["type"] = self.split_underscore(artifact_type)
            to_append["x"] = point[0]
            to_append["y"] = point[1]
            to_append["z"] = point[2]
            print 'Artifact location in w frame:', to_append
            to_append_list.append(to_append)
        json_data["artifacts"] = to_append_list
        return json_data

    def write_to_json(self, json_data):
        # Write to JSON
        with open(self.datasets_path + self.params['gt_json_file'], 'w+') as outfile:
            json.dump(json_data, outfile)
        print 'JSON file written!'

class Node(object):
    '''
    Simple class representing a pose graph node.
    Only stores x, y, z and id.
    '''

    def __init__(self, string=''):

        self.id = 0
        self.x = 0
        self.y = 0
        self.z = 0
        self.stamp = 0.0

        if string == '':
            return

        data = string.split()

        self.id = int(data[1])
        self.x = float(data[2])
        self.y = float(data[3])
        self.z = float(data[4])

    def InitFromNodeMsg(self, node):
        self.id = node.ID
        self.x = node.pose.position.x
        self.y = node.pose.position.y
        self.z = node.pose.position.z
        self.stamp = node.header.stamp.secs + node.header.stamp.nsecs / 1e9

class GenerateSummaryReport(CommonUtils):
    def __init__(self, params):
        CommonUtils.__init__(self, params)

        # Extract dictionary of ground truth placed artifacts, mapping placement ID to coordinates in world frame
        self.artifact_gt_w_from_json = self.get_artifact_gt_w_from_json()
        self.sorted_placement_ids = sorted(self.artifact_gt_w_from_json.keys())

        # Extract dictionary of observed artifacts, including all the metadata associated with the observation
        self.artifacts_db = self.read_database_bson()
        # self.artifacts_db = self.read_database_json() # this only for debugging!

        # Prepare panda DataFrame
        df_template = pd.DataFrame({'Placement ID': [], 'Artifact type GT': [], 'Artifact UUID': [], 'Artifact type obs': [], 'Min distance [m]': [
        ], 'Position error [m]': [], 'Distance traversed [m]': []})
        # https://stackoverflow.com/questions/41968732/set-order-of-columns-in-pandas-dataframe
        self.pd_column_titles = ['Placement ID', 'Artifact type GT', 'Artifact UUID', 'Artifact type obs',
                                 'Min robot offset [m]', 'Position error [m]', 'Distance traversed [m]']
        self.df_template = df_template.reindex(columns=self.pd_column_titles)

        # Read in the names of the robots to be reported on
        self.robots = self.params['robots'].keys()

        self.pose_graph_nodes = self.reconstruct_pose_graph()

        # self.get_distance_traversed_at_time(1574839517.75, 'husky3')

        # Get where each robot drove
        self.poses = self.get_lamp_poses()

        # Do the actual report generation
        self.generate_summary_report()

    def get_artifact_gt_w_from_json(self):
        """Return dictionary of {placement id: [x,y,z]} in w frame read in from the json
        This is for ground truth placed artifacts, NOT observed ones"""
        artifact_coords = {}
        with open(self.datasets_path + self.params['gt_json_file'], 'r') as infile:
            data = json.load(infile)
            for artifact in data['artifacts']:
                artifact_coords.update(
                    {artifact['id']: [artifact['x'], artifact['y'], artifact['z']]})
        return (artifact_coords)

    def generate_summary_report(self):
        """The steps in report generation"""
        dfs = self.update_df_columns()
        self.print_results(dfs)

    def get_lamp_poses(self):
        """Return list of [x,y,z] in w frame of where a given robot drove"""
        """Used to calculate how far it was from each artifact"""
        """(and hence whether or not it had a chance to observe it)"""
        poses = {}

        for robot in self.robots:
            bagpath = self.params['robots'][robot]
            bagfiles = [x for x in os.listdir(bagpath) if not x.find(
                'lamp') == -1]  # find the lamp bags, which contain pose information
            for f in bagfiles:
                bag = rosbag.Bag(bagpath + '/' + f)
                p = []
                for topic, msg, t in bag.read_messages(topics='/'+robot+'/lamp/lamp_pose'):
                    p.append([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
            poses.update({robot: p})
        return poses

    def update_df_columns(self):
        """Updates one column at a time"""
        dfs = {}
        for robot in self.robots:
            # Generate table of how far each observed artifact was from each GT artifact
            distance_pairs = self.get_all_distance_pairs(
                robot)  # seen by robotN

            # Get a dictionary of which GT artifacts correspond to which observed artifacts
            # A GT artifact may not have been observed
            assoc = self.associate_database_records_to_placed_artifacts(
                distance_pairs)

            df = copy.deepcopy(self.df_template)
            df['Placement ID'] = self.update_placement_id_col()
            df['Artifact type GT'] = self.update_artifact_type_col()
            df['Artifact UUID'] = self.update_artifact_uuid_col(assoc)
            df['Artifact type obs'] = self.update_artifact_type_obs_col(assoc)
            # df['Scored?'] = self.update_scored_col(assoc)
            df['Min robot offset [m]'] = self.update_min_dist_col(robot)
            df['Position error [m]'] = self.update_pos_error_col(assoc)
            df['Distance traversed [m]'] = self.update_distance_traversed_col(robot, assoc)
            df['Position error [%]'] = self.update_pos_error_perc_col(df['Position error [m]'], df['Distance traversed [m]'])
            dfs.update({robot: df})
        return dfs

    def reconstruct_pose_graph(self):
        pose_graph_nodes = {}
        for robot in self.robots:
            bagpath = self.params['robots'][robot]
            bagfiles = [x for x in os.listdir(bagpath) if not x.find(
                'lamp') == -1]  # find the lamp bags, which contain pose information
            for f in bagfiles:
                bag = rosbag.Bag(bagpath + '/' + f)
                nodes = []
                for topic, msg, t in bag.read_messages(topics='/'+robot+'/lamp/pose_graph_incremental'):
                    for n in msg.nodes:
                        if n.ID == 'odom_node':
                            new_node = Node()
                            new_node.InitFromNodeMsg(n)
                            nodes.append(new_node)
            pose_graph_nodes.update({robot: nodes})
        return pose_graph_nodes

    def get_distance_traversed_at_time(self, timestamp, robot):
        ''' timestamp [double] '''
        pose_graph_nodes = self.pose_graph_nodes[robot]
        distance = 0.0
        index = 0
        n_prev = pose_graph_nodes[index]
        n_curr = n_prev
        while n_curr.stamp < timestamp and index < len(pose_graph_nodes) - 1:
            distance += self.distance_two_points([n_prev.x, n_prev.y, n_prev.z], [n_curr.x, n_curr.y, n_curr.z])
            n_prev = n_curr
            index += 1
            n_curr = pose_graph_nodes[index]
        return distance

    def update_distance_traversed_col(self, robot, assoc):
        """TODO"""
        distance_traversed = {}
        # Start with the case where an observed artifact was matched to a ground truth artifact
        for placement_id, uuid in assoc.items():
            timestamp = self.get_timestamp_from_uuid(uuid)
            distance = self.get_distance_traversed_at_time(timestamp, robot)
            distance_traversed.update({placement_id: distance})
        # Then deal with the rest
        for id in self.artifact_gt_w_from_json.keys():
            if not id in distance_traversed.keys():
                distance_traversed.update({id: '---'})
        return [distance_traversed[placement_id] for placement_id in self.sorted_placement_ids]

    def update_pos_error_col(self, assoc):
        """Calculates the distance between an observed artifact and its ground truth match, if this exists"""
        pos_error = {}
        # Start with the case where an observed artifact was matched to a ground truth artifact
        for placement_id, uuid in assoc.items():
            for record in self.artifacts_db:
                if record['id'].startswith(uuid):
                    pos_error.update({placement_id: self.distance_two_points(
                        record['point'], self.artifact_gt_w_from_json[placement_id])})
        # Then deal with the rest
        for id in self.artifact_gt_w_from_json.keys():
            if not id in pos_error.keys():
                pos_error.update({id: '---'})
        return [pos_error[placement_id] for placement_id in self.sorted_placement_ids]

    def update_pos_error_perc_col(self, pos_error_m, dist_traversed):
        pos_error_perc = {}
        id = 0
        for e, d in zip(pos_error_m, dist_traversed):
            if e == "---":
                pos_error_perc.update({id: 0})
            else: 
                pos_error_perc.update({id: e/d * 100})
            id += 1
        print pos_error_perc
        return list(pos_error_perc.values())

    def update_placement_id_col(self):
        return [x for x in self.sorted_placement_ids]

    def update_artifact_type_col(self):
        placed_types = self.artifact_placement
        return [placed_types[placement_id] for placement_id in self.sorted_placement_ids]

    def update_artifact_type_obs_col(self, assoc):
        obs = {}
        # Start with the case where an observed artifact was matched to a ground truth artifact
        for placement_id, uuid in assoc.items():
            for record in self.artifacts_db:
                if record['id'].startswith(uuid):
                    obs.update({placement_id: str(record['label'])})
        # Then deal with the rest
        for id in self.artifact_gt_w_from_json.keys():
            if not id in obs.keys():
                obs.update({id: '---'})
        # TODO
        return [obs[placement_id] for placement_id in self.sorted_placement_ids]

    def update_scored_col(self, assoc):
        scored = {}
        # Start with the case where an observed artifact was matched to a ground truth artifact
        # In most cases it will have a 'scored' status
        # but if it was outside the 5m radius, it will be 'unscored'
        # or if it wasn't submitted, it will be 'pending'
        for placement_id, uuid in assoc.items():
            for record in self.artifacts_db:
                if record['id'].startswith(uuid):
                    scored.update({placement_id: str(record['status'])})
        # Then deal with the rest
        for id in self.artifact_gt_w_from_json.keys():
            if not id in scored.keys():
                scored.update({id: '---'})
        return [scored[placement_id] for placement_id in self.sorted_placement_ids]

    def update_min_dist_col(self, robot):
        min_dists = {}
        for placement_id in self.artifact_gt_w_from_json.keys():
            min_dists.update({placement_id:
                              self.calculate_closest_approach(robot, placement_id)})
        return [min_dists[placement_id] for placement_id in self.sorted_placement_ids]

    def update_artifact_uuid_col(self, assoc):
        return [assoc[placement_id] for placement_id in self.sorted_placement_ids]

    def get_closest_reported_artifact_of_same_type(self, coord_w, type):
        for cat, coord_w in self.artifact_gt_w_from_json.items():
            if self.distance_two_points(coord_scored, coord_w) < self.params['scoring_distance_threshold'] and cat == category:
                return self.get_placement_id_from_coord_w(coord_w)

    def calculate_closest_approach(self, robot, placement_id):
        """Calculates the min Euclidean distance between a given artifact and a given robot as a single float"""
        min_dist = 10000000.0  # very high number, [m]
        for pose in self.poses[robot]:
            dist = self.distance_two_points(
                pose, self.artifact_gt_w_from_json[placement_id])
            if dist < min_dist:
                min_dist = dist
        return min_dist

    def read_database_bson(self):
        """Reads in .bson format, which is how data is stored in database. Returns dictionary version of it"""
        with open(self.params['artifacts_bson_path'], 'r') as infile:
            return bson.decode_all(infile.read())

    def read_database_json(self):
        """Reads in .json format, which is how data is stored in database. Returns dictionary version of it"""
        with open(self.params['artifacts_json_path'], 'r') as infile:
            return json.load(infile)

    def associate_database_records_to_placed_artifacts(self, df):
        """Returns a dictionary {placement_id: parent_id} """
        assoc = {}
        for k, v in df.iteritems():
            if not k in self.pd_column_titles:
                # the contents of each column, in list form
                vals = [x for x in v]
                # Association distance threshold is half the min distance between a given pair of ground truth artifacts
                # This allows you to capture the case where it was observed, but localized outside the 5m scoring threshold
                below_thresh = min(
                    vals) < self.params['association_distance_threshold']
                # print below_thresh
                min_index = vals.index(min(vals))
                for report in self.artifacts_db:
                    # if first 4 characters match the full uuid
                    if report['id'].startswith(k):
                        artifact_type = report['label']
                placement_id = self.sorted_placement_ids[min_index]
                if below_thresh and artifact_type == self.split_underscore(self.get_artifact_type_by_placement_id(placement_id)):
                    assoc.update(
                        {self.sorted_placement_ids[min_index]: str(k)})
        for id in self.sorted_placement_ids:
            if not id in assoc.keys():
                assoc.update({id: '---'})
        return assoc

    def get_timestamp_from_uuid(self, uuid):
        ''' timestamp [double] '''
        timestamp = 0.0
        for record in self.artifacts_db:
            if record['parent_id'].startswith(uuid):
                timestamp = (record['header']['stamp'] - datetime.datetime.utcfromtimestamp(0)).total_seconds()
        return timestamp

    def get_all_distance_pairs(self, robot):
        """Returns a panda DataFrame with 1 column of placement ID, 1 column of artifact type
           and the remaining columns of the distance of each reported artifact
           (by first 4 characters of UUID) to the ground truth list"""
        df = pd.DataFrame()
        for record in self.artifacts_db:
            if record['name'] == robot:  # if observation was made by the robot in question
                dists = []  # distances of each database record from the corresponding placed artifact
                coord = record['point']  # observed location
                for id in sorted(self.artifact_gt_w_from_json.keys()): # go through placement locations
                    dists.append(self.distance_two_points(
                        coord, self.artifact_gt_w_from_json[id]))
                # truncate uuid to 4 characters
                df.insert(0, record['parent_id'][:4], dists)
        df = df.round(1)
        artifact_types = [self.get_artifact_type_by_placement_id(
            placement_id) for placement_id in self.sorted_placement_ids]
        placement_ids = [
            placement_id for placement_id in self.sorted_placement_ids]
        df.insert(0, "Artifact type GT", artifact_types)
        df.insert(0, "Placement ID", placement_ids)
        # df["Placement ID"] = df["Placement ID"].astype(int)  # convert to int
        return df

    def print_results(self, dfs):
        """Prints results, duh!"""
        print '=== ARTIFACT DETECTION AND LOCALIZATION SUMMARY REPORT ===\n'
        print self.params['date'], '\n'
        for robot, df in dfs.items():
            print 'Report for', robot
            print df
            print '\n'


def main():
    os.system('clear')  # clear the terminal
    yaml_path = os.path.join(rospack.get_path(
        'darpa_server_surrogate'), 'cfg/artifact_reporting_utils.yaml')  # read in params
    with open(yaml_path, 'r') as infile:
        params = yaml.safe_load(infile)

    # Main process
    if params['task'] == 0:
        GenerateGtJson(params)
    elif params['task'] == 1:
        GenerateSummaryReport(params)


if __name__ == '__main__':
    main()
