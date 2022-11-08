#!/usr/bin/env python

import argparse
import subprocess, yaml, csv
import rosbag 
import os, sys
import rospy

### USAGE!
### python bag_analyzer.py --in /home/costar/Data/parking11oct/ --out husky2_parking11oct.bag --topics camera_right/color/image_raw
###

parser = argparse.ArgumentParser()
# https://mkaz.blog/code/python-argparse-cookbook/
parser.add_argument('--in', dest='in_bag_path', nargs = 1)
# parser.add_argument('--out', dest='out_bag', nargs = 1)
parser.add_argument('--topics', dest='topics_to_include', nargs = '*')
args = parser.parse_args()
in_bag_path = args.in_bag_path[0]
out_bags = []
topics_to_include = args.topics_to_include

time_buffer = 0.0 # [s]

# times_ = {}
times = []
with open('parking_distances.csv') as f: ## FOR SOME REASON, CAN'T READ CSV IN OTHER DIRECTORY
    csv_reader = csv.reader(f, delimiter = ',')
    # https://stackoverflow.com/questions/13428318/reading-rows-from-a-csv-file-in-python
    for i, line in enumerate(csv_reader):
        out_bags.append(line[0]+'.bag') 
        times.append((float(line[1]) - time_buffer, float(line[2]) + time_buffer))
        print 'Start bracket:', float(line[1]) - time_buffer, " ", float(line[2]) + time_buffer

print 'Going to eventually create this list of output bags:', out_bags

# if os.path.exists(out_bag):
    # os.remove(out_bag)
in_bags = []
# if it is a "*.bag" file in this directory
for _, _, files in os.walk(in_bag_path):
    for f in files:
        if f.split(".")[1] == 'bag':
            if f.split(".")[0].find('vision') >= 0:
                in_bags.append(in_bag_path + f)

# sort these bags to facilitate cutting out the boring bits
in_bags.sort()
prev_in_bracket = False
curr_in_bracket = False
time_bracket_ended = 0.0
time_to_cut_out = 0.0
first_msg = True
total_time_to_cut_out = 0.0
time_bracket_index = 0

for outbag_name in out_bags:
    done = False
    with rosbag.Bag(outbag_name, 'w') as outbag:
        print '>>> Saving to', outbag_name
        for in_bag in in_bags: # for all the input bags
            if not done: 
                print "Processing " + in_bag
                for topic, msg, t in rosbag.Bag(in_bag).read_messages(): # go through contents of the input bag
                    if topic in topics_to_include: # if it's a topic we want
                            time_bracket = times[time_bracket_index]
                            if t.to_sec() > time_bracket[0] and t.to_sec() < time_bracket[1]: # and in the specified time bracket
                                curr_in_bracket = True
                            else:
                                curr_in_bracket = False
                            if first_msg:
                                first_msg = False
                                time_bracket_ended = t.to_sec()       
                                print 'First time start', t.to_sec()           
                            if curr_in_bracket and prev_in_bracket: # you're in a bracket
                                outbag.write(topic, msg, t - rospy.Duration(total_time_to_cut_out))
                            elif curr_in_bracket and not prev_in_bracket: # you've just entered a bracket
                                print 'Just entered bracket at', t.to_sec()
                                total_time_to_cut_out += time_to_cut_out
                                prev_in_bracket = True
                                outbag.write(topic, msg, t - rospy.Duration(total_time_to_cut_out))
                            elif not curr_in_bracket and prev_in_bracket: # you've just left a bracket
                                print 'Just left bracket at', t.to_sec()
                                time_bracket_ended = t.to_sec()
                                time_bracket_index += 1
                                prev_in_bracket = False
                                done = True
                            elif not curr_in_bracket and not prev_in_bracket: # you're not in a bracket
                                time_to_cut_out = (t.to_sec() - time_bracket_ended)