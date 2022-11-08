#!/bin/bash

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <rosbag_dir> [<extra_args> ...]"
  exit -1
fi

bagdir=$1
robot_name=drone1
extra_args=${@:2}

# NOTE: KAIST team: history_cloud_transform -> /drone1/history_cloud_transform
#       namespace change should be done on your drone/base, then the remapping below
#       can be removed in future versions.

rosbag play \
  ${bagdir}/*.bag \
  $extra_args \
  /$robot_name/co2:=/$robot_name/co2_unused \
  /$robot_name/wifi_rssi:=/$robot_name/wifi_rssi \
  /$robot_name/unreconciled_artifact:=/$robot_name/unreconciled_artifact_original \
  /$robot_name/artifact:=/$robot_name/artifact_original \
  /$robot_name/artifact/update:=/$robot_name/artifact/update_original \
  /history_cloud_transform:=/$robot_name/history_cloud_transform \
  #/$robot_name/thermal/detected_object:=/$robot_name/thermal/detected_object_unused
