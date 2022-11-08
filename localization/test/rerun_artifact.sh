#!/bin/bash

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <rosbag_dir> [<extra_args> ...]"
  exit -1
fi

bagdir=$1
extra_args=${@:2}
robot_name=$(ls ${bagdir}/*_tf_* -1 | head -1 | xargs basename | cut -f 1 -d '_')

rosbag play --clock \
  ${bagdir}/*_tf_* \
  ${bagdir}/*_artifact_* \
  ${bagdir}/*_vision_* \
  ${bagdir}/*_lidar_* \
  $extra_args \
  /$robot_name/co2:=/$robot_name/co2_unused \
  /$robot_name/wifi_rssi:=/$robot_name/wifi_rssi \
  /$robot_name/unreconciled_artifact:=/$robot_name/unreconciled_artifact_original \
  /$robot_name/artifact:=/$robot_name/artifact_original \
  /$robot_name/artifact/update:=/$robot_name/artifact/update_original \
  #/$robot_name/thermal/detected_object:=/$robot_name/thermal/detected_object_unused
