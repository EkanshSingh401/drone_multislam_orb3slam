#!/usr/bin/env bash
# euroc_prep.sh <group zip> <seq>: extract <seq>.bag + ASL zip, convert the ROS1 bag to ROS2 (mcap).
set -e; E=/out/euroc; z=$E/zip/$1.zip; s=$2; mkdir -p $E/seq/$s; cd $E/seq/$s
[[ -f asl/mav0/state_groundtruth_estimate0/data.csv ]] || { unzip -o -j -q $z "$1/$s/$s.zip" -d . && unzip -o -q $s.zip -d asl && rm -f $s.zip; }
[[ -d ros2 ]] || { unzip -o -j -q $z "$1/$s/$s.bag" -d . && rosbags-convert --src $s.bag --dst ros2 --dst-storage mcap && rm -f $s.bag; }
ls ros2 asl/mav0 | tr '\n' ' '; echo
