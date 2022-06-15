#!/usr/bin/env bash
source /opt/ros/noetic/setup.sh
pip install empy
pip install catkin_pkg
catkin_make
source devel/setup.bash
export PYTHONPATH="$PYTHONPATH:/ws/src/player_detection/include/Sequoia/"
