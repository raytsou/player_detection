#!/usr/bin/env bash
i=1;
for arg in "$@" 
do
    echo "Arg - $i: $arg";
	case "${arg}" in
        --run) RUN_DETECT=true;;
    esac

    i=$((i + 1));
done

source /opt/ros/noetic/setup.sh
catkin_make
source devel/setup.bash
export PYTHONPATH="$PYTHONPATH:/ws/src/player_detection/include/Sequoia/"

if [ "$RUN_DETECT" = true ] ; then
    roslaunch player_detection detect.launch
fi