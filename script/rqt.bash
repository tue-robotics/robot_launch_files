#!/bin/bash
if [ -z "$1" ]
then
    echo "Usage: ./rqt.bash [rqt_perspective] [rviz_config] (optional)"
else
    if [[ $2 == *.rviz ]]
    then
        cp $2 ~/.rviz/default.rviz
    fi
    rqt --perspective-file $1
fi
