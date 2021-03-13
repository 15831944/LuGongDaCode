#!/bin/bash

if pgrep Perc >/dev/null;then
    echo "Perc is already running"
else
    cd /home/locate/runPerc
    nohup ./start.sh &
fi
