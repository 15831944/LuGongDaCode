#!/bin/bash

if pgrep Trans >/dev/null;then
    echo "Trans is already running"
else
    cd /home/locate/runTrans
    nohup ./start.sh &
fi
