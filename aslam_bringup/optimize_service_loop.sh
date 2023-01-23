#!/bin/bash

trap '
  trap - INT # restore default INT handler
  kill -s INT "$$"
' INT

echo "Calling /optimize_current_pose service in an infinite loop [ hit CTRL+C to stop]"
k=0
while true; do
  echo "Call $k"
  echo $(date "+%H:%M:%S (%s)")
  rosservice call /optimize_current_pose "time: 1.0"
  k=$((k+1))
done
