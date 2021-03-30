#!/bin/bash
# Script to automatically start monitoring when a roscore is running
# 1: Name the Tmux session should have
# 2: Command to run
# and restarted when started again
running=false
session=monitoring
source "/home/cmr_user/cmr_utils/Scripts/tmux/tmux_utils.sh"

# Kill session if exists at start
tmux kill-session -t $session

# If Terminate signal received, kill session
trap "tmux kill-session -t $session" SIGTERM
sleep 2
while true
do
  # Check if rosmaster is running
  if rostopic list > /dev/null 2>&1; then
    if [ "$running" = false ]; then
      echo "Start $session tmux session"
      tmux new-session -d -s $session
      tmux new-window -t $session -n "Monitoring"
      tmux send-keys -t "Monitoring" "roslaunch cmr_monitoring monitoring.launch" ENTER
      tmux new-window -t $session -n "Axserver"
      tmux send-keys -t "Axserver" "roslaunch cmr_monitoring axserver.launch" ENTER
      tmux new-window -t $session -n "BT"
      tmux send-keys -t "BT" "roslaunch cmr_monitoring monitoring_bt.launch" ENTER
      running=true
    fi
   # echo "ROS Master running"
  else
    kill_session_if_running $session
    running=false
    # echo "ROS Master not running"
  fi
  sleep 5
done
exit 0
