#!/bin/bash
# Script to automatically start a tmux session when ROS-Master is running. Will automatically quit when ROS-Master is closed
# and restarted when started again
running=false
session=db_logging

# Function to check if tmux session is already running
function session_running(){
  tmux has-session -t $session 2>/dev/null

  if [ $? != 0 ]; then
    echo false
  else
    echo true
  fi
}
# Function to kill the tmux session if it is running, do nothing otherwise
function kill_session_if_running(){
    sr=$(session_running)
    if [ "$sr" = true ]; then
      echo "Kill db_logging tmux session"
      # Send SIGINT to logging process, this should end the session as well
      tmux list-panes -F $session | xargs -I {} tmux send-keys -t {} C-c &
      # Sleep and kill session, just in case
      sleep 10
      tmux kill-session -t $session
    fi
}
# Kill session if exists at start
kill_session_if_running

trap "{kill_session_if_running; exit 0; }" SIGTERM 

while true
do
  # Check if rosmaster is running
  if rostopic list > /dev/null 2>&1; then
    if [ "$running" = false ]; then
      echo "Start db_logging tmux session"
      tmux new-session -d -s db_logging
      tmux send-keys -t db_logging 'roslaunch cmr_logging log_to_db.launch --wait' Enter
      running=true
    fi
  # echo "ROS Master running"
  else
    kill_session_if_running
    running=false
    #echo "ROS Master not running"
  fi
  sleep 5
done
exit 0
