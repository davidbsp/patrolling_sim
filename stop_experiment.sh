#/bin/bash
rosnode kill -a
printf "Cleaning..."
printf "5..."
sleep 1
printf "4..."
sleep 1
printf "3..."
sleep 1
printf "2..."
sleep 1
printf "1..."
sleep 1
kill $(ps aux | grep ros | grep -v grep | awk '{print $2}')
sleep 1
printf "Experiment terminated!\n"

