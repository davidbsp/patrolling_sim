#/bin/bash
rosnode kill -a
sleep 10
kill $(ps aux | grep ros | grep -v grep | awk '{print $2}')
sleep 10

