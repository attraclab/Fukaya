#!/bin/bash

rosws=catkin_ws
house=A_house

sleep 1

export DISPLAY=:0.0
export LOGFILE=/home/$USER/Fukaya/$house/autostart_scripts/roscore_node.log

source /home/$USER/Fukaya/$house/autostart_scripts/ROS_CONFIG.txt
source /opt/ros/melodic/setup.bash
source /home/$USER/$rosws/devel/setup.bash


while true
do
		echo >>$LOGFILE
		echo "----------------------------------------------" >> $LOGFILE
		date >> $LOGFILE

		echo "Starting roscore" >> $LOGFILE
		
		roscore >> $LOGFILE

		echo "program seems to have stopped" >> $LOGFILE

		date >> $LOGFILE
		sleep 1

done
