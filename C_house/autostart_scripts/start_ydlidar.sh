#!/bin/bash

rosws=catkin_ws
house=C_house

sleep 5

export DISPLAY=:0.0
export LOGFILE=/home/$USER/Fukaya/$house/autostart_scripts/ydlidar_node.log

source /home/$USER/Fukaya/$house/autostart_scripts/ROS_CONFIG.txt
source /opt/ros/melodic/setup.bash
source /home/$USER/$rosws/devel/setup.bash


while true
do
		echo >>$LOGFILE
		echo "----------------------------------------------" >> $LOGFILE
		date >> $LOGFILE

		echo "Starting ydlidar TG.launch" >> $LOGFILE
		
		roslaunch ydlidar_ros TG.launch >> $LOGFILE

		echo "program seems to have stopped" >> $LOGFILE

		date >> $LOGFILE
		sleep 1

done