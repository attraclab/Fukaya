#!/bin/bash

rosws=catkin_ws

sleep 5

export DISPLAY=:0.0
export LOGFILE=/home/$USER/Fukaya/autostart_scripts/atcart_node.log

source /home/$USER/Fukaya/autostart_scripts/ROS_CONFIG.txt
source /opt/ros/melodic/setup.bash
source /home/$USER/$rosws/devel/setup.bash


while true
do
		echo >>$LOGFILE
		echo "----------------------------------------------" >> $LOGFILE
		date >> $LOGFILE

		echo "Starting jmoab-ros atcart_basic.py" >> $LOGFILE
		
		rosrun jmoab-ros atcart_basic.py >> $LOGFILE

		echo "program seems to have stopped" >> $LOGFILE

		date >> $LOGFILE
		sleep 1

done
