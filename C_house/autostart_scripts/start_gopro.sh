#!/bin/bash

rosws=catkin_ws
house=C_house

sleep 10

export DISPLAY=:0.0
export LOGFILE=/home/$USER/Fukaya/$house/autostart_scripts/gopro_node.log

source /home/$USER/Fukaya/$house/autostart_scripts/ROS_CONFIG.txt
source /opt/ros/melodic/setup.bash
source /home/$USER/$rosws/devel/setup.bash

cd /home/$USER/Fukaya/$house/

while true
do
		echo >>$LOGFILE
		echo "----------------------------------------------" >> $LOGFILE
		date >> $LOGFILE

		echo "Starting gopro_capture_udp.py" >> $LOGFILE
		
		python3 -u gopro_capture_udp.py >> $LOGFILE

		echo "program seems to have stopped" >> $LOGFILE

		date >> $LOGFILE
		sleep 1

done
