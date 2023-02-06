#!/bin/bash

rosws=catkin_ws
house=A_house

sleep 15

export DISPLAY=:0.0
export LOGFILE=/home/$USER/Fukaya/$house/autostart_scripts/greenhouse_nav_node.log

source /home/$USER/Fukaya/$house/autostart_scripts/ROS_CONFIG.txt
source /opt/ros/melodic/setup.bash
source /home/$USER/$rosws/devel/setup.bash

cd /home/$USER/Fukaya/$house/

while true
do
		echo >>$LOGFILE
		echo "----------------------------------------------" >> $LOGFILE
		date >> $LOGFILE

		echo "Starting greenhouse_navigation2.py" >> $LOGFILE
		
		python -u greenhouse_navigation2.py --params_file GreenhouseNavParams.yaml >> $LOGFILE

		echo "program seems to have stopped" >> $LOGFILE

		date >> $LOGFILE
		sleep 1

done
