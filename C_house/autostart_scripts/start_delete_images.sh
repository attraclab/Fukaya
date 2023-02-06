#!/bin/bash

house=C_house

export LOGFILE=/home/$USER/Fukaya/$house/autostart_scripts/delete_images.log

echo "----------------------------------------------" >> $LOGFILE
date >> $LOGFILE
cd /home/$USER/Fukaya/$house/
echo "Starting delete_images.py" >> $LOGFILE
python -u delete_images.py >> $LOGFILE