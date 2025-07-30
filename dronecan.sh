#!/bin/bash
sudo slcand -o -c -f -s8 /dev/ttyACM2 can0
sudo ifconfig can0 up
dronecan_gui_tool