#!/bin/bash
sudo slcand -o -c -f -s8 /dev/ttyACM0 can0
sudo ifconfig can0 up
uavcan_gui_tool