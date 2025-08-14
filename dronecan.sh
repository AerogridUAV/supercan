#!/bin/bash
sudo slcand -o -c -f -s8 /dev/ttyACM2 can0
sudo ifconfig can0 up

# Start DroneCAN GUI tool in background
dronecan_gui_tool &

# Start Feetech Instruction GUI
python3 tools/feetech_instruction_gui.py --iface can0 --node-id 100