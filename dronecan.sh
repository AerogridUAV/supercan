#!/bin/bash

# Find the ACM port for CANable by matching the product string
CANABLE_PORT=$(for dev in /dev/ttyACM*; do
    udevadm info -a -n "$dev" | grep -q 'ATTRS{product}=="CANable2 b158aa7 github.com/normaldotcom/canable2.git"' && echo "$dev"
done | head -n1)

if [ -z "$CANABLE_PORT" ]; then
    echo "CANable device not found!"
    exit 1
fi

sudo slcand -o -c -f -s8 "$CANABLE_PORT" can0
sudo ifconfig can0 up

# Start DroneCAN GUI tool in background
dronecan_gui_tool &

# Start Feetech Instruction GUI
# python3 tools/feetech_instruction_gui.py --iface can0 --node-id 100