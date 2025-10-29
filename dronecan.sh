#!/bin/bash

# Find the ACM port for CANable by matching the product string
CANABLE_PORT=$(for dev in /dev/ttyACM*; do
    if udevadm info -a -n "$dev" | grep -qi 'ATTRS{product}.*canable'; then
        echo "$dev"
        break
    fi
done | head -n1)

if [ -z "$CANABLE_PORT" ]; then
    echo "CANable device not found!"
    exit 1
fi

sudo slcand -o -c -f -s8 "$CANABLE_PORT" can0
sudo ifconfig can0 up

# Start DroneCAN GUI tool in background
dronecan_gui_tool &