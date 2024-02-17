#!/bin/bash

echo 77 > /sys/class/gpio/export
echo 81 > /sys/class/gpio/export
echo 83 > /sys/class/gpio/export
echo 76 > /sys/class/gpio/export

echo "out" > /sys/class/gpio/PC13/direction
echo "out" > /sys/class/gpio/PC17/direction
echo "out" > /sys/class/gpio/PC19/direction
echo "in" > /sys/class/gpio/PC12/direction

while true; do
    buff=$(cat /sys/class/gpio/PC12/value)
    
    if [ "$buff" == "1" ]; then
        echo 1 > /sys/class/gpio/PC13/value
        echo 1 > /sys/class/gpio/PC17/value
        echo 1 > /sys/class/gpio/PC19/value
    else
        echo 0 > /sys/class/gpio/PC13/value
        echo 0 > /sys/class/gpio/PC17/value
        echo 0 > /sys/class/gpio/PC19/value
    fi
    
    sleep 0.5
done
