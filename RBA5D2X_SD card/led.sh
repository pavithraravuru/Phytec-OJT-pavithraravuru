#!/bin/bash

echo 77 > /sys/class/gpio/export
echo 81 > /sys/class/gpio/export
echo 83 > /sys/class/gpio/export

echo "out" > /sys/class/gpio/PC13/direction
echo "out" > /sys/class/gpio/PC17/direction
echo "out" > /sys/class/gpio/PC19/direction

while true; do
    echo 1 > /sys/class/gpio/PC13/value
    sleep 1
    echo 0 > /sys/class/gpio/PC13/value
    sleep 1

    echo 1 > /sys/class/gpio/PC17/value
    sleep 1
    echo 0 > /sys/class/gpio/PC17/value
    sleep 1

    echo 1 > /sys/class/gpio/PC19/value
    sleep 1
    echo 0 > /sys/class/gpio/PC19/value
    sleep 1
done
exit 0

