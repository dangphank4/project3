#!/bin/bash

thermal_zones=(
/sys/class/thermal/thermal_zone1
/sys/class/thermal/thermal_zone2
/sys/class/thermal/thermal_zone3
/sys/class/thermal/thermal_zone4
/sys/class/thermal/thermal_zone5
/sys/class/thermal/thermal_zone6
/sys/class/thermal/thermal_zone7
/sys/class/thermal/thermal_zone8
/sys/class/thermal/thermal_zone9
/sys/class/thermal/thermal_zone10
/sys/class/thermal/thermal_zone11
/sys/class/thermal/thermal_zone12
/sys/class/thermal/thermal_zone13
/sys/class/thermal/thermal_zone14
/sys/class/thermal/thermal_zone15
/sys/class/thermal/thermal_zone17
/sys/class/thermal/thermal_zone18
/sys/class/thermal/thermal_zone19
/sys/class/thermal/thermal_zone33
/sys/class/thermal/thermal_zone34
/sys/class/thermal/thermal_zone35
/sys/class/thermal/thermal_zone36
/sys/class/thermal/thermal_zone37
/sys/class/thermal/thermal_zone38
/sys/class/thermal/thermal_zone39
/sys/class/thermal/thermal_zone40
)

for zone in "${thermal_zones[@]}"; do
    temp=$(cat "$zone/temp")
    name=$(cat "$zone/type" 2>/dev/null)
    echo "$zone ($name): $(($temp / 1000)) °C"
done
