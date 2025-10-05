#!/bin/bash

ulimit -c unlimited
if grep -q "MVS" /proc/sys/kernel/core_pattern; then 
    echo "had enable coredump"
else
    echo "start enable coredump"
    sudo tee /proc/sys/kernel/core_pattern <<< "/opt/MVS/bin/Temp/core-%e-%p-%t"
fi
