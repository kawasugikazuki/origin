#!/bin/bash
# 使用するGPIOのリスト
pins=(
    4
    17
    27
    22
    10
    9
    11
    13
    19
    26
    21
)

for pin in "${pins[@]}" ; do
    # GPIOをOUTPUTかつLOWにする
    echo $pin > /sys/class/gpio/export
    echo "out" > /sys/class/gpio/gpio${pin}/direction
    echo 0 > /sys/class/gpio/gpio${pin}/value
    echo $pin > /sys/class/gpio/unexport
done


