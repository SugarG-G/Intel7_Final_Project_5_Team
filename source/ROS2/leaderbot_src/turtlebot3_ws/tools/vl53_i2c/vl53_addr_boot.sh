#!/usr/bin/env bash
set -euo pipefail

# GPIO pins for XSHUT
GPIO_A=17
GPIO_B=27

# Power down both sensors
gpioset gpiochip0 ${GPIO_A}=0 ${GPIO_B}=0
sleep 0.05

# Bring up sensor A and set to 0x2A
gpioset gpiochip0 ${GPIO_A}=1
sleep 0.1
i2cset -y 1 0x29 0x8A 0x2A

# Bring up sensor B and set to 0x2B
gpioset gpiochip0 ${GPIO_B}=1
sleep 0.1
i2cset -y 1 0x29 0x8A 0x2B

echo "VL53L0X addresses set to 0x2A and 0x2B"
