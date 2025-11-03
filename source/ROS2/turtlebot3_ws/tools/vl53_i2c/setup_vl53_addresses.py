#!/usr/bin/env python3
"""Set VL53L0X sensors on I2C-1 to addresses 0x2A and 0x2B."""

import time

import board
import busio
import digitalio
import adafruit_vl53l0x

GPIO_SENSOR_A = board.D17
GPIO_SENSOR_B = board.D27
DEFAULT_ADDRESS = 0x29
FRONT_ADDRESS = 0x2A
REAR_ADDRESS = 0x2B


def main() -> None:
    xshut_a = digitalio.DigitalInOut(GPIO_SENSOR_A)
    xshut_a.direction = digitalio.Direction.OUTPUT
    xshut_b = digitalio.DigitalInOut(GPIO_SENSOR_B)
    xshut_b.direction = digitalio.Direction.OUTPUT

    # Power down both sensors
    xshut_a.value = False
    xshut_b.value = False
    time.sleep(0.05)

    i2c = busio.I2C(board.SCL, board.SDA)

    # Bring up sensor A and assign address
    xshut_a.value = True
    time.sleep(0.02)
    sensor_a = adafruit_vl53l0x.VL53L0X(i2c, address=DEFAULT_ADDRESS)
    sensor_a.set_address(FRONT_ADDRESS)

    # Bring up sensor B and assign address
    xshut_b.value = True
    time.sleep(0.02)
    sensor_b = adafruit_vl53l0x.VL53L0X(i2c, address=DEFAULT_ADDRESS)
    sensor_b.set_address(REAR_ADDRESS)

    print(f"Configured VL53L0X sensors at 0x{FRONT_ADDRESS:02X} and 0x{REAR_ADDRESS:02X}")


if __name__ == "__main__":
    main()
