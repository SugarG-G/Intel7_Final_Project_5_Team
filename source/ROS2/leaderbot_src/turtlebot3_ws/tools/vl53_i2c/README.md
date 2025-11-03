# VL53L0X I²C Support

This directory contains helper files for bringing up dual VL53L0X ToF sensors on the TurtleBot3 SBC.  
The goal is to keep everything under `~/turtlebot3_ws` so the setup can be version controlled together with the ROS workspace.

The pipeline is:

1. Configure XSHUT GPIOs through an overlay (optional, but keeps the pins in a known state at boot).
2. Run a small script at boot that toggles the XSHUT pins and assigns the addresses 0x2A/0x2B.
3. Launch `dual_vl53l0x_node` which publishes `/tb3_2/vl53_front_range` and `/tb3_2/vl53_rear_range`.

## Files

| File | Purpose |
|------|---------|
| `vl53_dual_overlay.dts` | Example device-tree overlay declaring the two XSHUT pins (GPIO17/27). |
| `Makefile` | Simple helper to compile the overlay with `make overlay`. |
| `vl53_addr_boot.sh` | Shell script that uses `gpioset` + `i2cset` to program the addresses. |
| `setup_vl53_addresses.py` | Python script that performs the same job using Blinka; can be launched via systemd or from a ROS launch file. |
| `README.md` | This guide. |

## Building and installing the overlay (optional)

If you want the pins to default HIGH at boot:

```bash
cd ~/turtlebot3_ws/tools/vl53_i2c
make overlay
sudo cp build/vl53-dual.dtbo /boot/firmware/overlays/
echo dtoverlay=vl53-dual | sudo tee -a /boot/firmware/config.txt
```

Reboot afterwards. The overlay does **not** change the I²C address – it only keeps the XSHUT pins available.

## Programming sensor addresses at boot

### One-shot shell script

```bash
chmod +x ~/turtlebot3_ws/tools/vl53_i2c/vl53_addr_boot.sh
~/turtlebot3_ws/tools/vl53_i2c/vl53_addr_boot.sh
sudo i2cdetect -y 1   # should show 0x2a and 0x2b
```

This script requires `libgpiod` tools (`gpioset`) and `i2c-tools` (`i2cset`).

### Python script (recommended for launch/systemd integration)

```
chmod +x ~/turtlebot3_ws/tools/vl53_i2c/setup_vl53_addresses.py
~/turtlebot3_ws/tools/vl53_i2c/setup_vl53_addresses.py
```

You can run this script from a systemd service **before** `robot.launch.py`, or from within the launch file using `launch.actions.ExecuteProcess`.

Example systemd unit:

```
[Unit]
Description=Configure VL53L0X addresses
After=multi-user.target

[Service]
Type=oneshot
User=ubuntu
ExecStart=/usr/bin/python3 /home/ubuntu/turtlebot3_ws/tools/vl53_i2c/setup_vl53_addresses.py

[Install]
WantedBy=multi-user.target
```

## Bringup commands

1. (Optional) run the address script manually to confirm:
   ```bash
   python3 ~/turtlebot3_ws/tools/vl53_i2c/setup_vl53_addresses.py
   sudo i2cdetect -y 1
   ```
2. Launch TurtleBot3 bringup (which already starts `dual_vl53l0x_node`):
   ```bash
   ros2 launch turtlebot3_bringup robot.launch.py
   ```
3. On the remote PC:
   ```bash
   ros2 run turtlebot3_example turtlebot3_patrol_stop_dummy \
     --ros-args -p laser_topic:=/tb3_2/vl53_front_range \
                -p laser_topic_type:=range \
                -p laser_data_timeout:=1.0
   ```

## Notes

- If a sensor loses power/XSHUT, it will reset to address 0x29. Rerun the script to restore 0x2A/0x2B.
- The overlay keeps the pins reserved; the actual address assignment still happens in user space.
- Adjust GPIO numbers in the scripts/overlay if you wire XSHUT to different pins.
