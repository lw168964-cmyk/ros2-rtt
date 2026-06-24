# lslidar_driver for N10P UART

This workspace keeps only the N10P UART launch/config files used by the robot
mapping workflow.

## Kept Files

- `launch/lsn10p_launch.py`
- `config/lslidar_n10p_uart.yaml`

## Topic And Frame

- LaserScan topic: `/scan`
- Laser frame: `laser`
- Serial port in lidar config: `/dev/ttyACM0`

## Standalone Lidar Start

```bash
ros2 launch lslidar_driver lsn10p_launch.py
```

For full robot SLAM mapping, use:

```bash
ros2 launch roscar_bringup slam.launch.py
```
