
# Simple RoboClaw Joystick Controller

## What It Does

- Reads joystick input
- Converts to output range [-127, 127]
- Sends directly to RoboClaw motors



```bash
ros2 launch roboclaw roboclaw.launch.py
```

## Usage

- **Left/Right**: correspond to M1/M2 respsectively

## Configuration

Edit in launch file (`roboclaw.launch.py`):

```python
'port': '/dev/ttyACM0',      # Your serial port
'baudrate': 38400,            # RoboClaw baudrate
'address': 128,               # RoboClaw address
'axis_left': 1,            # Joystick axis for M1
'axis_right': 0,               # Joystick axis for M2
```



## Troubleshooting

- launch only starts joy node if `launch_joy = True`
- see [roboclaw docs](https://resources.basicmicro.com/using-the-roboclaw-python-library/) for further info


