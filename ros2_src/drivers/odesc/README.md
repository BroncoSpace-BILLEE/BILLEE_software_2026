# odesc

ROS2 Control hardware interface for ODrive-compatible CAN devices.

## Features
- Exposes a single joint with velocity command and position/velocity state
- Uses SocketCAN to send input velocity and retrieve encoder estimates for position and velocity data

## Hardware Parameters
Set these in your URDF/ros2_control `hardware` block:
- `can_interface`: CAN network interface name (e.g., `can0`)
- `node_id`
- `joint_name`

## Example URDF Snippet
```xml
<ros2_control name="ODriveSystem" type="system">
  <hardware>
    <plugin>odesc/OdescSystemInterface</plugin>
    <param name="can_interface">can0</param>
    <param name="node_id">0</param>
  </hardware>
  <joint name="axis0">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

## Resources
 
- if there are issues, please reference the following documentation on ODRIVE CAN messages:

1. https://docs.odriverobotics.com/v/latest/guides/can-guide.html#writing-configuration
2. https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#can-protocol