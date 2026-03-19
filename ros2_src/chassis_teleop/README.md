# chassis_teleop

## Simple autonomy + ZED object distance

This package includes two Python nodes:

- `zed_object_distance`: uses the ZED SDK Python API to publish the nearest detected object distance.
- `simple_autonomy`: drives forward and performs a simple avoid maneuver when an object is within 1 meter.

### Topics

- Publishes: `/zed2i/nearest_object_distance` (`std_msgs/Float32`)
- Subscribes: `/zed2i/nearest_object_distance`
- Publishes: `/diff_drive_controller/cmd_vel_unstamped` (`geometry_msgs/Twist`)

### Requirements

- ZED SDK installed on the machine.
- ZED Python API (`pyzed.sl`) available on `PYTHONPATH`.

### Run

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run chassis_teleop zed_object_distance
```

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run chassis_teleop simple_autonomy
```

### Parameters (defaults)

`zed_object_distance`:
- `distance_topic`: `/zed2i/nearest_object_distance`
- `publish_hz`: `10.0`
- `no_object_value`: `-1.0`
- `confidence_threshold`: `20`
- `detection_model`: `MULTI_CLASS_BOX_FAST`

`simple_autonomy`:
- `cmd_vel_topic`: `/diff_drive_controller/cmd_vel_unstamped`
- `object_distance_topic`: `/zed2i/nearest_object_distance`
- `publish_hz`: `20.0`
- `forward_speed`: `0.4`
- `turn_speed`: `0.8`
- `avoid_forward_speed`: `0.3`
- `avoid_forward_duration`: `5.0`
- `turn_angle_deg`: `90.0`
- `object_distance_threshold`: `1.0`
- `no_object_value`: `-1.0`
