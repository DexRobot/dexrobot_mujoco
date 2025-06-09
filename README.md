# DexRobot MuJoCo

[English](README.md) | [中文](README_zh.md)

MuJoCo binding for DexRobot hand simulation with ROS integration, providing:

- Hand simulation with optimized collision models
- Robot arm integration
- Scene composition with furniture and environments
- ROS1/ROS2 compatibility
- VR visualization

## Installation

```bash
pip install -e .
```

## Hand Models

Available hand models:

```bash
# Right hand (full collision model)
python nodes/dexrobot_mujoco_ros.py dexrobot_mujoco/models/dexhand021_right.xml

# Right hand (simplified collision model - faster)
python nodes/dexrobot_mujoco_ros.py dexrobot_mujoco/models/dexhand021_right_simplified.xml

# Left hand variants
python nodes/dexrobot_mujoco_ros.py dexrobot_mujoco/models/dexhand021_left.xml
python nodes/dexrobot_mujoco_ros.py dexrobot_mujoco/models/dexhand021_left_simplified.xml

# Hand with floating base
python nodes/dexrobot_mujoco_ros.py dexrobot_mujoco/models/dexhand021_right_floating.xml

# Hand mounted on robot arm
python nodes/dexrobot_mujoco_ros.py dexrobot_mujoco/models/dexhand021_right_jaka_zu7.xml
```

The simplified collision model significantly improves simulation performance while maintaining accuracy:

![Hand Models](assets/hands.png)

## Scene Composition

Create a ball catching scene:

```bash
python nodes/dexrobot_mujoco_ros.py dexrobot_mujoco/scenes/ball_catching.xml --config config/ball_catching.yaml
```

Configure initial positions and tracked objects in YAML:

```yaml
camera:
  azimuth: -180
  distance: 2.5
  elevation: -25
  lookat: [0.0, 0.0, 0.55]

initial_qpos_freejoint:
  ball_joint: [2.0, -0.2, 0.0, 1.0, 0.0, 0.0, 0.0]

initial_qvel_freejoint:
  ball_joint: [-4.1, 0.0, 4.1, 0.0, 0.0, 0.0]
```

## ROS Interface

Control joints using standard ROS messages:

```python
from sensor_msgs.msg import JointState

# Publish joint commands
msg = JointState()
msg.name = ['r_f_joint1_1']
msg.position = [0.5]
publisher.publish(msg)
```

Record data in multiple formats:

```bash
python nodes/dexrobot_mujoco_ros.py model.xml \
    --output-formats ros csv mp4 \
    --output-csv-path data.csv \
    --output-mp4-path video.mp4 \
    --output-bag-path recording.bag
```

## Touch Sensor Support

DexRobot MuJoCo includes comprehensive touch sensor capabilities with both MuJoCo touch sensors and advanced Tactile Sensor (TS) support.

### Sensor Types

All models now include **both sensor types**:

1. **Regular MuJoCo Touch Sensors**: Simple contact detection
2. **TS Sensors**: Advanced tactile sensing with spatial information

### Usage

Choose which sensor data to publish at runtime:

```bash
# Publish regular MuJoCo touch sensor data (5 values per hand)
python nodes/dexrobot_mujoco_ros.py dexrobot_mujoco/scenes/ball_catching.xml --config config/ball_catching.yaml

# Publish TS sensor data (11 values per hand)
python nodes/dexrobot_mujoco_ros.py dexrobot_mujoco/scenes/ball_catching.xml --config config/ball_catching.yaml --enable-ts-sensor
```

### Data Structure

**Regular MuJoCo Touch Sensors** (`/right_hand/touch_sensors` without `--enable-ts-sensor`):
```python
# 5 elements: simple contact force per fingertip
[
    touch_thumb_force,     # 0.0 = no contact, >0 = contact force
    touch_index_force,     # Values represent contact magnitude
    touch_middle_force,
    touch_ring_force,
    touch_pinky_force
]
```

**TS Sensors** (`/right_hand/touch_sensors` with `--enable-ts-sensor`):
```python
# 11 elements following dextactisim format: sdata = Data.sensordata[[1,2, 12,13, 23,24, 34,35, 45,46] + user1_data_id]
[
    # Fixed MuJoCo sensor indices (10 values)
    sensordata[1],   sensordata[2],   # Finger pair 1 (th): normal, tangential
    sensordata[12],  sensordata[13],  # Finger pair 2 (ff): normal, tangential  
    sensordata[23],  sensordata[24],  # Finger pair 3 (mf): normal, tangential
    sensordata[34],  sensordata[35],  # Finger pair 4 (rf): normal, tangential
    sensordata[45],  sensordata[46],  # Finger pair 5 (lf): normal, tangential
    
    # TS-F-A sensor data (1 value - starting index of 11-dimensional sensor)
    ts_sensor_value  # First element of TS-F-A sensor array
]
```

### TS-F-A Sensor Details

The TS-F-A sensor itself is an 11-dimensional tactile sensor with:

```python
# Complete TS-F-A sensor structure (11 dimensions)
[
    proximity_sensing,        # [0] Distance/proximity measurement
    normal_force,            # [1] Force perpendicular to sensor surface  
    tangential_force,        # [2] Force parallel to sensor surface
    tangential_direction,    # [3] Force direction (0-360°, fingertip = 0°)
    capacitance_f1,          # [4] Raw capacitance from sensor element F1
    capacitance_f2,          # [5] Raw capacitance from sensor element F2  
    capacitance_f3,          # [6] Raw capacitance from sensor element F3
    capacitance_f4,          # [7] Raw capacitance from sensor element F4
    capacitance_f5,          # [8] Raw capacitance from sensor element F5
    capacitance_f6,          # [9] Raw capacitance from sensor element F6
    capacitance_f7           # [10] Raw capacitance from sensor element F7
]
```

### TS Sensor Implementation

- **Hardware Support**: Automatically detects and uses TS sensor hardware library if available
- **Simulation Fallback**: When hardware is unavailable, provides realistic simulated tactile data based on contact physics
- **Spatial Patterns**: TS simulation creates realistic force distribution patterns across the sensor surface
- **Real-time Updates**: Sensor data updates with each simulation step

### ROS Topics

Both sensor types publish to the same topics but with different data formats:

- `/left_hand/touch_sensors` (Float64MultiArray)
- `/right_hand/touch_sensors` (Float64MultiArray)

The topic names remain consistent - only the data content and dimensionality change based on the `--enable-ts-sensor` flag.

## Documentation

For more details, see the full documentation in `docs/`:

- Getting Started
- Hand Models and Configuration
- Scene Creation
- ROS Integration
- API Reference

## License

Copyright 2024 DexRobot. Licensed under the Apache License, Version 2.0.
