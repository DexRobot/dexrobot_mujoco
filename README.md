# DexRobot MuJoCo

[English](README.md) | [中文](README_zh.md)

Tactile simulation for DexRobot hands in MuJoCo with ROS integration.

## Key Features

- **Tactile sensing**: Support for both MuJoCo native touch sensors and TaShan 11-dimensional tactile sensors
- **Optimized hand models**: Full and simplified collision geometries for performance tuning
- **Robot integration**: Mount hands on various robot arms (JAKA Zu7, etc.)
- **Rich environments**: Compose scenes with furniture and interactive objects
- **ROS compatibility**: Integration with ROS1/ROS2 ecosystems
- **VR visualization**: Optional VR support for debugging

## Demo Video

<video src="assets/ts_touch_demo.mp4" width="100%" controls>
  View the tactile sensor demo: <a href="assets/ts_touch_demo.mp4">ts_touch_demo.mp4</a>
</video>

The video demonstrates real-time TaShan sensor visualization during a pinch gesture, showing force magnitude and direction feedback from thumb and index finger sensors.

## Installation

### Standard Installation
```bash
pip install -e .
```

### With TaShan Sensor Support
To use TaShan touch sensors, install with the specific MuJoCo version required:
```bash
pip install -e .[tashan]
```
**Important Requirements:**
- TaShan sensors require MuJoCo 3.2.3
- TaShan sensors require Python 3.8
- Without the [tashan] option, the latest MuJoCo version will be used with any Python version ≥ 3.8

**Note**: Using Python 3.8 with ROS can be challenging on modern systems. For detailed setup instructions with Conda and RoboStack, see the documentation. Alternatively, use the [standalone example](examples/tashan_standalone_demo.py) without ROS.

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

## Tactile Sensing

DexRobot MuJoCo provides tactile simulation with two sensor options:

### 1. MuJoCo Native Touch Sensors
Simple contact force detection (1 value per fingertip):
```bash
python nodes/dexrobot_mujoco_ros.py dexrobot_mujoco/scenes/ball_catching.xml
```

### 2. TaShan Tactile Sensors
11-dimensional tactile feedback per sensor pad:
```bash
# Requires Python 3.8 and MuJoCo 3.2.3
python nodes/dexrobot_mujoco_ros.py dexrobot_mujoco/scenes/ball_catching.xml --enable-ts-sensor
```

**TaShan sensor data includes:**
- Normal and tangential force components
- Force direction
- 7-channel capacitance array for detailed contact geometry

### Quick Demo

Try the standalone tactile demo with real-time visualization:
```bash
# Install visualization dependency
pip install rerun-sdk==0.18.2

# Run pinch gesture demo
python examples/tashan_standalone_demo.py
```

This demo shows thumb and index finger forces during object manipulation with live force vector visualization.

### ROS Integration

Sensor data is published to ROS topics:

- `touch_sensors` (Float32MultiArray) - MuJoCo touch sensors or raw TS sensor data
- `ts_forces` (Float32MultiArray) - Semantic force data when using `--enable-ts-sensor`

The TaShan force data provides semantic information (normal force, tangential force, direction) for each sensor pad in a 2D array format.

## Documentation

For more details, see the full documentation in `docs/`:

- Getting Started
- Hand Models and Configuration
- Scene Creation
- ROS Integration
- API Reference

## License

Copyright 2024 DexRobot. Licensed under the Apache License, Version 2.0.
