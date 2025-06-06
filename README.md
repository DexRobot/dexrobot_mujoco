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

## Documentation

For more details, see the full documentation in `docs/`:

- Getting Started
- Hand Models and Configuration
- Scene Creation
- ROS Integration
- API Reference

## License

Copyright 2024 DexRobot. Licensed under the Apache License, Version 2.0.
