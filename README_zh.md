# DexRobot MuJoCo

[English](README.md) | [中文](README_zh.md)

DexRobot 灵巧手的触觉仿真环境，基于 MuJoCo 并集成 ROS 支持。

## 核心功能

- **触觉传感**：支持 MuJoCo 原生触觉传感器和他山（TaShan）11维触觉传感器
- **优化的手部模型**：提供完整和简化的碰撞几何体，可根据性能需求调整
- **机器人集成**：支持将手安装在各种机械臂上（JAKA Zu7 等）
- **丰富的环境**：可组合家具和交互物体的场景
- **ROS 兼容性**：与 ROS1/ROS2 生态系统集成
- **VR 可视化**：可选的 VR 支持，用于调试

## 演示视频

https://github.com/user-attachments/assets/5d51bcc1-133a-4675-80da-6b9ec1d85e5c

视频展示了捏取手势过程中他山传感器的实时可视化，显示拇指和食指传感器的力大小和方向反馈。

## 安装

### 标准安装
```bash
pip install -e .
```

### 支持他山传感器
要使用他山触觉传感器，请安装所需的特定 MuJoCo 版本：
```bash
pip install -e .[tashan]
```
**重要要求：**
- 他山传感器需要 MuJoCo 3.2.3
- 他山传感器需要 Python 3.8
- 不使用 [tashan] 选项时，将使用最新的 MuJoCo 版本，支持 Python ≥ 3.8

**注意**：在现代系统上使用 Python 3.8 与 ROS 可能会有挑战。请参阅文档了解使用 Conda 和 RoboStack 的详细设置说明。或者，使用[独立示例](examples/tashan_standalone_demo.py)而无需 ROS。

## 灵巧手模型

可用的灵巧手模型：

```bash
# 右手（完整碰撞模型）
python nodes/dexrobot_mujoco_ros.py dexrobot_mujoco/models/dexhand021_right.xml

# 右手（简化碰撞模型 - 更快）
python nodes/dexrobot_mujoco_ros.py dexrobot_mujoco/models/dexhand021_right_simplified.xml

# 左手
python nodes/dexrobot_mujoco_ros.py dexrobot_mujoco/models/dexhand021_left.xml
python nodes/dexrobot_mujoco_ros.py dexrobot_mujoco/models/dexhand021_left_simplified.xml

# 浮动基座的手
python nodes/dexrobot_mujoco_ros.py dexrobot_mujoco/models/dexhand021_right_floating.xml

# 安装在机械臂上的手
python nodes/dexrobot_mujoco_ros.py dexrobot_mujoco/models/dexhand021_right_jaka_zu7.xml
```

简化的碰撞模型在保持准确性的同时显著提高了仿真性能：

![灵巧手模型](assets/hands.png)

## 场景组合

创建一个接球场景：

```bash
python nodes/dexrobot_mujoco_ros.py dexrobot_mujoco/scenes/ball_catching.xml --config config/ball_catching.yaml
```

在 YAML 中配置初始位置和跟踪对象：

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

## ROS 接口

使用标准 ROS 消息控制关节：

```python
from sensor_msgs.msg import JointState

# 发布关节指令
msg = JointState()
msg.name = ['r_f_joint1_1']
msg.position = [0.5]
publisher.publish(msg)
```

支持多种格式的数据记录：

```bash
python nodes/dexrobot_mujoco_ros.py model.xml \
    --output-formats ros csv mp4 \
    --output-csv-path data.csv \
    --output-mp4-path video.mp4 \
    --output-bag-path recording.bag
```

## 触觉传感

DexRobot MuJoCo 提供触觉仿真，有两种传感器选项：

### 1. MuJoCo 原生触觉传感器
简单的接触力检测（每个指尖 1 个值）：
```bash
python nodes/dexrobot_mujoco_ros.py dexrobot_mujoco/scenes/ball_catching.xml
```

### 2. 他山触觉传感器
每个传感器垫 11 维触觉反馈：
```bash
# 需要 Python 3.8 和 MuJoCo 3.2.3
python nodes/dexrobot_mujoco_ros.py dexrobot_mujoco/scenes/ball_catching.xml --enable-ts-sensor
```

**他山传感器数据包括：**
- 法向和切向力分量
- 力方向
- 7 通道电容阵列，用于详细的接触几何信息

### 快速演示

尝试带实时可视化的独立触觉演示：
```bash
# 安装可视化依赖
pip install rerun-sdk==0.18.2

# 运行捏取手势演示
python examples/tashan_standalone_demo.py
```

该演示展示了物体操作过程中拇指和食指的力，并提供实时力矢量可视化。

### ROS 集成

传感器数据通过 ROS 话题发布：

- `touch_sensors` (Float32MultiArray) - MuJoCo 触觉传感器或原始 TS 传感器数据
- `ts_forces` (Float32MultiArray) - 使用 `--enable-ts-sensor` 时的语义化力数据

他山力数据以 2D 数组格式为每个传感器垫提供语义信息（法向力、切向力、方向）。

## 文档

完整文档请访问：https://dexrobot.github.io/dexrobot_mujoco

涵盖主题：
- 入门指南
- 触觉传感器集成
- 灵巧手模型与配置
- 场景创建与组合
- ROS 集成与控制
- API 参考

## 许可证

Copyright 2024 DexRobot. 基于 Apache License 2.0 授权。
