===================
ROS Integration
===================

This section covers the ROS integration capabilities of DexRobot MuJoCo for control and monitoring through ROS topics and services.

.. toctree::
   :maxdepth: 2
   :caption: Contents

   node_configuration
   topics_and_services

Overview
--------

The ROS node (``dexrobot_mujoco_ros.py``) provides:

- Joint and pose control interfaces
- State feedback and monitoring
- Data recording (CSV, ROS bags, MP4)
- VR visualization capabilities
- Automatic ROS1/ROS2 compatibility via ros_compat

Core Features
-----------

Joint Control
^^^^^^^^^^^
- Direct joint position control via ``joint_commands``
- Hand pose control with 6-DoF floating base
- Position scaling and velocity damping
- Real-time position tracking

State Feedback
^^^^^^^^^^^^
- Joint state publishing
- Body pose tracking
- Touch sensor data
- Custom sensor outputs

Data Recording
^^^^^^^^^^^^
- CSV file recording
- ROS bag recording
- MP4 video recording
- Synchronized multi-modal data

VR Visualization
^^^^^^^^^^^^^
- Stereo image rendering
- Web-based visualization
- Real-time streaming
- Adjustable viewpoints

Quick Start
----------

Basic Usage
^^^^^^^^^

1. Launch the node:

   .. code-block:: bash

       python nodes/dexrobot_mujoco_ros.py model.xml

2. Send joint commands:

   .. code-block:: bash

       # ROS2
       ros2 topic pub /joint_commands sensor_msgs/msg/JointState \
           "{name: ['r_f_joint1_1'], position: [0.5]}"

       # ROS1
       rostopic pub /joint_commands sensor_msgs/JointState \
           "{name: ['r_f_joint1_1'], position: [0.5]}"

3. Monitor states:

   .. code-block:: bash

       # ROS2
       ros2 topic echo /joint_states
       ros2 topic echo /body_poses
       ros2 topic echo /touch_sensors

       # ROS1
       rostopic echo /joint_states
       rostopic echo /body_poses
       rostopic echo /touch_sensors

Advanced Usage
^^^^^^^^^^^

Enable VR and recording:

.. code-block:: bash

    python nodes/dexrobot_mujoco_ros.py model.xml \
        --enable-vr \
        --output-formats ros csv mp4 \
        --output-csv-path data.csv \
        --output-mp4-path video.mp4 \
        --output-bag-path recording.bag

Next Steps
---------

Continue reading:

- :doc:`node_configuration` - Detailed node setup and configuration
- :doc:`topics_and_services` - Available ROS interfaces
