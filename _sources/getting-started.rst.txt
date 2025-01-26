===============
Getting Started
===============

This guide will help you install and run DexRobot MuJoCo, and walk you through basic usage patterns.

Prerequisites
------------

Required Software
^^^^^^^^^^^^^^^
- Python >= 3.8
- MuJoCo >= 3.0.0
- ROS or ROS2 (any distribution)

Python Dependencies
^^^^^^^^^^^^^^^^^
- mujoco >= 3.0.0
- numpy
- pyyaml
- loguru
- scipy
- flask (for VR visualization)
- opencv-python
- pandas
- ros_compat

Installation
-----------

1. Set up your ROS environment (ROS1 or ROS2)

2. Install Python dependencies:

   .. code-block:: bash

       pip install mujoco numpy pyyaml loguru scipy flask opencv-python pandas
       pip install git+https://gitee.com/dexrobot/ros_compat.git

3. Install DexRobot MuJoCo:

   .. code-block:: bash

       pip install git+https://gitee.com/dexrobot/dexrobot_mujoco.git

   Or for development:

   .. code-block:: bash

       git clone https://gitee.com/dexrobot/dexrobot_mujoco.git
       cd dexrobot_mujoco
       pip install -e .[dev]

Quick Start
----------

Basic Hand Control
^^^^^^^^^^^^^^^^

1. Launch the simulation with a simplified hand model:

   .. code-block:: bash

       python nodes/dexrobot_mujoco_ros.py dexrobot_mujoco/models/dexhand021_right_simplified.xml

2. In another terminal, send joint commands:

   .. code-block:: bash

       # Using ROS2
       ros2 topic pub /joint_commands sensor_msgs/msg/JointState \
           "{name: ['r_f_joint1_1'], position: [0.5]}"

       # Using ROS1
       rostopic pub /joint_commands sensor_msgs/JointState \
           "{name: ['r_f_joint1_1'], position: [0.5]}"

Simulating a Scene
^^^^^^^^^^^^^^^^

1. Launch a scene with configuration:

   .. code-block:: bash

       python nodes/dexrobot_mujoco_ros.py dexrobot_mujoco/scenes/box.xml \
           --config config/scene_default.yaml

2. Save the simulation:

   .. code-block:: bash

       python nodes/dexrobot_mujoco_ros.py dexrobot_mujoco/scenes/box.xml \
           --config config/scene_default.yaml \
           --output-formats ros csv mp4 \
           --output-csv-path data.csv \
           --output-mp4-path video.mp4

Key Command-line Options
----------------------

The ROS node supports various command-line options:

.. code-block:: text

    Required Arguments:
      model_path             Path to the Mujoco model XML file

    Optional Arguments:
      --config-yaml         Path to the YAML configuration file
      --replay-csv          Path to CSV file for replay
      --hand-pose-topic     Topic name for hand pose control
      --position-magnifiers Position magnifiers for hand pose control [default: 2.5,2.0,0.8]
      --output-formats      Output formats (ros/csv/mp4) [default: ros]
      --output-csv-path     Path for CSV output
      --output-mp4-path     Path for MP4 output
      --output-bag-path     Path for ROS bag output
      --enable-vr           Enable VR visualization
      --renderer-dimension  Renderer dimensions as width,height (e.g., 640,480)
      --seed               Random seed [default: 0]

Basic Configuration
-----------------

Scene configuration example (config/scene_default.yaml):

.. code-block:: yaml

    # Camera settings
    camera:
      azimuth: 0
      distance: 1.2
      elevation: -20
      lookat: [0.0, 0.0, 1.2]

    # Track these joints
    tracked_joints:
    - [ARTx, ARTy, ARTz]
    - [r_f_joint1_1, r_f_joint1_2, r_f_joint1_3, r_f_joint1_4]
    # ... more joints ...

    # Track these bodies
    tracked_bodies:
    - [right_hand_base]
    - [r_f_link1_1, r_f_link1_2, r_f_link1_3, r_f_link1_4]
    # ... more bodies ...

Next Steps
---------

After getting familiar with basic usage, you might want to:

- Learn about :doc:`hand model configuration </hand_models/index>`
- Create custom :doc:`simulation scenes </scenes/index>`
- Explore advanced :doc:`ROS integration </ros_integration/index>`
- Check the :doc:`API reference </api/mj_control_wrapper>` for programmatic control

Common Issues
------------

ModuleNotFoundError: No module named 'mujoco'
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Make sure MuJoCo is properly installed:

.. code-block:: bash

    pip uninstall mujoco
    pip install mujoco>=3.0.0

ROS messages not being received/sent
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Check if your ROS environment is properly sourced and ROS_DOMAIN_ID is set correctly.

Low simulation performance
^^^^^^^^^^^^^^^^^^^^^^
Consider using the simplified collision model:

.. code-block:: bash

    python nodes/dexrobot_mujoco_ros.py \
        dexrobot_mujoco/models/dexhand021_right_simplified.xml
