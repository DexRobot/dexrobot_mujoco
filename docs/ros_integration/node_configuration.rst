====================
Node Configuration
====================

This section covers the configuration options and setup of the DexRobot MuJoCo ROS node.

Command Line Options
-----------------

Required Arguments
^^^^^^^^^^^^^^^
- ``model_path``: Path to the MuJoCo model XML file

Optional Arguments
^^^^^^^^^^^^^^^

General Options
~~~~~~~~~~~~~
.. code-block:: text

    --config-yaml PATH        YAML configuration file path
    --seed N                  Random seed [default: 0]

Input Configuration
~~~~~~~~~~~~~~~~
.. code-block:: text

    --replay-csv PATH         CSV file to replay instead of ROS input
    --hand-pose-topic NAME    Topic name for hand pose (enables 6-DoF control)
    --position-magnifiers X,Y,Z  Position scaling factors [default: 2.5,2.0,0.8]

Output Configuration
^^^^^^^^^^^^^^^^^
.. code-block:: text

    --output-formats LIST     Output formats (ros/csv/mp4) [default: ros]
    --output-csv-path PATH    CSV output file path
    --output-mp4-path PATH    MP4 output file path
    --output-bag-path PATH    ROS bag output path
    --additional-bag-topics LIST  Additional topics to record in bag

Visualization
^^^^^^^^^^^
.. code-block:: text

    --enable-vr              Enable VR visualization
    --renderer-dimension W,H  Renderer dimensions (e.g., 640,480)
    --enable-ts-sensor       Use TaShan sensors instead of MuJoCo touch sensors

YAML Configuration
----------------

The configuration file specifies initial states, camera settings, and tracked objects:

.. code-block:: yaml

    # Camera configuration
    camera:
      azimuth: 0
      distance: 1.2
      elevation: -20
      lookat: [0.0, 0.0, 1.2]
    
    # Initial joint positions
    initial_ctrl:
      act_r_f_joint1_1: 0.5
      act_r_f_joint2_1: 0.3
    
    # Tracked objects for output
    tracked_joints:
      - [r_f_joint1_1, r_f_joint1_2]
    tracked_bodies:
      - [right_hand_base]
    tracked_sensors:
      - [touch_r_f_link1_pad]

Data Recording
--------------

The node supports three recording formats that can be used simultaneously:

CSV Recording
^^^^^^^^^^^^^

.. code-block:: bash

    python nodes/dexrobot_mujoco_ros.py model.xml \
        --output-formats csv \
        --output-csv-path data.csv

**Data includes:** Timestamps, joint positions/velocities, body poses, sensor readings

ROS Bag Recording
^^^^^^^^^^^^^^^^^

.. code-block:: bash

    python nodes/dexrobot_mujoco_ros.py model.xml \
        --output-formats ros \
        --output-bag-path recording.bag

**Recorded topics:** ``joint_commands``, ``joint_states``, ``body_poses``, ``touch_sensors``

MP4 Video Recording
^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

    python nodes/dexrobot_mujoco_ros.py model.xml \
        --output-formats mp4 \
        --output-mp4-path video.mp4 \
        --renderer-dimension 1920,1080

**Parameters:** 20 FPS fixed rate, mp4v codec

VR Visualization
----------------

Enable stereoscopic web streaming:

.. code-block:: bash

    python nodes/dexrobot_mujoco_ros.py model.xml --enable-vr

**Access:** Open browser to ``http://localhost:5000/video``

**Features:**
- Stereoscopic rendering with 0.1m eye separation
- 20 FPS streaming
- Synchronized with MuJoCo viewer camera

Camera Settings
^^^^^^^^^^^^
Configure viewer camera:

.. code-block:: yaml

    camera:
      azimuth: 0          # Horizontal angle
      distance: 1.2       # Distance from target
      elevation: -20      # Vertical angle
      lookat:            # Target point
        - 0.0
        - 0.0
        - 1.2

Joint Tracking
^^^^^^^^^^^^
Specify joints to track:

.. code-block:: yaml

    tracked_joints:
      - [ARTx, ARTy, ARTz]                        # Translation
      - [ARRx, ARRy, ARRz]                        # Rotation
      - [r_f_joint1_1, r_f_joint1_2, r_f_joint1_3] # Finger joints

Body Tracking
^^^^^^^^^^^
Specify bodies to track:

.. code-block:: yaml

    tracked_bodies:
      - [right_hand_base]                         # Hand base
      - [r_f_link1_1, r_f_link1_2, r_f_link1_3]  # Finger links

Sensor Tracking
^^^^^^^^^^^^
Specify sensors to track:

.. code-block:: yaml

    tracked_sensors:
      - [touch_r_f_link1_4]  # Fingertip sensors
      - [touch_r_f_link2_4]
      - [touch_r_f_link3_4]

Initial States
^^^^^^^^^^^^
Set initial joint positions:

.. code-block:: yaml

    initial_qpos:
      r_f_joint1_1: 0.5
      r_f_joint1_2: 0.7

Set initial pose for free joints:

.. code-block:: yaml

    initial_qpos_freejoint:
      ball_joint: [2.0, -0.2, 0.0, 1.0, 0.0, 0.0, 0.0]  # pos + quat

Set initial velocities:

.. code-block:: yaml

    initial_qvel_freejoint:
      ball_joint: [-4.1, 0.0, 4.1, 0.0, 0.0, 0.0]  # linear + angular

Node Initialization
----------------

The node initializes in the following sequence:

1. Process command line arguments
2. Load MuJoCo model
3. Parse YAML configuration (if provided)
4. Set up ROS publishers/subscribers
5. Configure data recording (if enabled)
6. Start VR visualization (if enabled)
7. Begin simulation loop

Example Configurations
-------------------

Basic Scene
^^^^^^^^^
Minimal configuration for visualization:

.. code-block:: yaml

    # config/scene_default.yaml
    camera:
      azimuth: 0
      distance: 1.2
      elevation: -20
      lookat: [0.0, 0.0, 1.2]

    tracked_joints:
      - [r_f_joint1_1, r_f_joint1_2, r_f_joint1_3, r_f_joint1_4]

    tracked_bodies:
      - [right_hand_base]
      - [r_f_link1_4]  # Thumb tip

Data Recording
^^^^^^^^^^^
Configuration with all outputs enabled:

.. code-block:: bash

    python nodes/dexrobot_mujoco_ros.py model.xml \
        --config config/scene_default.yaml \
        --output-formats ros csv mp4 \
        --output-csv-path output/data.csv \
        --output-mp4-path output/video.mp4 \
        --output-bag-path output/recording.bag \
        --additional-bag-topics /camera/image_raw

VR Visualization
^^^^^^^^^^^^^
Configuration for VR control:

.. code-block:: bash

    python nodes/dexrobot_mujoco_ros.py model.xml \
        --config config/scene_default.yaml \
        --enable-vr \
        --hand-pose-topic hand_pose \
        --renderer-dimension 1920,1080


Next Steps
---------

- Learn about available :doc:`topics_and_services`
- Configure :doc:`data_recording`
- Set up :doc:`vr_visualization`
