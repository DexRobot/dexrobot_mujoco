=============
Scene Examples
=============

This section provides complete examples of scene composition using the DexRobot MuJoCo components.

Box Manipulation Scene
-------------------

A scene with a floating hand manipulating a box on a table (box.xml):

.. code-block:: xml

    <?xml version="1.0" encoding="utf-8"?>
    <mujoco model="box">
        <!-- Include defaults -->
        <include file="../parts/defaults.xml"/>

        <!-- Include assets -->
        <include file="scene_sim/topfloor_scene.xml"/>
        <include file="furniture_sim/simpleTable/simpleTable_asset.xml"/>
        <include file="../parts/dexhand021_right_floating_asset.xml"/>

        <!-- Scene hierarchy -->
        <worldbody>
            <!-- Hand -->
            <body name="floating_hand_base" pos="0 0 1.2">
                <include file="../parts/dexhand021_right_floating_body.xml"/>
            </body>

            <!-- Table -->
            <body name="scenetable" pos="0 0 0" euler="0 0 1.57">
                <include
                    file="furniture_sim/simpleTable/simpleMarbleTable_body.xml"/>
            </body>

            <!-- Box object -->
            <body name="box" pos="0 0 0.82">
                <inertial pos="0 0 0"
                         mass="0.03"
                         diaginertia="0.00002 0.00002 0.00002"/>
                <geom type="box"
                      size="0.03 0.03 0.03"
                      pos="0 0 0"
                      euler="0. 0. 0."
                      rgba="0.49803922 0.72156863 0.87058824 1"/>
                <freejoint/>
            </body>
        </worldbody>

        <!-- Include hand components -->
        <include file="../parts/dexhand021_right_floating_actuator.xml"/>
        <include file="../parts/dexhand021_right_floating_sensor.xml"/>
        <include file="../parts/dexhand021_right_floating_contact.xml"/>
    </mujoco>

Ball Catching Scene
----------------

A scene for ball catching experiments (ball_catching.xml):

.. code-block:: xml

    <?xml version="1.0" encoding="utf-8"?>
    <mujoco model="ball_catching">
        <!-- Include defaults -->
        <include file="../parts/defaults.xml"/>

        <!-- Include assets -->
        <include file="scene_sim/room_scene.xml"/>
        <include file="../parts/dexhand021_right_floating_asset.xml"/>

        <!-- Scene hierarchy -->
        <worldbody>
            <!-- Hand -->
            <body name="floating_hand_base" pos="0 0 1">
                <include file="../parts/dexhand021_right_floating_body.xml"/>
            </body>

            <!-- Ball -->
            <body name="ball" pos="2.0 -0.2 1.0">
                <inertial pos="0 0 0"
                         mass="0.05"
                         diaginertia="0.00004 0.00004 0.00004"/>
                <geom type="sphere"
                      size="0.02"
                      rgba="0.8 0.2 0.2 1"/>
                <freejoint name="ball_joint"/>
            </body>
        </worldbody>

        <!-- Include hand components -->
        <include file="../parts/dexhand021_right_floating_actuator.xml"/>
        <include file="../parts/dexhand021_right_floating_sensor.xml"/>
        <include file="../parts/dexhand021_right_floating_contact.xml"/>
    </mujoco>

Usage Notes
---------

Scene Configuration
^^^^^^^^^^^^^^^
Each scene has a corresponding YAML configuration file:

.. code-block:: yaml

    # ball_catching.yaml
    camera:
      azimuth: -180
      distance: 2.5
      elevation: -25
      lookat: [0.0, 0.0, 0.55]

    tracked_joints:
    - [ARTx, ARTy, ARTz]
    - [ARRx, ARRy, ARRz]
    - [r_f_joint1_1, r_f_joint1_2, r_f_joint1_3, r_f_joint1_4]
    # ... additional joints ...

    initial_qpos_freejoint:
      ball_joint: [2.0, -0.2, 0.0, 1.0, 0.0, 0.0, 0.0]

    initial_qvel_freejoint:
      ball_joint: [-4.1, 0.0, 4.1, 0.0, 0.0, 0.0]

Running Scenes
^^^^^^^^^^^

Launch with configuration:

.. code-block:: bash

    # Box manipulation
    python nodes/dexrobot_mujoco_ros.py \
        dexrobot_mujoco/scenes/box.xml \
        --config config/scene_default.yaml

    # Ball catching
    python nodes/dexrobot_mujoco_ros.py \
        dexrobot_mujoco/scenes/ball_catching.xml \
        --config config/ball_catching.yaml

Common Patterns
------------

1. Asset Organization
   - Include defaults first
   - Group related assets
   - Order from environment to specific components

2. Body Hierarchy
   - Position static elements first
   - Group related components
   - Consider interaction spaces

3. Component Inclusion
   - Keep actuators together
   - Include all required sensors
   - Configure contact properties

4. Scene Configuration
   - Use YAML for runtime settings
   - Configure tracked elements
   - Set initial states

Next Steps
---------

- Review :doc:`composition` for composition techniques
- Study :doc:`furniture` for available components
- Explore :doc:`scenery` for environment options
