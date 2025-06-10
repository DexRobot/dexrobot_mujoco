===================
Scene Composition
===================

This section explains how to compose scenes using MuJoCo's include system and the DexRobot MuJoCo component library.

Basic Structure
-------------

A scene XML file consists of several key sections:

1. Default inclusions
2. Asset inclusions
3. World body definitions
4. Component inclusions

Example Scene
-----------

Here's a complete example (based on box.xml):

.. code-block:: xml

    <?xml version="1.0" encoding="utf-8"?>
    <mujoco model="box">
        <!-- Include defaults first -->
        <include file="../parts/defaults.xml"/>

        <!-- Include required assets -->
        <include file="scene_sim/topfloor_scene.xml"/>
        <include file="furniture_sim/simpleTable/simpleTable_asset.xml"/>
        <include file="../parts/dexhand021_right_floating_asset.xml"/>

        <!-- Define scene hierarchy -->
        <worldbody>
            <!-- Add floating hand -->
            <body name="floating_hand_base" pos="0 0 1.2">
                <include file="../parts/dexhand021_right_floating_body.xml"/>
            </body>

            <!-- Add table -->
            <body name="scenetable" pos="0 0 0" euler="0 0 1.57">
                <include
                    file="furniture_sim/simpleTable/simpleMarbleTable_body.xml"/>
            </body>

            <!-- Add manipulatable object -->
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

        <!-- Include remaining hand components -->
        <include file="../parts/dexhand021_right_floating_actuator.xml"/>
        <include file="../parts/dexhand021_right_floating_sensor.xml"/>
        <include file="../parts/dexhand021_right_floating_contact.xml"/>
    </mujoco>

Component Types
-------------

Defaults
^^^^^^^
Always include defaults first:

.. code-block:: xml

    <include file="../parts/defaults.xml"/>

This sets up standard parameters for physics, rendering, and contacts.

Assets
^^^^^
Include all required visual assets:

.. code-block:: xml

    <!-- Environment -->
    <include file="scene_sim/topfloor_scene.xml"/>

    <!-- Furniture -->
    <include file="furniture_sim/simpleTable/simpleTable_asset.xml"/>

    <!-- Hand -->
    <include file="../parts/dexhand021_right_floating_asset.xml"/>

Bodies
^^^^^
Add physical bodies to the scene:

.. code-block:: xml

    <body name="component_name" pos="x y z" euler="r p y">
        <include file="path/to/component_body.xml"/>
    </body>

The body tag sets:

- ``name``: Unique identifier
- ``pos``: Position [x y z]
- ``euler``: Orientation [roll pitch yaw]

Additional Components
^^^^^^^^^^^^^^^^^^
Include remaining components after worldbody:

.. code-block:: xml

    <include file="path/to/component_actuator.xml"/>
    <include file="path/to/component_sensor.xml"/>
    <include file="path/to/component_contact.xml"/>

Building Scenes
-------------

Step-by-Step Process
^^^^^^^^^^^^^^^^^

1. Start with model declaration:

   .. code-block:: xml

       <?xml version="1.0" encoding="utf-8"?>
       <mujoco model="scene_name">

2. Include defaults:

   .. code-block:: xml

       <include file="../parts/defaults.xml"/>

3. Include all required assets:

   .. code-block:: xml

       <include file="scene_sim/environment.xml"/>
       <include file="path/to/component1_asset.xml"/>
       <include file="path/to/component2_asset.xml"/>

4. Define worldbody with components:

   .. code-block:: xml

       <worldbody>
           <body name="component1" pos="x1 y1 z1">
               <include file="path/to/component1_body.xml"/>
           </body>
           <body name="component2" pos="x2 y2 z2">
               <include file="path/to/component2_body.xml"/>
           </body>
       </worldbody>

5. Include remaining component files:

   .. code-block:: xml

       <include file="path/to/component1_actuator.xml"/>
       <include file="path/to/component1_sensor.xml"/>
       <include file="path/to/component1_contact.xml"/>

Common Patterns
------------

Floating Hand Scene
^^^^^^^^^^^^^^^^
Basic setup for hand manipulation:

.. code-block:: xml

    <mujoco model="hand_scene">
        <include file="../parts/defaults.xml"/>
        <include file="scene_sim/room_scene.xml"/>
        <include file="../parts/dexhand021_right_floating_asset.xml"/>

        <worldbody>
            <body name="floating_hand_base" pos="0 0 1">
                <include file="../parts/dexhand021_right_floating_body.xml"/>
            </body>
        </worldbody>

        <include file="../parts/dexhand021_right_floating_actuator.xml"/>
        <include file="../parts/dexhand021_right_floating_sensor.xml"/>
        <include file="../parts/dexhand021_right_floating_contact.xml"/>
    </mujoco>

Hand-Arm Scene
^^^^^^^^^^^
Setup for mounted hand:

.. code-block:: xml

    <mujoco model="arm_scene">
        <include file="../parts/defaults.xml"/>
        <include file="scene_sim/room_scene.xml"/>
        <include file="../parts/dexhand021_right_jaka_zu7_asset.xml"/>

        <worldbody>
            <body name="robot_base" pos="0 0 0">
                <include file="../parts/dexhand021_right_jaka_zu7_body.xml"/>
            </body>
        </worldbody>

        <include file="../parts/dexhand021_right_jaka_zu7_actuator.xml"/>
        <include file="../parts/dexhand021_right_jaka_zu7_sensor.xml"/>
        <include file="../parts/dexhand021_right_jaka_zu7_contact.xml"/>
    </mujoco>

Available Components
------------------

Furniture Components
^^^^^^^^^^^^^^^^^^

Tables
~~~~~~

Structure: ``furniture_sim/[type]/[type]_{asset,body}.xml``

.. code-block:: xml

    <!-- Assets and materials -->
    <include file="furniture_sim/simpleTable/simpleTable_asset.xml"/>

    <!-- Physical structure (pick one): -->
    <include file="furniture_sim/simpleTable/simpleGraniteTable_body.xml"/>
    <include file="furniture_sim/simpleTable/simpleMarbleTable_body.xml"/>
    <include file="furniture_sim/simpleTable/simpleWoodTable_body.xml"/>

Available:
- simpleTable: Basic surface with material variants
- studyTable: Surface with drawer/handle mechanics
- ventionTable: Industrial frame structure

Storage Units
~~~~~~~~~~~

.. code-block:: xml

    <!-- Cabinet with hinged door -->
    <include file="furniture_sim/hingecabinet/hingecabinet_asset.xml"/>
    <include file="furniture_sim/hingecabinet/hingecabinet_body.xml"/>

    <!-- Cabinet with sliding door -->
    <include file="furniture_sim/slidecabinet/slidecabinet_asset.xml"/>
    <include file="furniture_sim/slidecabinet/slidecabinet_body.xml"/>

Available:
- hingecabinet: Hinged door mechanics
- slidecabinet: Linear sliding mechanism
- counters: Kitchen counter units
- bin: Storage containers

Appliances
~~~~~~~~~

.. code-block:: xml

    <!-- Microwave with version selection -->
    <include file="furniture_sim/microwave/microwave_asset.xml"/>
    <include file="furniture_sim/microwave/microwave_body0.xml"/> <!-- Versions 0-3 -->

    <!-- Kettle with version selection -->
    <include file="furniture_sim/kettle/kettle_asset.xml"/>
    <include file="furniture_sim/kettle/kettle_body0.xml"/> <!-- Versions 0-7 -->

Available:
- microwave: Door and control panel mechanics
- kettle: Handle and pour mechanics
- oven: Door, knob and burner mechanics

Environment Components
^^^^^^^^^^^^^^^^^^^^

Base Scenes
~~~~~~~~~

Structure: ``scene_sim/[name]_scene.xml``

.. code-block:: xml

    <!-- Basic environment -->
    <include file="scene_sim/basic_scene.xml"/>

    <!-- Indoor room -->
    <include file="scene_sim/room_scene.xml"/>

    <!-- Rooftop -->
    <include file="scene_sim/rooftop_scene.xml"/>

Available:
- basic_scene: Ground plane and lighting
- room_scene: Enclosed space with windows
- rooftop_scene: Open space with skyline

Lighting
~~~~~~~

.. code-block:: xml

    <!-- Main lighting -->
    <light name="main" pos="0 0 4" dir="0 0 -1" castshadow="true"/>

    <!-- Window/rim lighting -->
    <light name="window" pos="-2 0 2" dir="1 0 -0.5" castshadow="false"/>

Textures
~~~~~~~

Standard textures:
- Materials: wood[0-4].png, metal[0-4].png, stone[0-4].png
- Environment: dawn/night/cloudy/stormy.png (skyboxes)
- Surfaces: oak/maple_floorboard.png, high_contrast_brick.png

Next Steps
---------

- Study complete examples in :doc:`examples`
