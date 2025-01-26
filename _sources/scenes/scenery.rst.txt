===============
Scene Components
===============

Core environment components for scene composition.

Base Environments
---------------

Structure: ``scene_sim/[name]_scene.xml``

.. code-block:: xml

    <!-- Basic environment -->
    <include file="scene_sim/basic_scene.xml"/>

    <!-- Indoor room -->
    <include file="scene_sim/room_scene.xml"/>

Available Types:

- basic_scene: Ground plane and lighting
- room_scene: Enclosed space with windows
- rooftop_scene: Open space with skyline

Standard Elements
---------------

Lighting:

.. code-block:: xml

    <!-- Main lighting -->
    <light name="main" pos="0 0 4" dir="0 0 -1" castshadow="true"/>

    <!-- Window/rim lighting -->
    <light name="window" pos="-2 0 2" dir="1 0 -0.5" castshadow="false"/>

Scenery Meshes:

- simpleroom1.stl: Room geometry
- highwindow.stl: Window frames
- skyline.stl: City backdrop
- wall.stl: Wall sections

Environment Textures:

- dawn/night/cloudy/stormy.png: Sky conditions
- high_contrast_brick.png: Wall surface
- oak/maple_floorboard.png: Floor surfaces

Scene Configuration
----------------

Light Setup:

.. code-block:: xml

    <light name="main" pos="0 0 3" dir="0 0 -1" castshadow="true"/>

Background:

.. code-block:: xml

    <texture name="skybox" file="dawn.png" type="skybox"
             width="800" height="800"/>

Room Structure:

.. code-block:: xml

    <!-- Room geometry -->
    <geom name="room" type="mesh" mesh="simpleroom1"
          material="wall" contype="0" conaffinity="0"/>

    <!-- Window -->
    <geom name="window" type="mesh" mesh="highwindow"
          material="glass" rgba="0.9 0.9 1 0.3"/>
