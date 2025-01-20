===================
Scenery Components
===================

This section documents the environment and scenery components available in ``scenes/scene_sim/``.

Base Environments
--------------

Basic Scene
^^^^^^^^^
Minimal environment setup:

.. code-block:: xml

    <!-- Include environment -->
    <include file="scene_sim/basic_scene.xml"/>

Features:
- Simple ground plane
- Basic lighting
- Minimal visual elements
- Efficient performance

Room Scene
^^^^^^^^
Indoor room environment:

.. code-block:: xml

    <!-- Include environment -->
    <include file="scene_sim/room_scene.xml"/>

Features:
- Enclosed space
- Windows and walls
- Interior lighting
- Floor textures

Rooftop Scene
^^^^^^^^^^^
Open rooftop environment:

.. code-block:: xml

    <!-- Include environment -->
    <include file="scene_sim/rooftop_scene.xml"/>

Features:
- Skyline backdrop
- Natural lighting
- Open space
- Atmospheric effects

Visual Elements
-------------

Textures
^^^^^^^
Available environment textures:

- ``dawn.png``: Early morning lighting
- ``night.png``: Evening/dark setting
- ``cloudy.png``: Overcast conditions
- ``stormy.png``: Dramatic weather
- ``high_contrast_brick.png``: Wall texture
- ``oak_floorboard.png``: Floor texture
- ``maple_floorboard.png``: Alternative floor
- ``white_marble_tile2.png``: Tile texture

Usage:
    .. code-block:: xml

        <texture name="floor"
                file="oak_floorboard.png"
                type="2d"/>

Materials
^^^^^^^^
Standard material configurations:

.. code-block:: xml

    <!-- Floor material -->
    <material name="floor"
             texture="oak_floorboard"
             texrepeat="8 8"
             rgba="1 1 1 1"/>

    <!-- Wall material -->
    <material name="wall"
             texture="high_contrast_brick"
             texrepeat="4 4"
             rgba="1 1 1 1"/>

Lighting
^^^^^^^
Lighting configurations:

.. code-block:: xml

    <!-- General lighting -->
    <light name="top"
           pos="0 0 4"
           dir="0 0 -1"
           castshadow="true"/>

    <!-- Window lighting -->
    <light name="window"
           pos="-2 0 2"
           dir="1 0 -0.5"
           castshadow="false"/>

Meshes
^^^^^
Available scenery meshes:

- ``simpleroom1.stl``: Basic room geometry
- ``highwindow.stl``: Window frame
- ``skyline.stl``: City backdrop
- ``wall.stl``: Wall sections
- ``logo.obj``: Decorative elements

Usage:
    .. code-block:: xml

        <mesh name="room"
              file="simpleroom1.stl"
              scale="1 1 1"/>

Environment Setup
--------------

Basic Configuration
^^^^^^^^^^^^^^^^

1. Include environment:

   .. code-block:: xml

       <include file="scene_sim/basic_scene.xml"/>

2. Configure lighting:

   .. code-block:: xml

       <light name="main"
              pos="0 0 3"
              dir="0 0 -1"
              castshadow="true"/>

3. Set ground properties:

   .. code-block:: xml

       <geom name="floor"
             type="plane"
             size="5 5 0.1"
             material="floor"/>

Advanced Configuration
^^^^^^^^^^^^^^^^^^^

1. Custom skybox:

   .. code-block:: xml

       <texture name="skybox"
               file="dawn.png"
               type="skybox"
               width="800"
               height="800"/>

2. Multiple light sources:

   .. code-block:: xml

       <light name="key"
              pos="3 1 4"
              dir="-1 0 -1"
              castshadow="true"/>

       <light name="fill"
              pos="-2 -1 3"
              dir="1 0 -1"
              castshadow="false"/>

       <light name="rim"
              pos="0 -2 2"
              dir="0 1 -0.5"
              castshadow="false"/>

3. Room configuration:

   .. code-block:: xml

       <!-- Load room mesh -->
       <geom name="room"
             type="mesh"
             mesh="simpleroom1"
             material="wall"
             rgba="1 1 1 1"
             contype="0"
             conaffinity="0"/>

       <!-- Add window -->
       <geom name="window"
             type="mesh"
             mesh="highwindow"
             material="glass"
             rgba="0.9 0.9 1 0.3"/>

Best Practices
------------

Lighting Setup
^^^^^^^^^^^
1. Use multiple light sources
2. Configure shadows appropriately
3. Balance intensity and color
4. Consider performance impact

Visual Quality
^^^^^^^^^^^^
1. Choose appropriate texture resolutions
2. Configure material properties
3. Set reasonable view distances
4. Balance quality and performance

Performance
^^^^^^^^^^
1. Minimize shadow-casting lights
2. Use efficient mesh geometries
3. Optimize texture sizes
4. Disable unnecessary features

Common Issues
-----------

Lighting Problems
^^^^^^^^^^^^^^
If experiencing lighting issues:

1. Check light positions
2. Verify shadow settings
3. Adjust intensities
4. Review material properties

Texture Problems
^^^^^^^^^^^^^
If textures don't appear correctly:

1. Verify file paths
2. Check texture dimensions
3. Configure UV mapping
4. Review material settings

Performance Issues
^^^^^^^^^^^^^^^
If performance is poor:

1. Reduce shadow-casting lights
2. Optimize mesh complexity
3. Adjust texture sizes
4. Minimize reflection/refraction

Next Steps
---------

- Review :doc:`composition` for scene creation
- Study complete examples in :doc:`examples`
