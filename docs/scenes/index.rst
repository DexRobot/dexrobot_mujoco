=================
Scene Creation
=================

This section covers scene creation in DexRobot MuJoCo using the include-based composition system.

.. toctree::
   :maxdepth: 2
   :caption: Contents

   composition
   furniture
   scenery
   examples

Overview
--------

The scene system in DexRobot MuJoCo uses MuJoCo's ``include`` directive to compose scenes from modular components:

- Scene XML files reference other XML files using ``<include>``
- Components are organized into reusable parts
- Models maintain separate asset, body, actuator, and sensor definitions

Example Scene Structure
--------------------

A typical scene XML follows this pattern:

.. code-block:: xml

    <?xml version="1.0" encoding="utf-8"?>
    <mujoco model="scene_name">
        <!-- Include defaults -->
        <include file="../parts/defaults.xml"/>

        <!-- Include assets -->
        <include file="scene_sim/environment_scene.xml"/>
        <include file="furniture_sim/table/table_asset.xml"/>
        <include file="../parts/dexhand021_right_floating_asset.xml"/>

        <!-- Define scene hierarchy -->
        <worldbody>
            <!-- Add hand -->
            <body name="floating_hand_base" pos="0 0 1.2">
                <include file="../parts/dexhand021_right_floating_body.xml"/>
            </body>

            <!-- Add furniture -->
            <body name="table" pos="0 0 0">
                <include file="furniture_sim/table/table_body.xml"/>
            </body>

            <!-- Add objects -->
            <body name="object" pos="0 0 0.8">
                <!-- Object definition -->
            </body>
        </worldbody>

        <!-- Include additional components -->
        <include file="../parts/dexhand021_right_floating_actuator.xml"/>
        <include file="../parts/dexhand021_right_floating_sensor.xml"/>
        <include file="../parts/dexhand021_right_floating_contact.xml"/>
    </mujoco>

Available Components
-----------------

Hand Components
^^^^^^^^^^^^^
Located in ``parts/``:

- Floating hand components: ``dexhand021_right_floating_*.xml``
- Hand-arm components: e.g., ``dexhand021_right_jaka_zu7_*.xml``

Environment Components
^^^^^^^^^^^^^^^^^^^
Located in ``scenes/scene_sim/``:

- ``basic_scene.xml``: Simple environment
- ``room_scene.xml``: Indoor room
- ``topfloor_scene.xml``: Rooftop environment

Furniture Components
^^^^^^^^^^^^^^^^^
Located in ``scenes/furniture_sim/``:

- Tables (``simpleTable/``, ``studyTable/``, ``ventionTable/``)
- Cabinets (``hingecabinet/``, ``slidecabinet/``)
- Appliances (``microwave/``, ``oven/``, ``kettle/``)
- Storage (``bin/``, ``counters/``)

Component Organization
-------------------

Each model typically separates its definitions into:

- ``*_asset.xml``: Mesh, texture, and material definitions
- ``*_body.xml``: Physical structure and properties
- ``*_actuator.xml``: Actuator configurations
- ``*_sensor.xml``: Sensor definitions
- ``*_contact.xml``: Contact properties

This modular organization allows:

- Selective inclusion of components
- Clear separation of concerns
- Reuse across different scenes
- Easy modification of individual aspects

Next Steps
---------

Continue reading:

- :doc:`composition` - How to compose scenes using includes
- :doc:`furniture` - Available furniture components
- :doc:`scenery` - Environment components
- :doc:`examples` - Example scene compositions
