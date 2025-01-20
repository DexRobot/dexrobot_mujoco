===================
Furniture Components
===================

This section documents the furniture components available in ``scenes/furniture_sim/``.

Tables
-----

Simple Table
^^^^^^^^^^
Basic table model with different surface materials:

.. code-block:: xml

    <!-- Include assets -->
    <include file="furniture_sim/simpleTable/simpleTable_asset.xml"/>

    <!-- Add table body -->
    <body name="table" pos="0 0 0">
        <!-- Choose one: -->
        <include file="furniture_sim/simpleTable/simpleGraniteTable_body.xml"/>
        <include file="furniture_sim/simpleTable/simpleMarbleTable_body.xml"/>
        <include file="furniture_sim/simpleTable/simpleWoodTable_body.xml"/>
    </body>

Properties:
- Fixed height
- Multiple surface materials
- Stable base
- Suitable for object placement

Study Table
^^^^^^^^^
Table with drawers:

.. code-block:: xml

    <!-- Include assets -->
    <include file="furniture_sim/studyTable/studyTable_asset.xml"/>

    <!-- Add table body -->
    <body name="study_table" pos="0 0 0">
        <include file="furniture_sim/studyTable/studyTable_body.xml"/>
    </body>

Features:
- Sliding drawers
- Handle interaction
- Wood texture
- Complex articulation

Vention Table
^^^^^^^^^^^
Industrial-style table:

.. code-block:: xml

    <!-- Include assets -->
    <include file="furniture_sim/ventionTable/ventionTable_asset.xml"/>

    <!-- Add table body -->
    <body name="vention_table" pos="0 0 0">
        <include file="furniture_sim/ventionTable/ventionTable_body.xml"/>
    </body>

Features:
- Metal construction
- Industrial aesthetic
- High stability
- Durable surface

Cabinets
-------

Hinge Cabinet
^^^^^^^^^^^
Cabinet with hinged door:

.. code-block:: xml

    <!-- Include assets -->
    <include file="furniture_sim/hingecabinet/hingecabinet_asset.xml"/>

    <!-- Add cabinet body -->
    <body name="hinge_cabinet" pos="0 0 0">
        <include file="furniture_sim/hingecabinet/hingecabinet_body.xml"/>
    </body>

Features:
- Swinging door
- Interior storage
- Handle interaction
- Configurable hinge properties

Slide Cabinet
^^^^^^^^^^^
Cabinet with sliding door:

.. code-block:: xml

    <!-- Include assets -->
    <include file="furniture_sim/slidecabinet/slidecabinet_asset.xml"/>

    <!-- Add cabinet body -->
    <body name="slide_cabinet" pos="0 0 0">
        <include file="furniture_sim/slidecabinet/slidecabinet_body.xml"/>
    </body>

Features:
- Sliding mechanism
- Linear motion
- Interior access
- Smooth operation

Box Cabinet
^^^^^^^^^
Simple storage cabinet:

.. code-block:: xml

    <!-- Include assets -->
    <include file="furniture_sim/boxcabinet/boxcabinet_assets.xml"/>

    <!-- Add cabinet body -->
    <body name="box_cabinet" pos="0 0 0">
        <include file="furniture_sim/boxcabinet/boxcabinet_body.xml"/>
    </body>

Features:
- Basic storage
- Fixed structure
- Simple design
- Efficient collision model

Appliances
--------

Microwave
^^^^^^^^
Interactive microwave model:

.. code-block:: xml

    <!-- Include assets -->
    <include file="furniture_sim/microwave/microwave_asset.xml"/>

    <!-- Add microwave body -->
    <body name="microwave" pos="0 0 0">
        <!-- Choose version 0-3 -->
        <include file="furniture_sim/microwave/microwave_body0.xml"/>
    </body>

Features:
- Door interaction
- Button panel
- Multiple versions
- Interior space

Kettle
^^^^^
Electric kettle model:

.. code-block:: xml

    <!-- Include assets -->
    <include file="furniture_sim/kettle/kettle_asset.xml"/>

    <!-- Add kettle body -->
    <body name="kettle" pos="0 0 0">
        <!-- Choose version 0-7 -->
        <include file="furniture_sim/kettle/kettle_body0.xml"/>
    </body>

Features:
- Handle grasp points
- Multiple versions
- Pour simulation
- Lid interaction

Oven
^^^^
Kitchen oven model:

.. code-block:: xml

    <!-- Include assets -->
    <include file="furniture_sim/oven/oven_asset.xml"/>

    <!-- Add oven body -->
    <body name="oven" pos="0 0 0">
        <include file="furniture_sim/oven/oven_body.xml"/>
    </body>

Features:
- Door interaction
- Control knobs
- Burner plates
- Hood element

Storage
------

Counters
^^^^^^^
Kitchen counter units:

.. code-block:: xml

    <!-- Include assets -->
    <include file="furniture_sim/counters/counters_asset.xml"/>

    <!-- Add counter bodies -->
    <body name="sink_counter" pos="0 0 0">
        <include file="furniture_sim/counters/sink_counter_body.xml"/>
    </body>
    <body name="box_counter" pos="1 0 0">
        <include file="furniture_sim/counters/box_counter_body.xml"/>
    </body>

Features:
- Sink option
- Storage drawers
- Cabinet doors
- Modular design

Bin
^^^
Storage bin models:

.. code-block:: xml

    <!-- Include assets -->
    <include file="furniture_sim/bin/bin_asset.xml"/>

    <!-- Add bin body -->
    <body name="bin" pos="0 0 0">
        <include file="furniture_sim/bin/bin_body.xml"/>
        <!-- Or business bin -->
        <include file="furniture_sim/bin/busbin1_body.xml"/>
    </body>

Features:
- Multiple styles
- Open top
- Interior volume
- Collision properties

Common Elements
-------------

Materials
^^^^^^^^
Shared textures in ``common/textures/``:

- ``wood[0-4].png``
- ``metal[0-4].png``
- ``stone[0-4].png``

Usage:
    .. code-block:: xml

        <material name="custom_wood"
                 texture="wood2"
                 rgba="1 1 1 1"/>

Contact Properties
^^^^^^^^^^^^^^^
Standard contact settings:

.. code-block:: xml

    <contact>
        <pair geom1="hand"
              geom2="furniture"
              friction="1 0.005 0.0001"/>
    </contact>

Best Practices
------------

Furniture Placement
^^^^^^^^^^^^^^^^
1. Consider natural heights and positions
2. Allow sufficient interaction space
3. Account for articulation clearance
4. Maintain realistic scales

Contact Configuration
^^^^^^^^^^^^^^^^^^
1. Use appropriate friction for materials
2. Configure collision margins
3. Consider interaction requirements
4. Test stability

Next Steps
---------

- Explore environments in :doc:`scenery`
- Study scene composition in :doc:`examples`
