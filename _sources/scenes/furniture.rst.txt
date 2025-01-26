===================
Furniture Components
===================

Component types and usage in scene composition.

Tables
-----

Structure: ``furniture_sim/[type]/[type]_{asset,body}.xml``

.. code-block:: xml

    <!-- Assets and materials -->
    <include file="furniture_sim/simpleTable/simpleTable_asset.xml"/>

    <!-- Physical structure (pick one): -->
    <include file="furniture_sim/simpleTable/simpleGraniteTable_body.xml"/>
    <include file="furniture_sim/simpleTable/simpleMarbleTable_body.xml"/>
    <include file="furniture_sim/simpleTable/simpleWoodTable_body.xml"/>

Available Types:

- simpleTable: Basic surface with material variants
- studyTable: Surface with drawer/handle mechanics
- ventionTable: Industrial frame structure

Storage Units
----------

Structure: ``furniture_sim/[type]/[type]_{asset,body}.xml``

.. code-block:: xml

    <!-- Cabinet with hinged door -->
    <include file="furniture_sim/hingecabinet/hingecabinet_asset.xml"/>
    <include file="furniture_sim/hingecabinet/hingecabinet_body.xml"/>

    <!-- Cabinet with sliding door -->
    <include file="furniture_sim/slidecabinet/slidecabinet_asset.xml"/>
    <include file="furniture_sim/slidecabinet/slidecabinet_body.xml"/>

Available Types:

- hingecabinet: Hinged door mechanics
- slidecabinet: Linear sliding mechanism
- counters: Kitchen counter units
- bin: Storage containers

Appliances
--------

Structure: ``furniture_sim/[type]/[type]_[index].xml`` for variants

.. code-block:: xml

    <!-- Microwave with version selection -->
    <include file="furniture_sim/microwave/microwave_asset.xml"/>
    <include file="furniture_sim/microwave/microwave_body0.xml"/> <!-- Versions 0-3 -->

    <!-- Kettle with version selection -->
    <include file="furniture_sim/kettle/kettle_asset.xml"/>
    <include file="furniture_sim/kettle/kettle_body0.xml"/> <!-- Versions 0-7 -->

Available Types:

- microwave: Door and control panel mechanics
- kettle: Handle and pour mechanics
- oven: Door, knob and burner mechanics

Materials and Textures
-------------------

Standard textures in ``common/textures/``:

.. code-block:: xml

    <material name="surface"
             texture="wood2"
             rgba="1 1 1 1"/>

Categories:

- wood[0-4].png: Wood textures
- metal[0-4].png: Metal textures
- stone[0-4].png: Stone textures

Contact Properties
---------------

Standard contact parameters:

.. code-block:: xml

    <contact>
        <pair geom1="hand" geom2="furniture" friction="1 0.005 0.0001"/>
    </contact>
