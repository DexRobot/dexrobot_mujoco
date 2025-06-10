==================
Touch Sensors
==================

DexRobot MuJoCo provides two types of tactile sensing: MuJoCo's native touch sensors and TaShan (他山) 11-dimensional sensors.

Sensor Types
============

MuJoCo Touch Sensors
--------------------

Simple force magnitude per fingertip (1 value per sensor).

**Usage:**

.. code-block:: bash

    python nodes/dexrobot_mujoco_ros.py dexrobot_mujoco/scenes/box.xml

**Data format:** 5 float values in ``touch_sensors`` topic


TaShan Sensors
--------------

11-dimensional tactile feedback with force vectors and contact geometry.

**Requirements:**
- Python 3.8
- MuJoCo 3.2.3
- Install with: ``pip install -e .[tashan]``

**Usage:**

.. code-block:: bash

    python nodes/dexrobot_mujoco_ros.py dexrobot_mujoco/scenes/box.xml --enable-ts-sensor

**Data format:** 
- Raw sensor data (11 values per sensor) in ``touch_sensors`` topic  
- Semantic force data in ``ts_forces`` topic as 2D array: ``[sensor_id, normal_force, tangential_force, direction_degrees]``

How TaShan Sensors Work
=======================

Sensor Naming Convention
------------------------

The TaShan callback requires specific sensor names in your model:

1. **Rangefinder sensors** (``rf1`` - ``rf5``): Input sensors for proximity data
2. **User sensors** (``TS-F-A-1`` - ``TS-F-A-5``): Output sensors for tactile data
3. **Force bodies** (``force1_f1`` - ``force5_f7``): Visual components for force distribution

Example sensor configuration:

.. code-block:: xml

    <sensor>
        <!-- Touch sensors for basic contact -->
        <touch name="touch_r_f_link1_pad" site="site_r_f_link1_pad"/>
        <touch name="touch_r_f_link2_pad" site="site_r_f_link2_pad"/>
        
        <!-- Rangefinder sensors (required by TaShan) -->
        <rangefinder name="rf1" site="site_r_f_link1_4" cutoff="0.1"/>
        <rangefinder name="rf2" site="site_r_f_link2_4" cutoff="0.1"/>
        
        <!-- TaShan user sensors (11-dimensional output) -->
        <user name="TS-F-A-1" dim="11" noise="0.0"/>
        <user name="TS-F-A-2" dim="11" noise="0.0"/>
    </sensor>

Callback Registration
---------------------

TaShan sensors work by registering a MuJoCo sensor callback:

.. code-block:: python

    from dexrobot_mujoco.wrapper import TSSensorManager
    
    # Initialize callback before model load
    TSSensorManager.initialize_before_model_load()
    
    # Create wrapper and manager
    wrapper = MJSimWrapper("model.xml")
    ts_manager = TSSensorManager(wrapper)
    wrapper.set_sensor_manager(ts_manager)

Data Access
-----------

Access sensor data programmatically:

.. code-block:: python

    # Get force data for specific sensor
    force = ts_manager.get_force_data_by_id(1)  # Thumb
    print(f"Normal: {force.normal} N")
    print(f"Tangential: {force.tangential_magnitude} N") 
    print(f"Direction: {force.tangential_direction} degrees")
    
    # Get all sensor forces
    all_forces = ts_manager.get_all_force_data()
    for sensor_id, force in all_forces.items():
        print(f"Sensor {sensor_id}: {force.normal:.2f} N normal")

Model Conversion
================

The conversion script automatically generates required sensors:

.. code-block:: bash

    # Convert URDF with TaShan sensors
    python scripts/convert_hand.py path/to/hand.urdf

This creates:
- Touch sensors on finger pads
- Rangefinder sensors on fingertips
- TaShan user sensors for output
- Force visualization bodies

Standalone Demo
===============

Test sensors without ROS:

.. code-block:: bash

    # Requires rerun-sdk==0.18.2
    python examples/tashan_standalone_demo.py

The demo shows:
- Real-time force visualization
- Pinch gesture with box interaction  
- Force vectors for thumb and index finger

Troubleshooting
===============

**Zero sensor readings:**
- Ensure Python 3.8 and MuJoCo 3.2.3 are installed
- Check sensor names match expected pattern
- Verify ``TSensor.so`` library is loaded

**Import errors:**
- TaShan requires specific versions due to binary compatibility
- Use conda environment as described in :doc:`tashan_python38_setup`

See Also
--------

- :doc:`wrapper/index` - MuJoCo control wrapper
- :doc:`hand_models/sensors` - Sensor placement details
- :doc:`hand_models/conversion` - Model conversion with sensors
- :doc:`ros_integration/topics_and_services` - ROS sensor topics
- :doc:`getting-started` - Installation and basic usage