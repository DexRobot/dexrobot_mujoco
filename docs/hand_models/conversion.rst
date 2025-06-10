======================
URDF to MJCF Conversion
======================

This section covers the process of converting DexHand URDF models to MJCF format for use with MuJoCo.

Conversion Process
----------------

Overview
^^^^^^^
The conversion from URDF to MJCF is handled by the ``convert_hand.py`` script, which performs the following steps:

1. Basic URDF to MJCF conversion using MuJoCo's built-in converter
2. Post-processing to add required elements and configurations
3. Optimization of collision geometries (optional)
4. Addition of actuators and sensors
5. Addition of TaShan sensor components (force bodies and sensor definitions)
6. Configuration of model parameters

Basic Usage
^^^^^^^^^

.. code-block:: bash

    python scripts/convert_hand.py --urdf /path/to/hand.urdf

Advanced Usage
^^^^^^^^^^^^

.. code-block:: bash

    python scripts/convert_hand.py \
        --urdf /path/to/hand.urdf \
        --simplified-collision config/collision_geoms/dexhand021_right_simplified.yaml

TaShan Sensor Integration
^^^^^^^^^^^^^^^^^^^^^^^^^

The conversion script automatically adds TaShan sensor support by:

1. Creating force sensor bodies (``force1_f1`` through ``force5_f7``) on each distal finger link
2. Adding rangefinder sensors (``rf1`` through ``rf5``) for proximity sensing
3. Creating user sensors (``TS-F-A-1`` through ``TS-F-A-5``) for 11-dimensional output

For detailed information about TaShan sensors, see :doc:`/touch_sensors`.

Model Hierarchy
-------------

Input URDF Structure
^^^^^^^^^^^^^^^^^
The URDF model should follow this basic structure:

.. code-block:: xml

    <robot name="dexhand">
        <!-- Base link -->
        <link name="hand_base">
            <!-- Visual and collision geometries -->
        </link>

        <!-- Finger links -->
        <link name="r_f_link1_1">
            <!-- Thumb base -->
        </link>
        <!-- Additional finger links -->

        <!-- Joints -->
        <joint name="r_f_joint1_1" type="revolute">
            <!-- Joint properties -->
        </joint>
        <!-- Additional joints -->
    </robot>

Output MJCF Structure
^^^^^^^^^^^^^^^^^^
The converted MJCF maintains a similar hierarchy with additional elements:

.. code-block:: xml

    <mujoco model="dexhand">
        <!-- Compiler settings -->
        <compiler meshdir="" texturedir=""/>

        <!-- Visual and physics defaults -->
        <default>
            <!-- Default properties -->
        </default>

        <!-- Asset definitions -->
        <asset>
            <!-- Meshes -->
        </asset>

        <!-- Actuator definitions -->
        <actuator>
            <!-- Position actuators -->
        </actuator>

        <!-- Sensor definitions -->
        <sensor>
            <!-- Touch sensors -->
        </sensor>

        <!-- Body hierarchy -->
        <worldbody>
            <body name="hand_base">
                <!-- Base link elements -->
                <body name="r_f_link1_1">
                    <!-- Finger elements -->
                </body>
                <!-- Additional bodies -->
            </body>
        </worldbody>
    </mujoco>

Configuration Files
-----------------

The conversion process uses several configuration files:

Actuator Configuration
^^^^^^^^^^^^^^^^^^^
.. code-block:: yaml

    # Example actuator configuration
    r_f_joint1_1:  # Thumb base
        kp: 20
        kv: 1
        ctrlrange: "0 2.2"
        forcerange: "-20 20"

Collision Configuration
^^^^^^^^^^^^^^^^^^^^
.. code-block:: yaml

    # Example collision geometry configuration
    right_hand_base:
        type: box
        size: [0.0273, 0.05, 0.05]
        pos: [-0.0022, 0, 0.11]

    r_f_link1_2:
        type: capsule
        size: [0.0095]
        fromto: [0.00, 0, 0, 0.03, 0, -0.002]

Implementation Details
-------------------

The conversion is implemented in the ``convert_hand_urdf()`` function:

.. code-block:: python

    def convert_hand_urdf(
        urdf_path=None,
        output_dir=None,
        simplified_collision_yaml=None
    ):
        """Convert URDF to MJCF and add necessary configurations.

        Args:
            urdf_path: Path to input URDF
            output_dir: Output directory for MJCF
            simplified_collision_yaml: Path to collision config
        """
        # Convert URDF to MJCF with enhanced fixed link handling
        urdf2mjcf(urdf_path, output_dir, 
                  fixed_to_body_pattern=r".*(pad|tip).*", 
                  fixed_to_site_pattern=r".*pad.*")

        # Add defaults
        apply_defaults(output_path, defaults_path)

        # Configure actuators
        add_position_actuators(output_path, actuator_config)

        # Add sensors
        add_touch_sensors(output_path, sensor_info)

        # Add base body
        add_trunk_body(output_path, base_name)

        # Update collisions if specified
        if simplified_collision_yaml:
            update_geom_collisions(
                output_path,
                simplified_collision_yaml
            )

Key Functions
^^^^^^^^^^^

urdf2mjcf()
~~~~~~~~~~
Handles URDF to MJCF conversion using MuJoCo's built-in converter with enhanced fixed link handling.

By default, MuJoCo's URDF converter ignores fixed links or converts them to mere geoms.
This function extends the conversion by allowing fixed links to be explicitly converted
to either MuJoCo bodies (with geoms) or MuJoCo sites, based on link name patterns.

add_position_actuators()
~~~~~~~~~~~~~~~~~~~~~
Adds and configures position-controlled actuators for each joint.

add_touch_sensors()
~~~~~~~~~~~~~~~~
Adds touch sensors at specified sites (typically fingertips).

update_geom_collisions()
~~~~~~~~~~~~~~~~~~~~~
Updates collision geometries based on simplified configuration.

Customization
-----------

The conversion process can be customized in several ways:

Custom Actuator Parameters
^^^^^^^^^^^^^^^^^^^^^^^
Modify actuator parameters in the configuration:

.. code-block:: python

    actuator_config = {
        "joint_pattern": {
            "kp": "value",
            "kv": "value",
            "ctrlrange": "min max",
            "forcerange": "min max"
        }
    }

Custom Collision Geometries
^^^^^^^^^^^^^^^^^^^^^^^^
Define custom collision geometries in YAML:

.. code-block:: yaml

    link_name:
        type: box/capsule
        size: [dimensions]
        pos/fromto: [coordinates]

Troubleshooting
-------------

Common Issues
^^^^^^^^^^^

Missing Meshes
~~~~~~~~~~~~
If meshes are not found:

1. Check mesh paths in URDF
2. Verify meshdir setting in MJCF
3. Ensure meshes are in correct directory

Collision Issues
~~~~~~~~~~~~~
If experiencing collision problems:

1. Check collision geometry definitions
2. Verify transform chains
3. Consider using simplified collisions

Next Steps
---------

After converting your model:

- Configure :doc:`collision_models`
- Set up :doc:`actuators`
- Add :doc:`sensors`
- Test with :doc:`examples`
