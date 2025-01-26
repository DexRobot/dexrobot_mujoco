=============
Articulation
=============

This section covers the process of articulating DexHand models with other mechanical systems, such as robot arms or floating bases.

Overview
-------

The articulation system allows:

- Attaching hands to robot arms
- Creating floating base configurations
- Preserving visual and collision properties
- Configuring joint limits and dynamics

Basic Concepts
------------

Articulation Methods
^^^^^^^^^^^^^^^^^
Two main methods of articulation:

1. **Fixed Mount**
   - Rigid attachment to another model
   - Suitable for robot arms
   - Preserves kinematic chain

2. **Floating Base**
   - 6-DoF free joint
   - Suitable for teleoperation
   - Independent position/orientation control

Model Components
^^^^^^^^^^^^^
Each articulated model requires:

- Parent model (arm/base)
- Child model (hand)
- Attachment specifications
- Configuration parameters

Usage
-----

Basic Articulation
^^^^^^^^^^^^^^^
Using the articulation script:

.. code-block:: bash

    python scripts/articulate_hand.py \
        --base robot_arm.xml \
        --hand dexhand021_right.xml \
        --output combined_model.xml \
        --euler 0 90 0

Parameters:
^^^^^^^^^^^
- ``--base``: Parent model XML path
- ``--hand``: Hand model XML path
- ``--output``: Output model path
- ``--pos``: Position offset [x y z]
- ``--euler``: Euler angles [roll pitch yaw]
- ``--quat``: Quaternion [w x y z]
- ``--add-ground``: Add ground plane

Example: Robot Arm
^^^^^^^^^^^^^^^^
Attaching to a JAKA Zu7 arm:

.. code-block:: bash

    python scripts/articulate_hand.py \
        --base models/jaka_zu7_right.xml \
        --hand models/dexhand021_right_simplified.xml \
        --output models/dexhand021_right_jaka_zu7.xml \
        --euler 0 0 0

Example: Floating Base
^^^^^^^^^^^^^^^^^^^
Creating a floating base configuration:

.. code-block:: bash

    python scripts/articulate_hand.py \
        --base models/floating_base.xml \
        --hand models/dexhand021_right.xml \
        --output models/dexhand021_right_floating.xml \
        --euler 0 90 0

Implementation
------------

The ``articulate()`` Function
^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

    def articulate(
        parent_xml_path,
        child_xml_path,
        link_name,
        output_xml_path,
        pos="0 0 0",
        quat="1 0 0 0"
    ):
        """Articulate child model to parent.

        Args:
            parent_xml_path: Parent model XML
            child_xml_path: Child model XML
            link_name: Attachment link name
            output_xml_path: Output path
            pos: Position offset
            quat: Orientation quaternion
        """
        # Core steps:
        # 1. Load models
        # 2. Merge configurations
        # 3. Update hierarchies
        # 4. Configure attachment
        # 5. Save combined model

MJCF Structure
^^^^^^^^^^^^
Example combined model structure:

.. code-block:: xml

    <mujoco>
        <!-- Combined model -->
        <compiler meshdir=""/>

        <!-- Merged assets -->
        <asset>
            <!-- Parent assets -->
            <!-- Child assets -->
        </asset>

        <!-- Body hierarchy -->
        <worldbody>
            <!-- Parent base -->
            <body name="parent_base">
                <!-- Parent links -->
                <body name="attachment_link">
                    <!-- Child base -->
                    <body name="hand_base"
                          pos="0 0 0"
                          quat="1 0 0 0">
                        <!-- Hand model -->
                    </body>
                </body>
            </body>
        </worldbody>

        <!-- Combined actuators -->
        <actuator>
            <!-- Parent actuators -->
            <!-- Child actuators -->
        </actuator>
    </mujoco>

Advanced Features
--------------

Custom Attachment Points
^^^^^^^^^^^^^^^^^^^^
Specify custom attachment configurations:

.. code-block:: python

    # Position offset
    pos = "0.1 0 0.05"  # x y z

    # Orientation (quaternion)
    quat = "0.707 0 0.707 0"  # w x y z

Mesh Management
^^^^^^^^^^^^
Handle mesh file paths:

.. code-block:: python

    # Update mesh directory
    compiler = ET.SubElement(root, "compiler")
    compiler.set("meshdir", "path/to/meshes")

Asset Merging
^^^^^^^^^^^
Combine model assets:

.. code-block:: python

    def merge_assets(parent_root, child_root):
        """Merge assets from both models."""
        # Get/create asset elements
        parent_asset = get_asset_element(parent_root)
        child_asset = child_root.find("asset")

        # Copy child assets
        if child_asset is not None:
            for asset in child_asset:
                parent_asset.append(asset)

Common Use Cases
-------------

Robot Arm Integration
^^^^^^^^^^^^^^^^^^
1. Convert URDF to MJCF
2. Configure attachment point
3. Articulate models
4. Test kinematics
5. Verify control

Floating Manipulation
^^^^^^^^^^^^^^^^^^
1. Start with floating base
2. Configure 6-DoF control
3. Add pose tracking
4. Implement teleoperation

Troubleshooting
-------------

Common Issues
^^^^^^^^^^^

Incorrect Placement
~~~~~~~~~~~~~~~~
If hand position is wrong:

1. Check attachment point
2. Verify coordinate systems
3. Adjust offset values
4. Review transformation chain

Control Problems
^^^^^^^^^^^^^
If experiencing control issues:

1. Check actuator configurations
2. Verify joint limits
3. Review control parameters
4. Test individual components

Next Steps
---------

After articulation:

- Configure control in :doc:`/ros_integration/index`
- Test with :doc:`examples`
