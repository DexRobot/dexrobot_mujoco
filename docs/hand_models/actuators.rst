==========
Actuators
==========

This section covers the actuator system in DexRobot MuJoCo, including actuator types, configuration, and control.

Actuator System
-------------

Overview
^^^^^^^
The DexHand uses position-controlled actuators:

- One actuator per joint
- PD control with configurable gains
- Adjustable control and force ranges
- Velocity damping for stability

Default Configuration
------------------

Joint Types
^^^^^^^^^

Bend Joints
~~~~~~~~~
Configuration for finger bending joints (joints 2-4):

.. code-block:: python

    r"[lr]_f_joint[1-5]_[2-4]": {
        "kp": "20",           # Position gain
        "kv": "0.1",         # Velocity gain
        "ctrlrange": "0 1.3", # Control limits (rad)
        "forcerange": "-20 20" # Force limits (N)
    }

Base Rotation Joints
~~~~~~~~~~~~~~~~~
Configuration for finger base joints (joint 1):

.. code-block:: python

    # Thumb
    r"[lr]_f_joint1_1": {
        "kp": "20",
        "kv": "1",
        "ctrlrange": "0 2.2",
        "forcerange": "-20 20"
    }

    # Index finger
    r"[lr]_f_joint2_1": {
        "kp": "20",
        "kv": "1",
        "ctrlrange": "0 0.3",
        "forcerange": "-20 20"
    }

    # Additional fingers...

Configuration Parameters
---------------------

Position Gain (kp)
^^^^^^^^^^^^^^^^
- Controls position tracking stiffness
- Higher values: Faster response, potential instability
- Lower values: Smoother motion, less precise tracking
- Default: 20.0

Velocity Gain (kv)
^^^^^^^^^^^^^^^
- Controls damping behavior
- Higher values: More damping, slower motion
- Lower values: Less damping, potential oscillation
- Default: 0.1 (bend joints), 1.0 (base joints)

Control Range
^^^^^^^^^^^
- Limits the commanded position range
- Specified in radians
- Format: "min max"
- Different ranges for each joint type

Force Range
^^^^^^^^^
- Limits the maximum force output
- Specified in Newtons
- Format: "min max"
- Default: "-20 20" for all joints

Implementation
------------

Adding Actuators
^^^^^^^^^^^^^
Actuators are added using the ``add_position_actuators()`` function:

.. code-block:: python

    def add_position_actuators(xml_path, actuator_info):
        """Add position actuators to MJCF XML file.

        Args:
            xml_path: Path to MJCF XML file
            actuator_info: Dictionary of actuator configs
        """
        tree = ET.parse(xml_path)
        root = tree.getroot()

        # Find/create actuator element
        actuator_element = ensure_actuator_element(root)

        # Add actuators for matching joints
        for joint_pattern, props in actuator_info.items():
            add_matching_actuators(
                actuator_element,
                joint_pattern,
                props
            )

MJCF Structure
^^^^^^^^^^^^
Generated actuator elements in MJCF:

.. code-block:: xml

    <actuator>
        <position name="act_r_f_joint1_1"
                 joint="r_f_joint1_1"
                 kp="20"
                 kv="1"
                 ctrlrange="0 2.2"
                 forcerange="-20 20"/>
        <!-- Additional actuators -->
    </actuator>

Usage
-----

Basic Control
^^^^^^^^^^^
Send control signals through ROS:

.. code-block:: python

    # Using ROS messages
    joint_msg = JointState()
    joint_msg.name = ['r_f_joint1_1']
    joint_msg.position = [0.5]
    publisher.publish(joint_msg)

Direct Control
^^^^^^^^^^^^
Control using the MuJoCo wrapper:

.. code-block:: python

    # Using MJControlWrapper
    sim.send_control("act_r_f_joint1_1", 0.5)
    sim.step()

Custom Configuration
-----------------

Modifying Parameters
^^^^^^^^^^^^^^^^^
Create custom actuator configurations:

.. code-block:: python

    actuator_config = {
        "joint_pattern": {
            "kp": "custom_value",
            "kv": "custom_value",
            "ctrlrange": "min max",
            "forcerange": "min max"
        }
    }

    add_position_actuators(xml_path, actuator_config)

Infinite Rotation
^^^^^^^^^^^^^^
Enable infinite rotation for specific joints:

.. code-block:: python

    # Using MJControlWrapper
    sim.enable_infinite_rotation(r"joint_name_pattern")

Troubleshooting
-------------

Common Issues
^^^^^^^^^^^

Unstable Control
~~~~~~~~~~~~~
If experiencing instability:

1. Reduce position gain (kp)
2. Increase velocity gain (kv)
3. Check force limits
4. Verify control ranges

Slow Response
~~~~~~~~~~~
If response is too slow:

1. Increase position gain (kp)
2. Decrease velocity gain (kv)
3. Check force limits
4. Verify control ranges

Next Steps
---------

After configuring actuators:

- Add :doc:`sensors`
- Test with :doc:`examples`
- Review :doc:`/advanced/performance` for optimization
