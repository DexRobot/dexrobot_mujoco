==========
Components
==========

This section covers the actuator and sensor systems in DexRobot MuJoCo hand models.

Actuator System
-------------

The DexHand uses position-controlled actuators with PD control:

.. code-block:: python

    # Bend joints (2-4)
    r"[lr]_f_joint[1-5]_[2-4]": {
        "kp": "20",           # Position gain
        "kv": "0.1",         # Velocity gain
        "ctrlrange": "0 1.3", # Control limits (rad)
        "forcerange": "-20 20" # Force limits (N)
    }

    # Base rotation joints
    r"[lr]_f_joint1_1": {  # Thumb
        "kp": "20",
        "kv": "1",
        "ctrlrange": "0 2.2",
        "forcerange": "-20 20"
    }

Control Parameters
^^^^^^^^^^^^^^^^

- **kp**: Position tracking stiffness (default: 20.0)
- **kv**: Damping behavior (default: 0.1 for bend, 1.0 for base)
- **ctrlrange**: Position limits in radians
- **forcerange**: Force limits in Newtons

Control Interface
^^^^^^^^^^^^^^^

.. code-block:: python

    # ROS control
    joint_msg = JointState()
    joint_msg.name = ['r_f_joint1_1']
    joint_msg.position = [0.5]
    publisher.publish(joint_msg)

    # Direct control
    sim.send_control("act_r_f_joint1_1", 0.5)

Sensor System
-----------

Touch Sensors
^^^^^^^^^^^

MuJoCo native touch sensors are automatically created at fingertip sites:

- Sites created from URDF fixed links with "pad" in name
- Touch sensors attached to these sites
- Real-time force feedback in Newtons

.. code-block:: xml

    <!-- Generated structure -->
    <site name="site_finger_pad" pos="0.025 0.003 0" size="0.01" type="sphere"/>
    <sensor>
        <touch name="touch_finger_pad" site="site_finger_pad"/>
    </sensor>

TaShan Sensors
^^^^^^^^^^^^

When using ``--enable-ts-sensor``, the system provides 11-dimensional tactile data:

- **Force bodies**: ``force1_f1`` through ``force5_f7`` on distal links
- **Rangefinders**: ``rf1`` through ``rf5`` for proximity
- **User sensors**: ``TS-F-A-1`` through ``TS-F-A-5`` for output

See :doc:`/touch_sensors` for detailed TaShan sensor information.

Data Access
^^^^^^^^^

.. code-block:: python

    # ROS topic (Float32MultiArray)
    def touch_callback(msg):
        sensor_values = msg.data  # Force values in Newtons
        
    rospy.Subscriber("touch_sensors", Float32MultiArray, touch_callback)

    # Direct API
    sensor_data = sim.data.sensor("touch_r_f_link1_4").data

Customization
-----------

Custom Actuators
^^^^^^^^^^^^^^

.. code-block:: python

    actuator_config = {
        "joint_pattern": {
            "kp": "30",
            "kv": "0.2",
            "ctrlrange": "0 1.5",
            "forcerange": "-25 25"
        }
    }
    add_position_actuators(xml_path, actuator_config)

Custom Sensors
^^^^^^^^^^^^

.. code-block:: python

    custom_sites = {
        "link_name": {
            "pos": "x y z",
            "size": "radius",
            "type": "sphere"
        }
    }
    add_sites(xml_path, custom_sites)
    add_touch_sensors(xml_path, sensor_info)

Next Steps
---------

- Configure :doc:`collision_models`
- Review :doc:`conversion` process
- Test with :doc:`examples`