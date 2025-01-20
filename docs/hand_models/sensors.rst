=======
Sensors
=======

This section covers the sensor system in DexRobot MuJoCo, focusing on touch sensors and their configuration.

Sensor System Overview
--------------------

The DexHand includes touch sensors at fingertips:

- One touch sensor per fingertip
- Based on MuJoCo's built-in contact sensing
- Configurable sensor sites
- Real-time force feedback

Components
---------

Sensor Sites
^^^^^^^^^^
Visual and physical locations for sensors:

.. code-block:: python

    sensor_sites = {
        "r_f_link1_4": {  # Thumb tip
            "pos": "0.025 0.003 0",
            "size": "0.01",
            "type": "sphere"
        },
        # Additional fingertip sites...
    }

Touch Sensors
^^^^^^^^^^^
Contact force sensors at each site:

.. code-block:: python

    sensor_info = {
        "touch_r_f_link1_4": {  # Thumb sensor
            "site": "site_r_f_link1_4"
        },
        # Additional touch sensors...
    }

Implementation
------------

Adding Sites
^^^^^^^^^^
Sites are added using the ``add_sites()`` function:

.. code-block:: python

    def add_sites(xml_file_path, site_info):
        """Add sites to specific bodies.

        Args:
            xml_file_path: Path to MJCF XML file
            site_info: Dictionary of site configurations
        """
        tree = ET.parse(xml_file_path)
        root = tree.getroot()

        # Add sites to specified bodies
        for body_name, details in site_info.items():
            body = find_body(root, body_name)
            if body is not None:
                add_site_to_body(body, body_name, details)

MJCF Structure
^^^^^^^^^^^^
Generated site and sensor elements:

.. code-block:: xml

    <!-- Site definition -->
    <site name="site_r_f_link1_4"
          pos="0.025 0.003 0"
          size="0.01"
          type="sphere"/>

    <!-- Sensor definition -->
    <sensor>
        <touch name="touch_r_f_link1_4"
               site="site_r_f_link1_4"/>
    </sensor>

Site Parameters
-------------

Position (pos)
^^^^^^^^^^^^
- Specified relative to link frame
- Format: "x y z"
- Typical fingertip offset

Size
^^^^
- Sphere radius for site visualization
- Also affects contact detection area
- Default: 0.01 (1cm)

Type
^^^^
- Visual representation type
- Usually "sphere" for fingertips
- Other options: "box", "cylinder"

Usage
-----

Reading Sensor Data
^^^^^^^^^^^^^^^^

Via ROS Topics:

.. code-block:: python

    def touch_callback(msg):
        """Handle touch sensor data.

        Args:
            msg (Float32MultiArray): Touch sensor values
        """
        sensor_values = msg.data
        # Process sensor data...

    # Subscribe to touch sensor topic
    rospy.Subscriber(
        "touch_sensors",
        Float32MultiArray,
        touch_callback
    )

Via MuJoCo API:

.. code-block:: python

    # Using MJControlWrapper
    sensor_data = sim.data.sensor("touch_r_f_link1_4").data

Data Format
^^^^^^^^^
- Each sensor returns contact force magnitude
- Values ≥ 0 (no negative forces)
- Units: Newtons
- Update rate: Simulation timestep

Configuration
-----------

Default Setup
^^^^^^^^^^^
The default configuration adds sites and sensors to all fingertips:

.. code-block:: python

    # Fingertip site pattern
    fingertip_names = [
        f"r_f_link{i}_4" for i in range(1, 6)
    ]

    # Generate configurations
    sensor_sites = {
        name: {
            "pos": "0.025 0.003 0",
            "size": "0.01",
            "type": "sphere"
        }
        for name in fingertip_names
    }

Custom Configuration
^^^^^^^^^^^^^^^^^
Add custom sensor sites:

.. code-block:: python

    # Custom site configuration
    custom_sites = {
        "link_name": {
            "pos": "x y z",
            "size": "radius",
            "type": "shape"
        }
    }

    # Add custom sites
    add_sites(xml_path, custom_sites)
    add_touch_sensors(xml_path, sensor_info)

Advanced Usage
------------

Contact Parameters
^^^^^^^^^^^^^^^
Adjust contact properties in MJCF:

.. code-block:: xml

    <default>
        <site friction="1 0.005 0.0001"
              solimp="0.9 0.95 0.001"
              solref="0.02 1"/>
    </default>

Filtering and Processing
^^^^^^^^^^^^^^^^^^^^
Example sensor data processing:

.. code-block:: python

    class TouchProcessor:
        def __init__(self):
            self.history = []
            self.threshold = 0.1

        def process(self, sensor_data):
            # Apply threshold
            filtered = [
                f if f > self.threshold else 0
                for f in sensor_data
            ]

            # Store history
            self.history.append(filtered)

            # Calculate features
            mean_force = np.mean(filtered)
            max_force = np.max(filtered)
            active_sensors = sum(f > 0 for f in filtered)

            return {
                'mean': mean_force,
                'max': max_force,
                'active': active_sensors
            }

Troubleshooting
-------------

Common Issues
^^^^^^^^^^^

No Sensor Readings
~~~~~~~~~~~~~~~
If not getting sensor data:

1. Check site placement
2. Verify sensor configuration
3. Confirm contact parameters
4. Check collision settings

Noisy Readings
~~~~~~~~~~~~
If experiencing noisy data:

1. Adjust contact parameters
2. Implement filtering
3. Check collision properties
4. Verify site size and placement

Next Steps
---------

After setting up sensors:

- Test with :doc:`examples`
- Review :doc:`/ros_integration/index` for ROS usage
