===================
Topics and Services
===================

This section documents all ROS topics and services provided by the DexRobot MuJoCo node.

Input Topics
----------

joint_commands
^^^^^^^^^^^^
Control joint positions directly.

- **Type**: ``sensor_msgs/JointState``
- **Fields**:
    - ``name``: List of joint names
    - ``position``: List of target positions
- **Example**:

  .. code-block:: python

      from sensor_msgs.msg import JointState

      msg = JointState()
      msg.name = ['r_f_joint1_1', 'r_f_joint1_2']
      msg.position = [0.5, 0.7]

hand_pose
^^^^^^^^
Control 6-DoF floating hand pose.

- **Type**: ``geometry_msgs/Pose``
- **Fields**:
    - ``position``: Position (x, y, z)
    - ``orientation``: Quaternion (w, x, y, z)
- **Notes**:
    - Must specify ``--hand-pose-topic`` to enable
    - Position is scaled by ``position_magnifiers``
- **Example**:

  .. code-block:: python

      from geometry_msgs.msg import Pose

      msg = Pose()
      msg.position.x = 0.5
      msg.position.y = 0.0
      msg.position.z = 1.0
      msg.orientation.w = 1.0
      msg.orientation.x = 0.0
      msg.orientation.y = 0.0
      msg.orientation.z = 0.0

Output Topics
-----------

joint_states
^^^^^^^^^^
Current joint positions and velocities.

- **Type**: ``sensor_msgs/JointState``
- **Fields**:
    - ``name``: List of tracked joint names
    - ``position``: Current joint positions
    - ``velocity``: Current joint velocities
- **Update Rate**: 100Hz (default)

body_poses
^^^^^^^^^
6-DoF poses of tracked bodies.

- **Type**: ``geometry_msgs/PoseArray``
- **Fields**:
    - ``poses``: List of poses for tracked bodies
    - ``header``: Standard header with timestamp
- **Update Rate**: 100Hz (default)

touch_sensors
^^^^^^^^^^^
Touch sensor readings from fingertips.

- **Type**: ``std_msgs/Float32MultiArray``
- **Fields**:
    - ``data``: List of force values
- **Update Rate**: 100Hz (default)
- **Units**: Newtons (N)

Services
-------

save_screenshot
^^^^^^^^^^^^
Save current viewer frame as image.

- **Type**: ``std_srvs/Trigger``
- **Response**:
    - ``success``: True if screenshot saved
    - ``message``: File path or error message
- **Example**:

  .. code-block:: bash

      # ROS2
      ros2 service call /save_screenshot std_srvs/srv/Trigger

      # ROS1
      rosservice call /save_screenshot

Message Formats
-------------

JointState Format
^^^^^^^^^^^^^^^
.. code-block:: yaml

    header:
      stamp: <time>
      frame_id: ''
    name: ['joint1', 'joint2', ...]
    position: [0.1, 0.2, ...]
    velocity: [0.0, 0.0, ...]

Pose Format
^^^^^^^^^^
.. code-block:: yaml

    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      w: 1.0
      x: 0.0
      y: 0.0
      z: 0.0

Touch Sensor Format
^^^^^^^^^^^^^^^^
.. code-block:: yaml

    data: [0.1, 0.2, 0.3, 0.4, 0.5]  # Force values for each sensor

Best Practices
------------

Topic Management
^^^^^^^^^^^^^
1. Check message rates and buffer sizes
2. Handle missing or delayed messages
3. Validate joint names
4. Monitor sensor data ranges

Error Handling
^^^^^^^^^^^^
1. Check joint limits
2. Validate quaternions
3. Handle missing sensors
4. Process stale data

Performance
^^^^^^^^^
1. Use appropriate message queue sizes
2. Monitor topic frequencies
3. Handle high-rate data efficiently
4. Clean up subscribers/publishers

Example Usage
-----------

Python Interface
^^^^^^^^^^^^^

.. code-block:: python

    #!/usr/bin/env python3
    import rospy
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import PoseArray
    from std_msgs.msg import Float32MultiArray
    from std_srvs.srv import Trigger

    class HandController:
        def __init__(self):
            # Publishers
            self.joint_pub = rospy.Publisher(
                'joint_commands',
                JointState,
                queue_size=10
            )

            # Subscribers
            rospy.Subscriber(
                'joint_states',
                JointState,
                self.joint_callback
            )
            rospy.Subscriber(
                'body_poses',
                PoseArray,
                self.pose_callback
            )
            rospy.Subscriber(
                'touch_sensors',
                Float32MultiArray,
                self.touch_callback
            )

        def joint_callback(self, msg):
            # Process joint states
            pass

        def pose_callback(self, msg):
            # Process body poses
            pass

        def touch_callback(self, msg):
            # Process touch data
            pass

        def save_screenshot(self):
            rospy.wait_for_service('save_screenshot')
            try:
                trigger = rospy.ServiceProxy(
                    'save_screenshot',
                    Trigger
                )
                response = trigger()
                return response.success
            except rospy.ServiceException as e:
                print(f"Service call failed: {e}")
                return False

Command Line Interface
^^^^^^^^^^^^^^^^^^^

Monitor topics:

.. code-block:: bash

    # Joint states
    ros2 topic echo /joint_states

    # Body poses
    ros2 topic echo /body_poses

    # Touch sensors
    ros2 topic echo /touch_sensors

Send commands:

.. code-block:: bash

    # Joint command
    ros2 topic pub /joint_commands sensor_msgs/msg/JointState \
        "{name: ['r_f_joint1_1'], position: [0.5]}"

    # Hand pose
    ros2 topic pub /hand_pose geometry_msgs/msg/Pose \
        "{position: {x: 0.5, y: 0.0, z: 1.0},
          orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}"

Next Steps
---------

- Configure :doc:`data_recording`
- Set up :doc:`vr_visualization`
