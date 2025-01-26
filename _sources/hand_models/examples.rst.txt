===================
Hand Model Examples
===================

This section provides practical examples of working with DexHand models in various scenarios.

Basic Examples
------------

Launch Single Hand
^^^^^^^^^^^^^^^^

Simple visualization of a right hand:

.. code-block:: bash

    python nodes/dexrobot_mujoco_ros.py \
        dexrobot_mujoco/models/dexhand021_right_simplified.xml

Joint Control Example
^^^^^^^^^^^^^^^^^^

Control individual joints using ROS messages:

.. code-block:: python

    #!/usr/bin/env python3
    import rospy
    from sensor_msgs.msg import JointState

    def main():
        rospy.init_node('hand_control_example')
        pub = rospy.Publisher('joint_commands', JointState, queue_size=10)

        # Create joint command
        msg = JointState()
        msg.name = ['r_f_joint1_1', 'r_f_joint1_2', 'r_f_joint1_3']
        msg.position = [0.5, 0.7, 0.9]

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            msg.header.stamp = rospy.Time.now()
            pub.publish(msg)
            rate.sleep()

    if __name__ == '__main__':
        main()

Read Touch Sensors
^^^^^^^^^^^^^^^

Monitor touch sensor values:

.. code-block:: python

    #!/usr/bin/env python3
    import rospy
    from std_msgs.msg import Float32MultiArray

    def touch_callback(msg):
        # Print sensor values
        for i, force in enumerate(msg.data):
            if force > 0.1:  # Threshold for noise
                print(f"Sensor {i}: {force:.2f}N")

    def main():
        rospy.init_node('touch_monitor_example')
        rospy.Subscriber(
            'touch_sensors',
            Float32MultiArray,
            touch_callback
        )
        rospy.spin()

    if __name__ == '__main__':
        main()

Advanced Examples
---------------

Floating Hand with VR
^^^^^^^^^^^^^^^^^^^

Launch hand with VR visualization:

.. code-block:: bash

    python nodes/dexrobot_mujoco_ros.py \
        dexrobot_mujoco/models/dexhand021_right_floating.xml \
        --enable-vr \
        --hand-pose-topic hand_pose \
        --position-magnifiers 2.5,2.0,0.8

Hand pose control code:

.. code-block:: python

    #!/usr/bin/env python3
    import rospy
    from geometry_msgs.msg import Pose
    import numpy as np

    def main():
        rospy.init_node('hand_pose_example')
        pub = rospy.Publisher('hand_pose', Pose, queue_size=10)

        # Create pose message
        msg = Pose()

        # Circular motion parameters
        radius = 0.3
        height = 1.0
        freq = 0.5

        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            # Calculate position
            t = rospy.Time.now().to_sec()
            angle = 2 * np.pi * freq * t

            msg.position.x = radius * np.cos(angle)
            msg.position.y = radius * np.sin(angle)
            msg.position.z = height

            # Fixed orientation (palm down)
            msg.orientation.w = 0.707
            msg.orientation.x = 0.707
            msg.orientation.y = 0
            msg.orientation.z = 0

            pub.publish(msg)
            rate.sleep()

    if __name__ == '__main__':
        main()

Robot Arm Integration
^^^^^^^^^^^^^^^^^^

Launch hand mounted on JAKA Zu7:

.. code-block:: bash

    python nodes/dexrobot_mujoco_ros.py \
        dexrobot_mujoco/models/dexhand021_right_jaka_zu7.xml \
        --config config/scene_default.yaml

Joint control with configuration:

.. code-block:: yaml

    # config/arm_control.yaml

    # Camera settings
    camera:
      azimuth: -90
      distance: 1.5
      elevation: -20
      lookat: [0.0, 0.0, 0.8]

    # Track these joints
    tracked_joints:
    - [joint1, joint2, joint3, joint4, joint5, joint6]  # Arm joints
    - [r_f_joint1_1, r_f_joint1_2, r_f_joint1_3, r_f_joint1_4]  # Thumb
    - [r_f_joint2_1, r_f_joint2_2, r_f_joint2_3, r_f_joint2_4]  # Index
    # ... other fingers ...

    # Initial joint positions
    initial_qpos:
      joint1: 0.0
      joint2: -0.5
      joint3: 0.0
      joint4: -1.57
      joint5: 0.0
      joint6: 0.0

Combined arm and hand control:

.. code-block:: python

    #!/usr/bin/env python3
    import rospy
    from sensor_msgs.msg import JointState
    import numpy as np

    class ArmHandController:
        def __init__(self):
            rospy.init_node('arm_hand_controller')
            self.pub = rospy.Publisher(
                'joint_commands',
                JointState,
                queue_size=10
            )

            # Joint names
            self.arm_joints = [
                'joint1', 'joint2', 'joint3',
                'joint4', 'joint5', 'joint6'
            ]
            self.hand_joints = [
                'r_f_joint1_1', 'r_f_joint1_2',
                'r_f_joint1_3', 'r_f_joint1_4'
            ]

        def move_to_pose(self, arm_pos, hand_pos):
            msg = JointState()
            msg.name = self.arm_joints + self.hand_joints
            msg.position = arm_pos + hand_pos
            msg.header.stamp = rospy.Time.now()
            self.pub.publish(msg)

    def main():
        controller = ArmHandController()
        rate = rospy.Rate(10)

        # Example motion sequence
        while not rospy.is_shutdown():
            # Pre-grasp pose
            controller.move_to_pose(
                arm_pos=[0, -0.5, 0, -1.57, 0, 0],
                hand_pos=[0, 0, 0, 0]
            )
            rospy.sleep(2.0)

            # Grasp pose
            controller.move_to_pose(
                arm_pos=[0, -0.5, 0, -1.57, 0, 0],
                hand_pos=[0.5, 0.7, 0.7, 0.7]
            )
            rospy.sleep(2.0)

    if __name__ == '__main__':
        main()

Recording Examples
---------------

Record All Data
^^^^^^^^^^^^

Record simulation to multiple formats:

.. code-block:: bash

    python nodes/dexrobot_mujoco_ros.py \
        dexrobot_mujoco/models/dexhand021_right_jaka_zu7.xml \
        --config config/scene_default.yaml \
        --output-formats ros csv mp4 \
        --output-csv-path data.csv \
        --output-mp4-path video.mp4 \
        --output-bag-path recording.bag

Data processing example:

.. code-block:: python

    #!/usr/bin/env python3
    import pandas as pd
    import numpy as np
    import matplotlib.pyplot as plt

    def analyze_recording(csv_path):
        # Load data
        data = pd.read_csv(csv_path)

        # Plot joint positions
        plt.figure(figsize=(10, 6))
        for joint in ['r_f_joint1_1', 'r_f_joint1_2',
                     'r_f_joint1_3', 'r_f_joint1_4']:
            plt.plot(
                data['timestamp'],
                data[f'{joint}_pos'],
                label=joint
            )

        plt.xlabel('Time (s)')
        plt.ylabel('Position (rad)')
        plt.title('Joint Positions')
        plt.legend()
        plt.grid(True)
        plt.show()

        # Analyze touch sensors
        touch_data = data[[col for col in data.columns
                          if 'touch' in col]]

        print("Touch Statistics:")
        print(f"Max force: {touch_data.max().max():.2f}N")
        print(f"Mean force: {touch_data.mean().mean():.2f}N")

    if __name__ == '__main__':
        analyze_recording('data.csv')

Custom Configurations
------------------

Scene with Hand Example
^^^^^^^^^^^^^^^^^^^^

Create a scene with hand and objects:

.. code-block:: python

    from dexrobot_mujoco.utils.mjcf_utils import merge_xml_files

    def create_scene():
        # Define models and their poses
        xml_dict = {
            'models/dexhand021_right_floating.xml': {
                'pos': '0 0 1.0',
                'quat': '0.707 0.707 0 0',
                'articulation_method': 'free'
            },
            'scenes/table.xml': {
                'pos': '0 0 0',
                'quat': '1 0 0 0',
                'articulation_method': 'fixed'
            },
            'scenes/objects/sphere.xml': {
                'pos': '0 0 1.1',
                'quat': '1 0 0 0',
                'articulation_method': 'free'
            }
        }

        # Merge models into scene
        merge_xml_files(
            xml_dict,
            'output_scene.xml',
            'manipulation_scene'
        )

    if __name__ == '__main__':
        create_scene()

Launch the custom scene:

.. code-block:: bash

    python nodes/dexrobot_mujoco_ros.py \
        output_scene.xml \
        --config config/scene_default.yaml

Next Steps
---------

After trying these examples:

- Create custom scenes in :doc:`/scenes/index`
- Explore ROS integration in :doc:`/ros_integration/index`
