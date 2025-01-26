=============
Data Recording
=============

This section covers the data recording capabilities of the DexRobot MuJoCo node.

Recording Options
--------------

The node supports three recording formats:

1. CSV files (tracked states and sensors)
2. ROS bags (all ROS topics)
3. MP4 videos (simulation visualization)

These can be used individually or simultaneously.

CSV Recording
-----------

Configuration
^^^^^^^^^^^

Enable CSV recording:

.. code-block:: bash

    python nodes/dexrobot_mujoco_ros.py model.xml \
        --output-formats csv \
        --output-csv-path data.csv

Data Format
^^^^^^^^^
CSV columns include:

- ``timestamp``: Nanosecond timestamp
- ``{joint_name}_pos``: Joint positions
- ``{joint_name}_vel``: Joint velocities
- ``{body_name}_pos``: Body positions (x, y, z)
- ``{body_name}_quat``: Body orientations (w, x, y, z)
- ``{sensor_name}``: Sensor readings

Example CSV:

.. code-block:: text

    timestamp,r_f_joint1_1_pos,r_f_joint1_1_vel,...
    1234567890,0.5,0.1,...
    1234567900,0.51,0.1,...

ROS Bag Recording
--------------

Configuration
^^^^^^^^^^^

Enable bag recording:

.. code-block:: bash

    python nodes/dexrobot_mujoco_ros.py model.xml \
        --output-formats ros \
        --output-bag-path recording.bag \
        --additional-bag-topics /camera/image_raw

Recorded Topics
^^^^^^^^^^^^
Default topics:

- ``/joint_commands``
- ``/joint_states``
- ``/body_poses``
- ``/touch_sensors``

Additional topics can be specified with ``--additional-bag-topics``.

MP4 Recording
-----------

Configuration
^^^^^^^^^^^

Enable MP4 recording:

.. code-block:: bash

    python nodes/dexrobot_mujoco_ros.py model.xml \
        --output-formats mp4 \
        --output-mp4-path video.mp4 \
        --renderer-dimension 640,480

Parameters:

- Frame rate: 20 FPS (fixed)
- Codec: mp4v
- Resolution: Specified by ``renderer-dimension``

Combined Recording
---------------

Enable all formats:

.. code-block:: bash

    python nodes/dexrobot_mujoco_ros.py model.xml \
        --output-formats ros csv mp4 \
        --output-csv-path data.csv \
        --output-bag-path recording.bag \
        --output-mp4-path video.mp4

Data Processing
-------------

CSV Processing
^^^^^^^^^^^^

Example Python script:

.. code-block:: python

    import pandas as pd
    import numpy as np

    def analyze_data(csv_path):
        # Load data
        data = pd.read_csv(csv_path)

        # Process joint positions
        joint_cols = [col for col in data.columns
                     if col.endswith('_pos')]
        joint_data = data[joint_cols]

        # Calculate statistics
        stats = {
            'mean': joint_data.mean(),
            'std': joint_data.std(),
            'max': joint_data.max(),
            'min': joint_data.min()
        }

        # Extract touch data
        touch_cols = [col for col in data.columns
                     if 'touch' in col]
        touch_data = data[touch_cols]

        return stats, touch_data

ROS Bag Processing
^^^^^^^^^^^^^^^

Example Python script:

.. code-block:: python

    import rosbag
    from sensor_msgs.msg import JointState

    def process_bag(bag_path):
        joint_data = []

        with rosbag.Bag(bag_path) as bag:
            # Process joint states
            for topic, msg, t in bag.read_messages(
                topics=['joint_states']
            ):
                joint_data.append({
                    'time': t.to_sec(),
                    'positions': msg.position,
                    'velocities': msg.velocity
                })

        return joint_data

Best Practices
------------

File Management
^^^^^^^^^^^^
1. Use absolute paths
2. Create output directories
3. Check disk space
4. Use descriptive filenames

Performance
^^^^^^^^^
1. Monitor memory usage
2. Check recording rates
3. Adjust buffer sizes
4. Balance quality settings

Data Organization
^^^^^^^^^^^^^^
1. Use consistent naming
2. Document data format
3. Include metadata
4. Maintain timestamps

Error Handling
^^^^^^^^^^^^
1. Check file permissions
2. Monitor disk space
3. Handle recording failures
4. Clean up resources

Example Pipeline
-------------

Complete Recording Setup
^^^^^^^^^^^^^^^^^^^^

1. Create configuration:

   .. code-block:: yaml

       # config/recording.yaml
       camera:
         distance: 1.5
         elevation: -20
         azimuth: 0
         lookat: [0.0, 0.0, 1.0]

       tracked_joints:
         - [r_f_joint1_1, r_f_joint1_2]

       tracked_bodies:
         - [right_hand_base]

       tracked_sensors:
         - [touch_r_f_link1_4]

2. Launch recording:

   .. code-block:: bash

       python nodes/dexrobot_mujoco_ros.py model.xml \
           --config config/recording.yaml \
           --output-formats ros csv mp4 \
           --output-csv-path data/trial1.csv \
           --output-bag-path data/trial1.bag \
           --output-mp4-path data/trial1.mp4

3. Process data:

   .. code-block:: python

       from pathlib import Path
       import pandas as pd
       import rosbag

       def process_trial(trial_dir):
           # Load CSV
           csv_data = pd.read_csv(
               trial_dir / 'trial1.csv'
           )

           # Process bag
           bag_data = []
           with rosbag.Bag(trial_dir / 'trial1.bag') as bag:
               for topic, msg, t in bag.read_messages():
                   bag_data.append((topic, t, msg))

           # Analyze data
           results = analyze_data(csv_data, bag_data)

           return results

Next Steps
---------

- Set up :doc:`vr_visualization`
