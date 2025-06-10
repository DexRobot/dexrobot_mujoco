======================
DexRobot MuJoCo
======================

Tactile simulation for DexRobot hands in MuJoCo with ROS integration.

.. figure:: assets/hands.png
   :width: 80%
   :align: center

   DexHand Model in MuJoCo

Key Features
-----------
- **Tactile sensing simulation** with MuJoCo native and TaShan 11-dimensional sensors
- **Real-time physics stepping** for synchronized visualization
- **Collision model variants** (full and simplified) for performance tuning
- **ROS/ROS2 integration** with standard message types
- **VR visualization** support
- **Multi-format recording** (ROS bags, CSV, MP4)

Documentation Contents
--------------------

.. toctree::
   :maxdepth: 2
   :caption: User Guide
   :titlesonly:

   getting-started
   touch_sensors
   wrapper/index
   hand_models/index
   scenes/index
   ros_integration/index
   tashan_python38_setup

.. toctree::
   :maxdepth: 2
   :caption: API Reference

   api/index


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

Support
-------
For issues, questions, or contributions:

* Check our `GitHub Issues <https://github.com/dexrobot/dexrobot_mujoco/issues>`_
* Submit pull requests via GitHub

License
-------
Copyright 2024 DexRobot

Licensed under the Apache License, Version 2.0. See LICENSE file for details.
