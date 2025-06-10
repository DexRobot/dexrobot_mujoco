======================
MuJoCo Control Wrapper
======================

The MuJoCo Control Wrapper (``MJSimWrapper``) provides a high-level interface for controlling MuJoCo simulations with real-time visualization, reset-safe operations, and modular sensor/VR support.

Key Features
============

Real-time Physics Stepping
--------------------------

The wrapper implements a sophisticated physics stepping system to ensure real-time visualization:

.. code-block:: python

    from dexrobot_mujoco.wrapper import MJSimWrapper
    
    # Initialize wrapper
    wrapper = MJSimWrapper("model.xml")
    
    # Step simulation in real-time
    while wrapper.viewer.is_running():
        # Physics step synchronized with wall clock
        wrapper.step()  # Automatically maintains real-time

The ``step()`` method:

- Advances physics simulation by one timestep
- Synchronizes with the viewer automatically
- Maintains real-time execution by sleeping when needed
- Can step to a specific simulation time: ``wrapper.step(until=5.0)``

Reset-Safe Operations
---------------------

All control operations are protected against race conditions during reset:

.. code-block:: python

    @reset_safe
    def get_qpos(self, joint_name):
        """Safe position reading during reset."""
        qpos_addr = self.get_qpos_addr(joint_name)
        return self.data.qpos[qpos_addr]

The ``@reset_safe`` decorator ensures operations are skipped during simulation reset, preventing crashes and data corruption.

Modular Architecture
--------------------

The wrapper supports optional managers for specialized functionality:

.. code-block:: python

    from dexrobot_mujoco.wrapper import MJSimWrapper, TSSensorManager, VRManager
    
    # Create wrapper
    wrapper = MJSimWrapper("scene.xml")
    
    # Add optional managers
    ts_manager = TSSensorManager(wrapper)
    wrapper.set_sensor_manager(ts_manager)
    
    vr_manager = VRManager(wrapper)
    wrapper.set_vr_manager(vr_manager)

Core API
========

Initialization
--------------

.. code-block:: python

    wrapper = MJSimWrapper(
        model_path="path/to/model.xml",
        mesh_dir=None,  # Optional custom mesh directory
        renderer_dimension=(640, 480),  # Optional for image rendering
        seed=0  # Random seed for reproducibility
    )

Joint Control
-------------

.. code-block:: python

    # Send control signal (with automatic wrapping for revolute joints)
    wrapper.send_control("act_r_f_joint1_1", 0.5)
    
    # Get joint position
    pos = wrapper.get_qpos("r_f_joint1_1")
    
    # Set joint position directly
    wrapper.set_qpos("r_f_joint1_1", 0.5)
    
    # Get joint velocity
    vel = wrapper.get_qvel("r_f_joint1_1")

Free Joint Control
------------------

For floating base or free joints:

.. code-block:: python

    # Get free joint state (7 DOF position, 6 DOF velocity)
    qpos = wrapper.get_qpos_freejoint("base_joint")  # [x,y,z,qw,qx,qy,qz]
    qvel = wrapper.get_qvel_freejoint("base_joint")  # [vx,vy,vz,wx,wy,wz]
    
    # Set free joint state
    wrapper.set_qpos_freejoint("base_joint", np.array([0,0,1, 1,0,0,0]))
    wrapper.set_qvel_freejoint("base_joint", np.zeros(6))

Camera Control
--------------

.. code-block:: python

    # Set camera view
    wrapper.set_view_angle(
        lookat=[0, 0, 0.5],  # Look-at point
        distance=1.5,         # Distance from look-at
        elevation=-20,        # Elevation angle (degrees)
        azimuth=45           # Azimuth angle (degrees)
    )

Configuration Loading
---------------------

Load initial configurations from YAML:

.. code-block:: python

    wrapper.parse_yaml("config.yaml")

Example YAML configuration:

.. code-block:: yaml

    camera:
      lookat: [0.0, 0.0, 1.2]
      distance: 1.2
      elevation: -20
      azimuth: 0
      
    initial_ctrl:
      act_r_f_joint1_1: 0.5
      act_r_f_joint2_1: 0.3
      
    initial_qpos_freejoint:
      base_joint: [0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0]

Viewer Integration
==================

The wrapper provides seamless integration with MuJoCo's viewer:

.. code-block:: python

    # Launch passive viewer (recommended)
    wrapper.launch_viewer("passive", show_ui=True)
    
    # Key callbacks are automatically registered:
    # - 'R': Reset simulation
    # - 'Q': Quit viewer
    # - 'S': Toggle shadows/reflections
    # - 'C': Print camera position
    # - 'D': Enter debug mode

Advanced Features
=================

Infinite Rotation
-----------------

Enable infinite rotation for continuous joints:

.. code-block:: python

    # Allow infinite rotation for all finger joints
    wrapper.enable_infinite_rotation("act_.*_f_joint.*")

Image Rendering
---------------

Render frames for video recording or computer vision:

.. code-block:: python

    # Initialize with renderer
    wrapper = MJSimWrapper("model.xml", renderer_dimension=(1920, 1080))
    
    # Render frame
    frame = wrapper.render_frame()  # Returns RGB numpy array

Reset Handling
--------------

Safe simulation reset with state preservation:

.. code-block:: python

    # Reset simulation (thread-safe)
    wrapper.reset_simulation()
    
    # All configurations from YAML are automatically restored
    # Reset-safe decorators prevent operations during reset

Example: Complete Control Loop
==============================

.. code-block:: python

    from dexrobot_mujoco.wrapper import MJSimWrapper
    import time
    
    # Load model with configuration
    wrapper = MJSimWrapper("dexrobot_mujoco/scenes/box.xml")
    wrapper.parse_yaml("config/scene_default.yaml")
    wrapper.launch_viewer("passive")
    
    # Control loop
    start_time = time.time()
    while wrapper.viewer.is_running():
        # Apply control
        t = time.time() - start_time
        wrapper.send_control("act_r_f_joint1_1", 0.5 * np.sin(t))
        
        # Step physics in real-time
        wrapper.step()

See Also
--------

- :doc:`/touch_sensors` - Touch sensor integration
- :doc:`/ros_integration/index` - ROS integration details