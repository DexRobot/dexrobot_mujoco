======================
Collision Models
======================

This section covers the collision model system in DexRobot MuJoCo, including both full mesh-based collisions and simplified primitive-based collisions.

.. figure:: ../assets/hands_simplified_collision_geom.png
   :width: 80%
   :align: center

   Comparison of full mesh-based (left) and simplified primitive-based (right) collision models.

Collision Model Types
-------------------

Full Mesh-Based Collisions
^^^^^^^^^^^^^^^^^^^^^^^
The default collision model uses the full mesh geometry from the URDF:

- Accurate representation of hand geometry
- Higher computational cost
- Better for visualization and precise contact modeling
- Suitable for non-real-time applications

Simplified Primitive-Based Collisions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The optimized collision model uses primitive shapes:

- Boxes for palm and base
- Capsules for finger segments
- Significantly faster computation
- Suitable for real-time applications
- Maintains essential contact behavior

Configuration
------------

Collision Model Definition
^^^^^^^^^^^^^^^^^^^^^^^
Simplified collision models are defined in YAML format:

.. code-block:: yaml

    # Example from dexhand021_right_simplified.yaml
    right_hand_base:
      type: box
      size: [0.0273, 0.05, 0.05]
      pos: [-0.0022, 0, 0.11]

    r_f_link1_2:
      type: capsule
      size: [0.0095]
      fromto: [0.00, 0, 0, 0.03, 0, -0.002]

Box Parameters
~~~~~~~~~~~~
- ``type: box`` - Specifies a box primitive
- ``size: [x, y, z]`` - Half-lengths in each dimension
- ``pos: [x, y, z]`` - Center position relative to link frame

Capsule Parameters
~~~~~~~~~~~~~~~
- ``type: capsule`` - Specifies a capsule primitive
- ``size: [radius]`` - Radius of the capsule
- ``fromto: [x1, y1, z1, x2, y2, z2]`` - Endpoint coordinates

Usage
-----

Converting with Simplified Collisions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Use the ``--simplified-collision`` flag with ``convert_hand.py``:

.. code-block:: bash

    python scripts/convert_hand.py \
        --urdf hand.urdf \
        --simplified-collision config/collision_geoms/dexhand021_right_simplified.yaml

Manual Application
^^^^^^^^^^^^^^^
Apply simplified collisions to an existing MJCF:

.. code-block:: python

    from dexrobot_mujoco.utils.mjcf_utils import update_geom_collisions

    update_geom_collisions(
        xml_file_path='model.xml',
        collision_yaml_path='collision_config.yaml'
    )

Collision Groups
--------------

The collision system uses different groups for visualization and collision:

.. code-block:: xml

    <!-- Visual geometry -->
    <geom group="1" contype="0" conaffinity="0"/>

    <!-- Collision geometry -->
    <geom group="3" contype="1" conaffinity="1"/>

Group Assignments
^^^^^^^^^^^^^^
- Group 1: Visual geometries (no collision)
- Group 3: Collision geometries
- Contype/Conaffinity: Contact generation properties

Collision Exclusions
------------------

Some collisions are automatically excluded to prevent self-interference:

Fingertip-Fingertip
^^^^^^^^^^^^^^^^^
Allows finger-finger contact:

.. code-block:: python

    fingertip_re = r"[lr]_f_link\d_4"  # Matches fingertip links
    allowed_collision_pairs = [
        (fingertip_re, fingertip_re)
    ]

Fingertip-Palm
^^^^^^^^^^^^
Allows fingers to contact palm:

.. code-block:: python

    palm_re = r"[lr]_p_link\d"  # Matches palm links
    allowed_collision_pairs.append(
        (fingertip_re, palm_re)
    )

Implementation
------------

The collision system is implemented in several utility functions:

update_geom_collisions()
^^^^^^^^^^^^^^^^^^^^^^
Updates collision properties based on YAML configuration:

.. code-block:: python

    def update_geom_collisions(xml_file_path, collision_yaml_path):
        """Update collision properties of an MJCF XML file.

        Args:
            xml_file_path: Path to MJCF XML file
            collision_yaml_path: Path to collision config YAML
        """
        # Load collision specifications
        with open(collision_yaml_path, 'r') as f:
            collision_specs = yaml.safe_load(f)

        # Disable existing collisions
        for geom in root.findall(".//body//geom"):
            if not collision_props_set(geom):
                disable_collision(geom)

        # Add collision geometries
        for body_name, spec in collision_specs.items():
            add_collision_geom(body, spec)

exclude_self_collisions()
^^^^^^^^^^^^^^^^^^^^^^
Excludes specified collision pairs:

.. code-block:: python

    def exclude_self_collisions(
        xml_file_path,
        allowed_collision_pairs=[]
    ):
        """Exclude self-collisions except allowed pairs.

        Args:
            xml_file_path: Path to MJCF XML file
            allowed_collision_pairs: List of allowed patterns
        """
        # Find collision pairs to exclude
        pairs = find_collision_pairs(root)

        # Filter allowed pairs
        pairs = filter_allowed_pairs(
            pairs,
            allowed_collision_pairs
        )

        # Add exclusions
        add_collision_exclusions(root, pairs)

Performance Considerations
-----------------------

Mesh vs. Primitive Performance
^^^^^^^^^^^^^^^^^^^^^^^^^^
Performance comparison on a typical system:

+----------------------+-----------------+------------------+
| Collision Model      | Update Time (ms)| Memory Usage (MB)|
+======================+=================+==================+
| Full Mesh           | 0.05      | 500       |
+----------------------+-----------------+------------------+
| Simplified Primitive | 0.02      | 300         |
+----------------------+-----------------+------------------+

Optimization Tips
^^^^^^^^^^^^^^
1. Use simplified collisions for real-time applications
2. Adjust collision margins if needed
3. Consider contact filtering for specific scenarios
4. Balance precision vs. performance based on needs

Troubleshooting
-------------

Common Issues
^^^^^^^^^^^

Missing Collisions
~~~~~~~~~~~~~~~
If collisions aren't working:

1. Check collision group assignments
2. Verify contype/conaffinity settings
3. Review collision exclusions
4. Validate primitive parameters

Unstable Contacts
~~~~~~~~~~~~~~
If experiencing contact instability:

1. Adjust collision margins
2. Check primitive sizes
3. Consider using different primitive types
4. Review solver parameters

Next Steps
---------

After setting up collision models:

- Configure :doc:`actuators`
- Add :doc:`sensors`
- Test with :doc:`examples`
