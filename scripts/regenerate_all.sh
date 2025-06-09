#!/bin/bash

# Ensure parts/defaults.xml is in sync with models/defaults.xml
cp ../dexrobot_mujoco/models/defaults.xml ../dexrobot_mujoco/parts/defaults.xml

# Convert URDF to MJCF
python convert_hand.py --urdf ../../dexrobot_urdf/urdf/dexhand021_left.urdf
python convert_hand.py --urdf ../../dexrobot_urdf/urdf/dexhand021_right.urdf
python convert_hand.py --urdf ../../dexrobot_urdf/urdf/dexhand021_left_simplified.urdf --simplified-collision ../dexrobot_mujoco/models/collision_geoms/dexhand021_left_simplified.yaml
python convert_hand.py --urdf ../../dexrobot_urdf/urdf/dexhand021_right_simplified.urdf --simplified-collision ../dexrobot_mujoco/models/collision_geoms/dexhand021_right_simplified.yaml

# Note: All models now generated with TS sensor support by default

# Articulate with floating base to create floating hand
python articulate_hand.py --base ../dexrobot_mujoco/models/floating_base.xml --hand ../dexrobot_mujoco/models/dexhand021_right.xml --output ../dexrobot_mujoco/models/dexhand021_right_floating.xml --euler 0 90 0

# Articulate with floating base to create floating hand with simplified mesh
python articulate_hand.py --base ../dexrobot_mujoco/models/floating_base.xml --hand ../dexrobot_mujoco/models/dexhand021_right_simplified.xml --output ../dexrobot_mujoco/models/dexhand021_right_simplified_floating.xml --euler 0 90 0

# Articulate with JAKA Zu7 to create an arm-hand system
python articulate_hand.py --base ../dexrobot_mujoco/models/jaka_zu7_right.xml --hand ../dexrobot_mujoco/models/dexhand021_right_simplified.xml --output ../dexrobot_mujoco/models/dexhand021_right_jaka_zu7.xml --euler 0 0 0

# Extract parts for the use of scenes (all models now have TS sensor support)
python extract_parts.py ../dexrobot_mujoco/models/dexhand021_right_floating.xml -o ../dexrobot_mujoco/parts/
python extract_parts.py ../dexrobot_mujoco/models/dexhand021_right_jaka_zu7.xml -o ../dexrobot_mujoco/parts/
