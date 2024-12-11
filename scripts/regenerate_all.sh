#!/bin/bash

# Convert URDF to MJCF
python convert_hand.py --urdf ../../dexrobot_urdf/urdf/dexhand021_left.urdf
python convert_hand.py --urdf ../../dexrobot_urdf/urdf/dexhand021_right.urdf

# Articulate with floating base to create floating hand
python articulate_hand.py --base ../dexrobot_mujoco/models/floating_base.xml --hand ../dexrobot_mujoco/models/dexhand021_right.xml --output ../dexrobot_mujoco/models/dexhand021_right_floating.xml --euler "-90" 0 "-90"

# Extract parts for the use of scenes
python extract_parts.py ../dexrobot_mujoco/models/dexhand021_right_floating.xml -o ../dexrobot_mujoco/parts/
