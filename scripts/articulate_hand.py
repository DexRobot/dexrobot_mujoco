#!/usr/bin/env python3

import os
import argparse
import numpy as np
from scipy.spatial.transform import Rotation as R
from dexrobot_mujoco.utils.mjcf_utils import articulate, add_ground, add_joint_limits

def quaternion_from_euler(roll, pitch, yaw):
    """Convert Euler angles to quaternion."""
    return R.from_euler('xyz', [roll, pitch, yaw]).as_quat()

def main():
    parser = argparse.ArgumentParser(description='Articulate a hand to a floating base')

    # Required arguments
    parser.add_argument('--base', type=str, required=True,
                      help='Path to base MJCF file')
    parser.add_argument('--hand', type=str, required=True,
                      help='Path to hand MJCF file')
    parser.add_argument('--output', type=str, required=True,
                      help='Path to output MJCF file')

    # Optional positioning arguments
    parser.add_argument('--pos', type=float, nargs=3, default=[0, 0, 0],
                      help='Position offset [x y z]')
    parser.add_argument('--euler', type=float, nargs=3,
                      help='Euler angles in degrees [roll pitch yaw]')
    parser.add_argument('--quat', type=float, nargs=4,
                      help='Quaternion [w x y z]')
    parser.add_argument('--add-ground', action='store_true',
                      help='Add a ground plane to the scene')

    args = parser.parse_args()

    # Validate paths
    if not os.path.exists(args.base):
        raise FileNotFoundError(f"Floating base MJCF not found: {args.base}")
    if not os.path.exists(args.hand):
        raise FileNotFoundError(f"Hand MJCF not found: {args.hand}")

    # Create output directory if needed
    os.makedirs(os.path.dirname(args.output), exist_ok=True)

    # Convert position list to string
    pos_str = f"{args.pos[0]} {args.pos[1]} {args.pos[2]}"

    # Handle rotation specification
    if args.euler is not None and args.quat is not None:
        raise ValueError("Cannot specify both euler angles and quaternion")
    elif args.euler is not None:
        # Convert degrees to radians and get quaternion
        quat = quaternion_from_euler(
            *[angle * np.pi / 180 for angle in args.euler]
        )
        quat_str = f"{quat[3]} {quat[0]} {quat[1]} {quat[2]}"  # Convert to w,x,y,z order
    elif args.quat is not None:
        quat_str = f"{args.quat[0]} {args.quat[1]} {args.quat[2]} {args.quat[3]}"
    else:
        quat_str = "1 0 0 0"  # Default to identity rotation

    # Perform articulation
    articulate(
        parent_xml_path=args.base,
        child_xml_path=args.hand,
        link_name="hand_base",
        output_xml_path=args.output,
        pos=pos_str,
        quat=quat_str
    )

    # Add limited="true" to all joints with range (required for Isaac Gym)
    add_joint_limits(args.output)

    # Optionally add ground
    if args.add_ground:
        add_ground(args.output)

if __name__ == '__main__':
    main()
