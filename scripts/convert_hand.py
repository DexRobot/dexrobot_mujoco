import os
import argparse
from pathlib import Path
from dexrobot_mujoco.utils.mjcf_utils import (
    urdf2mjcf,
    get_body_names,
    add_trunk_body,
    add_position_actuators,
    add_touch_sensors,
    add_sites,
    apply_defaults,
    exclude_self_collisions,
)


def convert_hand_urdf(urdf_path=None, output_dir=None):
    """Convert URDF to MJCF and add necessary configurations for the hand model.

    Args:
        urdf_path (str, optional): Path to input URDF file. If not provided, uses default path.
        output_dir (str, optional): Output directory for MJCF file. If not provided, uses default path.
    """
    # Set up paths
    current_dir = Path(__file__).parent
    if urdf_path is None:
        urdf_path = current_dir / "../../dexrobot_urdf/urdf/dexhand021_right.urdf"

    urdf_path = Path(urdf_path)
    if not urdf_path.exists():
        raise FileNotFoundError(f"URDF file not found: {urdf_path}")

    # Set up output paths
    output_dir = current_dir / "../dexrobot_mujoco/models"
    output_dir.mkdir(exist_ok=True)
    output_path = output_dir / f"{urdf_path.stem}.xml"

    # Convert URDF to MJCF
    urdf2mjcf(str(urdf_path), str(output_dir))

    # Add options and defaults
    apply_defaults(
        str(output_path), str(current_dir / "../dexrobot_mujoco/models/defaults.xml")
    )

    # Configure actuators
    actuator_config = {
        # Bend joints
        r"[lr]_f_joint[1-5]_[2-4]": {
            "kp": "20",
            "kv": "0.1",
            "ctrlrange": "0 1.3",
            "forcerange": "-20 20",
        },
        # Rotation/spread joints
        r"[lr]_f_joint1_1": {  # thumb
            "kp": "20",
            "kv": "1",
            "ctrlrange": "0 2.2",
            "forcerange": "-20 20",
        },
        r"[lr]_f_joint2_1": {  # index
            "kp": "20",
            "kv": "1",
            "ctrlrange": "0 0.3",
            "forcerange": "-20 20",
        },
        r"[lr]_f_joint3_1": {  # middle
            "kp": "20",
            "kv": "1",
            "ctrlrange": "-0.0001 0.0001",
            "forcerange": "-20 20",
        },
        r"[lr]_f_joint4_1": {  # ring
            "kp": "20",
            "kv": "1",
            "ctrlrange": "0 0.3",
            "forcerange": "-20 20",
        },
        r"[lr]_f_joint5_1": {  # pinky
            "kp": "20",
            "kv": "1",
            "ctrlrange": "0 0.6",
            "forcerange": "-20 20",
        },
    }
    add_position_actuators(str(output_path), actuator_config)

    left_fingertip_names = [f"l_f_link{i}_4" for i in range(1, 6)]
    right_fingertip_names = [f"r_f_link{i}_4" for i in range(1, 6)]
    available_bodies = get_body_names(str(output_path))
    if all(name in available_bodies for name in left_fingertip_names):
        handedness = "left"
        fingertip_names = left_fingertip_names
    elif all(name in available_bodies for name in right_fingertip_names):
        handedness = "right"
        fingertip_names = right_fingertip_names
    else:
        raise ValueError("Could not determine handedness of the hand model")

    # Configure touch sensors
    sensor_sites = {
        name: {"pos": "0.025 0.003 0", "size": "0.01", "type": "sphere"} for name in fingertip_names
    }
    add_sites(str(output_path), sensor_sites)

    sensor_info = {
        f"touch_{link}": {"site": f"site_{link}"} for link in sensor_sites.keys()
    }
    add_touch_sensors(str(output_path), sensor_info)

    # Add a base body to wrap everything else
    add_trunk_body(str(output_path), f"{handedness}_hand_base")

    # Exclude unwanted collision pairs
    fingertip_re = r"[lr]_f_link\d_4"  # Matches r_f_link1_4, r_f_link2_4, etc.
    palm_re = r"[lr]_p_link\d"  # Matches r_p_link0, r_p_link1, etc.
    allowed_collision_pairs = [
        # Allow fingertip-fingertip collisions
        (fingertip_re, fingertip_re),
        # Allow fingertip-palm collisions
        (fingertip_re, palm_re),
    ]
    exclude_self_collisions(output_path, allowed_collision_pairs=allowed_collision_pairs)


def main():
    parser = argparse.ArgumentParser(description="Convert hand URDF to MJCF format")
    parser.add_argument("--urdf", type=str, help="Input URDF file path")
    args = parser.parse_args()

    convert_hand_urdf(args.urdf)


if __name__ == "__main__":
    main()