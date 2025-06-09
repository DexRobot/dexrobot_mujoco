import os
import argparse
from pathlib import Path
import yaml
import xml.etree.ElementTree as ET
import subprocess
from dexrobot_mujoco.utils.mjcf_utils import (
    urdf2mjcf,
    get_body_names,
    add_trunk_body,
    add_position_actuators,
    add_touch_sensors,
    add_ts_touch_sensors,
    add_sites,
    apply_defaults,
    exclude_self_collisions,
    update_geom_collisions,
)


def _remove_detailed_collision_geometry(mjcf_path):
    """Remove detailed collision geometry (*_4_1 and *_4_2) from MJCF file.
    
    This function removes:
    1. Mesh definitions with names ending in *_4_1 or *_4_2 from <asset> section
    2. Geom elements using those meshes from <worldbody> section
    
    Args:
        mjcf_path (str): Path to the MJCF file to modify
    """
    tree = ET.parse(mjcf_path)
    root = tree.getroot()
    
    # Remove mesh definitions from asset section
    asset_section = root.find('asset')
    if asset_section is not None:
        meshes_to_remove = []
        for mesh in asset_section.findall('mesh'):
            mesh_name = mesh.get('name')
            if mesh_name and ('_4_1' in mesh_name or '_4_2' in mesh_name):
                meshes_to_remove.append(mesh)
        
        for mesh in meshes_to_remove:
            asset_section.remove(mesh)
    
    # Remove geom elements using those meshes from worldbody
    # We need to iterate through all parents to find and remove child geoms
    for parent in root.iter():
        geoms_to_remove = []
        for geom in parent.findall('geom'):
            mesh_name = geom.get('mesh')
            if mesh_name and ('_4_1' in mesh_name or '_4_2' in mesh_name):
                geoms_to_remove.append(geom)
        
        for geom in geoms_to_remove:
            parent.remove(geom)
    
    # Write the modified XML back to file
    tree.write(mjcf_path, encoding="utf-8", xml_declaration=True)
    subprocess.run(["xmllint", "--format", mjcf_path, "--output", mjcf_path])


def convert_hand_urdf(urdf_path=None, output_dir=None, simplified_collision_yaml=None):
    """Convert URDF to MJCF and add necessary configurations for the hand model.

    Args:
        urdf_path (str, optional): Path to input URDF file. If not provided, uses default path.
        output_dir (str, optional): Output directory for MJCF file. If not provided, uses default path.
        simplified_collision_yaml (str): Path to YAML file containing simplified collision model.
            If provided, this will be used instead of generating full collision model.
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
    
    # Always add ts_sensor suffix
    base_name = f"{urdf_path.stem}_ts_sensor"
    output_path = output_dir / f"{base_name}.xml"

    # Convert URDF to MJCF:
    # - Convert both pads and tips to bodies
    # - Convert only pads to sites (for touch sensors)
    urdf2mjcf(str(urdf_path), str(output_dir), 
              fixed_to_body_pattern=r".*(pad|tip).*", 
              fixed_to_site_pattern=r".*pad.*",
              enable_ts_sensor=True)

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

    # Get all site names from the converted MJCF
    tree = ET.parse(str(output_path))
    root = tree.getroot()

    # Find pad sites that were created from fixed links (they start with "site_")
    sensor_sites = {}
    for site in root.findall(".//site"):
        site_name = site.get("name")
        if site_name.startswith("site_") and "pad" in site_name:
            # Only attach touch sensors to pad sites (not tips)
            
            # Save the site name for sensor creation
            body_name = site_name.replace("site_", "")
            sensor_sites[body_name] = site_name

    # Write back any modifications to sites
    tree.write(str(output_path), encoding="utf-8", xml_declaration=True)
    subprocess.run(["xmllint", "--format", str(output_path), "--output", str(output_path)])

    # Always add both TS touch sensors and regular touch sensors
    # TS sensors provide tactile sensor data
    add_ts_touch_sensors(str(output_path), sensor_sites)
    
    # Regular touch sensors provide basic contact sensing (visualization/sensing only, no physics)
    sensor_info = {
        f"touch_{body_name}": {"site": site_name} for body_name, site_name in sensor_sites.items()
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

    # Apply simplified collision geometry by removing *_4_1 and *_4_2 variants
    # This ensures all generated models use simplified collision geometry
    _remove_detailed_collision_geometry(str(output_path))
    
    # Apply collision model if applicable
    if simplified_collision_yaml:
        # Use provided simplified collision model
        if not os.path.exists(simplified_collision_yaml):
            raise FileNotFoundError(f"Simplified collision YAML not found: {simplified_collision_yaml}")
        update_geom_collisions(str(output_path), str(simplified_collision_yaml))


def main():
    parser = argparse.ArgumentParser(description="Convert hand URDF to MJCF format")
    parser.add_argument("--urdf", type=str, help="Input URDF file path")
    parser.add_argument("--simplified-collisions", type=str, help="Path to simplified collision YAML file")

    args = parser.parse_args()

    convert_hand_urdf(args.urdf, simplified_collision_yaml=args.simplified_collisions)


if __name__ == "__main__":
    main()
