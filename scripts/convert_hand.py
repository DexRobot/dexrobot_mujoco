import os
import argparse
from pathlib import Path
import yaml
import xml.etree.ElementTree as ET
import subprocess
import re
from dexrobot_mujoco.utils.mjcf_utils import (
    urdf2mjcf,
    get_body_names,
    add_trunk_body,
    add_position_actuators,
    add_touch_sensors,
    add_rangefinder_sensors,
    add_user_sensors,
    add_sites,
    apply_defaults,
    exclude_self_collisions,
    update_geom_collisions,
)


def _add_ts_sensor_components(mjcf_path, sensor_sites):
    """Add TS sensor components required for TSensor callback.
    
    Creates the essential force sensor bodies (force1_f1, force2_f1, etc.) that
    the TSensor callback needs for tactile simulation. These are not just visual
    components but functional elements required for the sensor to work.
    
    Args:
        mjcf_path (str): Path to the MJCF file
        sensor_sites (dict): Dictionary mapping body names to site names
    """
    tree = ET.parse(mjcf_path)
    root = tree.getroot()
    
    # Add TS sensor mesh assets if not already present
    asset_element = root.find("asset")
    if asset_element is not None:
        existing_meshes = [mesh.get("name") for mesh in asset_element.findall("mesh")]
        ts_meshes = ["f1", "f2", "f3", "f4", "f5", "f6", "f7"]
        
        for i, mesh_name in enumerate(ts_meshes, 1):
            if mesh_name not in existing_meshes:
                mesh_element = ET.SubElement(asset_element, "mesh", name=mesh_name)
                mesh_element.set("file", f"F{i}b.stl")
                mesh_element.set("scale", ".001 .001 .001")
    
    # Add TS sensor components to distal finger link bodies
    for body_name, site_name in sensor_sites.items():
        # Find the corresponding distal finger link body (e.g., r_f_link1_4 for r_f_link1_pad)
        distal_body_name = body_name.replace("_pad", "_4")
        distal_body = None
        for body in root.iter("body"):
            if body.get("name") == distal_body_name:
                distal_body = body
                break
        
        if distal_body is not None:
            # Create wrapper body for TS sensor positioning
            ts_wrapper = ET.SubElement(distal_body, "body", name=f"ts_wrapper_{body_name}")
            ts_wrapper.set("pos", "0.0425 -0.0188 -0.0055")
            ts_wrapper.set("quat", "0.69101 -0.15003 -0.69101 -0.15003")
            
            # Add TS sensor component meshes
            ts_positions = [
                ("f1", "0 0 0.02"),
                ("f2", "0 0 0.0192"), 
                ("f3", "0.00545 0 0.02"),
                ("f4", "0 0 0.02465"),
                ("f5", "-0.0001 0 0.0285"),
                ("f6", "0.0055 0 0.0285"),
                ("f7", "0 0 0.01362")
            ]
            
            for mesh_name, pos in ts_positions:
                # Use the exact naming pattern required by TSensor callback: force{N}_f{M}
                # Extract finger number from body_name (e.g., r_f_link1_pad -> finger 1)
                finger_match = re.search(r'_link(\d)_', body_name)
                if finger_match:
                    finger_num = finger_match.group(1)
                    force_num = mesh_name[1]  # Extract number from f1, f2, etc.
                    ts_body_name = f"force{finger_num}_f{force_num}"
                else:
                    # Fallback to original naming if pattern doesn't match
                    ts_body_name = f"ts_{mesh_name}_{body_name}"
                
                ts_body = ET.SubElement(ts_wrapper, "body", name=ts_body_name)
                ts_body.set("pos", pos)
                
                # Add geom for TS sensor component
                ts_geom = ET.SubElement(ts_body, "geom", type="mesh")
                ts_geom.set("rgba", "1 0.3 0.3 1")
                ts_geom.set("mesh", mesh_name)
    
    # Write back
    tree.write(mjcf_path, encoding="utf-8", xml_declaration=True)
    subprocess.run(["xmllint", "--format", mjcf_path, "--output", mjcf_path])


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
    
    # Use base name without suffix (TS sensors are now included by default)
    base_name = urdf_path.stem
    output_path = output_dir / f"{base_name}.xml"

    # Convert URDF to MJCF:
    # - Convert both pads and tips to bodies
    # - Convert only pads to sites (for touch sensors)
    urdf2mjcf(str(urdf_path), str(output_dir), 
              fixed_to_body_pattern=r".*(pad|tip).*", 
              fixed_to_site_pattern=r".*pad.*")

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
            "forcelimited": "true",
        },
        # Rotation/spread joints
        r"[lr]_f_joint1_1": {  # thumb
            "kp": "20",
            "kv": "1",
            "ctrlrange": "0 2.2",
            "forcerange": "-20 20",
            "forcelimited": "true",
        },
        r"[lr]_f_joint2_1": {  # index
            "kp": "20",
            "kv": "1",
            "ctrlrange": "0 0.3",
            "forcerange": "-20 20",
            "forcelimited": "true",
        },
        r"[lr]_f_joint3_1": {  # middle
            "kp": "20",
            "kv": "1",
            "ctrlrange": "-0.0001 0.0001",
            "forcerange": "-20 20",
            "forcelimited": "true",
        },
        r"[lr]_f_joint4_1": {  # ring
            "kp": "20",
            "kv": "1",
            "ctrlrange": "0 0.3",
            "forcerange": "-20 20",
            "forcelimited": "true",
        },
        r"[lr]_f_joint5_1": {  # pinky
            "kp": "20",
            "kv": "1",
            "ctrlrange": "0 0.6",
            "forcerange": "-20 20",
            "forcelimited": "true",
        },
    }
    add_position_actuators(str(output_path), actuator_config)

    left_distal_link_names = [f"l_f_link{i}_4" for i in range(1, 6)]
    right_distal_link_names = [f"r_f_link{i}_4" for i in range(1, 6)]
    available_bodies = get_body_names(str(output_path))
    if all(name in available_bodies for name in left_distal_link_names):
        handedness = "left"
        distal_link_names = left_distal_link_names
    elif all(name in available_bodies for name in right_distal_link_names):
        handedness = "right"
        distal_link_names = right_distal_link_names
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

    # Add sensors in the exact order required by TSensor callback
    # 1. Add regular touch sensors (basic contact sensing)
    sensor_info = {
        f"touch_{body_name}": {"site": site_name} for body_name, site_name in sensor_sites.items()
    }
    add_touch_sensors(str(output_path), sensor_info)
    
    # 2. Add rangefinder sensors (required by TSensor callback for proximity data)
    # TSensor callback looks for rf1, rf2, rf3, rf4, rf5 to determine sensor range
    # These provide input data for tactile simulation
    sorted_sites = sorted(sensor_sites.items())  # Ensure consistent ordering
    
    # First, create sites on distal finger link bodies for rangefinder sensors
    # These sites need to be created on the _4 bodies (distal finger links) not _pad sites
    distal_link_sites = {}
    for i, (body_name, site_name) in enumerate(sorted_sites, 1):
        # Find the corresponding distal finger link body (replace _pad with _4)
        distal_body_name = body_name.replace("_pad", "_4")
        # Use the exact same positioning and attributes as in the reference model
        distal_link_sites[distal_body_name] = {
            "pos": "0.0143 0.006 0", 
            "size": "0.0001", 
            "type": "sphere", 
            "rgba": "1 0 0 1", 
            "zaxis": "0.46 1 0"
        }
        
    # Add sites to distal finger link bodies for rangefinder sensors
    add_sites(str(output_path), distal_link_sites)
    
    # Now create rangefinder sensors that reference these sites
    rangefinder_info = {}
    for i, (body_name, site_name) in enumerate(sorted_sites, 1):
        distal_body_name = body_name.replace("_pad", "_4")
        distal_site_name = f"site_{distal_body_name}"
        rangefinder_info[f"rf{i}"] = {"site": distal_site_name, "cutoff": "0.1"}
    
    add_rangefinder_sensors(str(output_path), rangefinder_info)
    
    # 3. Add TS user sensors (11-dimensional tactile data output)
    # TSensor callback writes computed tactile data to TS-F-A-1, TS-F-A-2, etc.
    # These must be placed AFTER rangefinders for correct ID range calculation
    ts_sensor_info = {}
    for i, _ in enumerate(sorted_sites, 1):
        ts_sensor_info[f"TS-F-A-{i}"] = {"dim": "11", "noise": "0.0"}
    
    add_user_sensors(str(output_path), ts_sensor_info)
    
    # 4. Add TS sensor components (essential for TSensor callback)
    _add_ts_sensor_components(str(output_path), sensor_sites)

    # Add a base body to wrap everything else
    add_trunk_body(str(output_path), f"{handedness}_hand_base")

    # Exclude unwanted collision pairs
    distal_link_re = r"[lr]_f_link\d_4"  # Matches r_f_link1_4, r_f_link2_4, etc.
    palm_re = r"[lr]_p_link\d"  # Matches r_p_link0, r_p_link1, etc.
    allowed_collision_pairs = [
        # Allow distal finger link collisions with each other
        (distal_link_re, distal_link_re),
        # Allow distal finger link collisions with palm
        (distal_link_re, palm_re),
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
