import mujoco
import os
import xml.etree.ElementTree as ET
import re
import subprocess
import yaml
import numpy as np
from scipy.spatial.transform import Rotation
from loguru import logger


def load_meshes(mesh_dir):
    """Load all mesh files in the given directory and return them as a dictionary.

    Args:
        mesh_dir (str): The directory containing the mesh files.

    Returns:
        dict[str, bytes]: A dictionary mapping the filenames to the contents of the mesh files.
    """
    meshes = {
        os.path.basename(filename): open(f"{mesh_dir}/{filename}", "rb").read()
        for filename in os.listdir(mesh_dir)
    }
    return meshes


def urdf2mjcf(urdf_path, mjcf_dir, mesh_dir=None, fixed_to_body_pattern=None, fixed_to_site_pattern=None):
    """Load a URDF file and save it to an MJCF XML file with enhanced fixed joint handling.
    
    By default, MuJoCo's URDF converter ignores fixed links or converts them to mere geoms.
    This function extends the conversion by allowing fixed links to be explicitly converted
    to either MuJoCo bodies (with geoms) or MuJoCo sites, based on link name patterns.

    Args:
        urdf_path (str): The path to the URDF file.
        mjcf_dir (str): The directory to save the output MJCF file.
        mesh_dir (str, optional): The directory containing the mesh files. When not provided, the default search rule of MuJoCo is used.
        fixed_to_body_pattern (str, optional): Regex pattern to match fixed link names that should be converted to MuJoCo bodies.
        fixed_to_site_pattern (str, optional): Regex pattern to match fixed link names that should be converted to MuJoCo sites.
    """
    # First convert using MuJoCo's native converter
    if mesh_dir is None:
        m = mujoco.MjModel.from_xml_path(urdf_path)
    else:
        m = mujoco.MjModel.from_xml_path(urdf_path, load_meshes(mesh_dir))

    # Generate output filename
    base_name = os.path.splitext(os.path.basename(urdf_path))[0]
    output_path = f"{mjcf_dir}/{base_name}.xml"
    mujoco.mj_saveLastXML(output_path, m)

    # If no patterns specified, we're done
    if fixed_to_body_pattern is None and fixed_to_site_pattern is None:
        return

    # Otherwise, we need to process the URDF and MJCF to convert fixed links

    # Parse the original URDF to find fixed links
    urdf_tree = ET.parse(urdf_path)
    urdf_root = urdf_tree.getroot()

    # Find all fixed joints and their child links
    fixed_joints = []
    for joint in urdf_root.findall(".//joint"):
        if joint.get("type") == "fixed":
            parent = None
            child = None

            for elem in joint:
                if elem.tag == "parent":
                    parent = elem.get("link")
                elif elem.tag == "child":
                    child = elem.get("link")

            if parent and child:
                joint_info = {
                    "parent": parent,
                    "child": child,
                    "xyz": None,
                    "rpy": None,
                    "as_body": fixed_to_body_pattern is not None and re.match(fixed_to_body_pattern, child),
                    "as_site": fixed_to_site_pattern is not None and re.match(fixed_to_site_pattern, child)
                }
                
                # Extract origin information if available
                origin = joint.find("origin")
                if origin is not None:
                    if "xyz" in origin.attrib:
                        joint_info["xyz"] = origin.get("xyz")
                    if "rpy" in origin.attrib:
                        joint_info["rpy"] = origin.get("rpy")
                
                # Only add if we need to convert this link
                if joint_info["as_body"] or joint_info["as_site"]:
                    fixed_joints.append(joint_info)

    # If no fixed links match our criteria, we're done
    if not fixed_joints:
        logger.info("No fixed links to convert")
        return

    # Parse the converted MJCF
    mjcf_tree = ET.parse(output_path)
    mjcf_root = mjcf_tree.getroot()

    # Find all bodies and create a map from body name to body element
    body_map = {}
    for body in mjcf_root.findall(".//body"):
        body_map[body.get("name")] = body

    # Convert fixed links as specified by patterns
    for joint_info in fixed_joints:
        parent_name = joint_info["parent"]
        child_name = joint_info["child"]
        
        if parent_name not in body_map:
            logger.warning(f"Parent body {parent_name} not found in MJCF for fixed joint to {child_name}")
            continue
            
        parent_body = body_map[parent_name]

        # Get link element from URDF for geometry information
        link_elem = urdf_root.find(f".//link[@name='{child_name}']")
        if link_elem is None:
            logger.warning(f"Link {child_name} not found in URDF, but was referenced by a fixed joint")
            continue
            
        # Parse visual geometry from URDF
        visual = link_elem.find(".//visual/geometry")
        
        # Common position and orientation attributes for both body and site
        pos_attrs = {}
        if joint_info["xyz"]:
            pos_attrs["pos"] = joint_info["xyz"]
        if joint_info["rpy"]:
            # Convert RPY (roll-pitch-yaw Euler angles) to quaternion
            try:
                # Parse RPY string to array of floats
                rpy = [float(val) for val in joint_info["rpy"].split()]
                
                # Use scipy's Rotation to convert from Euler angles to quaternion
                # URDF uses 'xyz' Euler angle sequence
                rotation = Rotation.from_euler('xyz', rpy)
                
                # Get quaternion with scalar first (w,x,y,z), which matches MuJoCo's format
                quat = rotation.as_quat(scalar_first=True)
                
                # Format as space-separated string for MuJoCo
                pos_attrs["quat"] = " ".join(map(str, quat))
                logger.info(f"Converted RPY {joint_info['rpy']} to quaternion {pos_attrs['quat']}")
            except Exception as e:
                logger.error(f"Failed to convert RPY to quaternion: {e}")
                logger.warning(f"Rotation information for {child_name} will be lost")
        
        # Process geometry information
        geom_attrs = {}
        if visual is not None:
            # Get geometry type and properties
            # MuJoCo uses half-sizes while URDF uses full sizes, so we need to convert
            if visual.find("box") is not None:
                box = visual.find("box")
                if "size" not in box.attrib:
                    logger.warning(f"Box geometry for link {child_name} missing required 'size' attribute")
                    continue
                
                # Parse box size and halve the dimensions for MuJoCo
                full_size = [float(val) for val in box.get("size").split()]
                half_size = [val / 2 for val in full_size]
                
                geom_attrs["type"] = "box"
                geom_attrs["size"] = " ".join(map(str, half_size))
                
            elif visual.find("sphere") is not None:
                sphere = visual.find("sphere")
                if "radius" not in sphere.attrib:
                    logger.warning(f"Sphere geometry for link {child_name} missing required 'radius' attribute")
                    continue
                
                # For spheres, MuJoCo size is the same as URDF radius
                radius = float(sphere.get("radius"))
                
                geom_attrs["type"] = "sphere"
                geom_attrs["size"] = str(radius)
                
            elif visual.find("cylinder") is not None:
                cylinder = visual.find("cylinder")
                if "radius" not in cylinder.attrib or "length" not in cylinder.attrib:
                    logger.warning(f"Cylinder geometry for link {child_name} missing required attributes")
                    continue
                
                # For cylinders, we keep radius the same but halve the length
                radius = float(cylinder.get("radius"))
                length = float(cylinder.get("length")) / 2  # Half the length
                
                geom_attrs["type"] = "cylinder"
                geom_attrs["size"] = f"{radius} {length}"
            else:
                logger.warning(f"Unknown geometry type for link {child_name}, using defaults")
            
            # Get color if available
            material = link_elem.find(".//visual/material/color")
            if material is not None and "rgba" in material.attrib:
                geom_attrs["rgba"] = material.get("rgba")
        
        # Convert to body if specified
        if joint_info["as_body"]:
            # Create body element with position and orientation
            body_attrs = {"name": child_name, **pos_attrs}  # Include pos and quat
            child_body = ET.SubElement(parent_body, "body", body_attrs)
            
            # Add geom to body if we have geometry info
            if geom_attrs:
                geom_attrs_with_name = {"name": f"geom_{child_name}", **geom_attrs}
                ET.SubElement(child_body, "geom", geom_attrs_with_name)
                
            logger.info(f"Converted fixed link {child_name} to body in {parent_name} with pos={pos_attrs.get('pos', 'None')} quat={pos_attrs.get('quat', 'None')}")
            
            # Add to body map in case another link has this as parent
            body_map[child_name] = child_body
        
        # Convert to site if specified
        if joint_info["as_site"]:
            # Create site attributes with position, orientation and geometry
            site_attrs = {"name": f"site_{child_name}", **pos_attrs, **geom_attrs}
            
            # Create and add the site element to the parent body
            ET.SubElement(parent_body, "site", site_attrs)
            logger.info(f"Converted fixed link {child_name} to site in {parent_name} with pos={pos_attrs.get('pos', 'None')} quat={pos_attrs.get('quat', 'None')}")

    # Write the modified MJCF back to the file
    mjcf_tree.write(output_path, encoding="utf-8", xml_declaration=True)

    # Format the XML file
    subprocess.run(["xmllint", "--format", output_path, "--output", output_path])

def get_joint_names(xml_path):
    """Get the names of all joints in the MJCF XML file.

    Args:
        xml_path (str): The path to the MJCF XML file.

    Returns:
        list[str]: A list of joint names.
    """
    tree = ET.parse(xml_path)
    root = tree.getroot()
    return [joint.get("name") for joint in root.findall(".//joint")]

def get_body_names(xml_path):
    """Get the names of all bodies in the MJCF XML file.

    Args:
        xml_path (str): The path to the MJCF XML file.

    Returns:
        list[str]: A list of body names.
    """
    tree = ET.parse(xml_path)
    root = tree.getroot()
    return [body.get("name") for body in root.findall(".//body")]


def add_position_actuators(xml_path, actuator_info):
    """Add position actuators to the given MJCF XML file.

    Args:
        xml_path (str): The path to the MJCF XML file.
        actuator_info (dict): Dictionary containing the actuator information.
            key (str): Regular expression for a group of joints.
            value (dict): A set of properties to add to the <position> element.
    """
    # Load the MJCF XML file
    tree = ET.parse(xml_path)
    root = tree.getroot()

    # Find or create the actuator element
    actuator_element = root.find("actuator")
    if actuator_element is None:
        actuator_element = ET.SubElement(root, "actuator")

    # Collect existing actuator names
    existing_actuator_names = {
        actuator.get("name") for actuator in actuator_element.findall("position")
    }

    # Process each regular expression and its associated properties
    for joint_names_re, properties in actuator_info.items():
        joint_re = re.compile(joint_names_re)
        # Find all joints matching the regular expression
        for joint in root.findall(".//joint"):
            joint_name = joint.get("name")
            if joint_name and joint_re.match(joint_name):
                actuator_name = f"act_{joint_name}"
                if actuator_name not in existing_actuator_names:
                    # Create the position element with the provided properties
                    position_element = ET.SubElement(
                        actuator_element,
                        "position",
                        name=actuator_name,
                        joint=joint_name,
                    )
                    for prop_name, prop_value in properties.items():
                        position_element.set(prop_name, str(prop_value))

    # Save the modified MJCF XML back to the file
    tree.write(xml_path, encoding="utf-8", xml_declaration=True)

    # Call xmllint to prettify the XML file
    subprocess.run(["xmllint", "--format", xml_path, "--output", xml_path])


def add_touch_sensors(xml_path, sensor_info):
    """
    Add touch sensors to the given MJCF XML file.

    Args:
        xml_path (str): The path to the MJCF XML file.
        sensor_info (dict): Dictionary containing the sensor information.
            key (str): The sensor name.
            value (dict): A set of properties to add to the <sensor> element.

    """
    # Load the MJCF XML file
    tree = ET.parse(xml_path)
    root = tree.getroot()

    # Find or create the sensor element
    sensor_element = root.find("sensor")
    if sensor_element is None:
        sensor_element = ET.SubElement(root, "sensor")

    # Process each joint and its associated properties
    for name, properties in sensor_info.items():
        # Create the touch sensor element
        touch_sensor = ET.SubElement(sensor_element, "touch", name=name)
        for prop_name, prop_value in properties.items():
            touch_sensor.set(prop_name, str(prop_value))

    # Save the modified MJCF XML back to the file
    tree.write(xml_path, encoding="utf-8", xml_declaration=True)

    # Call xmllint to prettify the XML file
    subprocess.run(["xmllint", "--format", xml_path, "--output", xml_path])


def add_rangefinder_sensors(xml_path, sensor_info):
    """
    Add rangefinder sensors to the given MJCF XML file.

    Args:
        xml_path (str): The path to the MJCF XML file.
        sensor_info (dict): Dictionary containing the sensor information.
            key (str): The sensor name.
            value (dict): A set of properties to add to the <rangefinder> element.
                Must include 'site' property for rangefinder sensors.

    Example:
        sensor_info = {
            "rf1": {"site": "site_r_f_link1_4", "cutoff": "0.1"},
            "rf2": {"site": "site_r_f_link2_4", "cutoff": "0.1"},
        }
    """
    # Load the MJCF XML file
    tree = ET.parse(xml_path)
    root = tree.getroot()

    # Find or create the sensor element
    sensor_element = root.find("sensor")
    if sensor_element is None:
        sensor_element = ET.SubElement(root, "sensor")

    # Process each sensor and its properties
    for name, properties in sensor_info.items():
        # Create the rangefinder sensor element
        rangefinder_attrs = {
            "name": name,
        }
        # Update with provided properties
        for prop_name, prop_value in properties.items():
            rangefinder_attrs[prop_name] = str(prop_value)
        
        rangefinder_sensor = ET.SubElement(sensor_element, "rangefinder", **rangefinder_attrs)

    # Save the modified MJCF XML back to the file
    tree.write(xml_path, encoding="utf-8", xml_declaration=True)

    # Call xmllint to prettify the XML file
    subprocess.run(["xmllint", "--format", xml_path, "--output", xml_path])


def add_user_sensors(xml_path, sensor_info):
    """
    Add user-defined sensors to the given MJCF XML file.

    Args:
        xml_path (str): The path to the MJCF XML file.
        sensor_info (dict): Dictionary containing the sensor information.
            key (str): The sensor name.
            value (dict): A set of properties to add to the <user> element.
                Must include 'dim' property for user sensors.

    Example:
        sensor_info = {
            "TS-F-A-1": {"dim": "11", "noise": "0.0"},
            "TS-F-A-2": {"dim": "11", "noise": "0.0"},
        }
    """
    # Load the MJCF XML file
    tree = ET.parse(xml_path)
    root = tree.getroot()

    # Find or create the sensor element
    sensor_element = root.find("sensor")
    if sensor_element is None:
        sensor_element = ET.SubElement(root, "sensor")

    # Process each sensor and its properties
    for name, properties in sensor_info.items():
        # Create the user sensor element with required attributes
        user_attrs = {
            "name": name,
        }
        # Update with provided properties
        for prop_name, prop_value in properties.items():
            user_attrs[prop_name] = str(prop_value)
        
        user_sensor = ET.SubElement(sensor_element, "user", **user_attrs)

    # Save the modified MJCF XML back to the file
    tree.write(xml_path, encoding="utf-8", xml_declaration=True)

    # Call xmllint to prettify the XML file
    subprocess.run(["xmllint", "--format", xml_path, "--output", xml_path])


def add_sites(xml_file_path, site_info):
    """
    Add sites to specific bodies in the MJCF XML file.

    Args:
        xml_file_path (str): Path to the input/output MJCF XML file.
        site_info (dict): Dictionary containing the site information.
            key (str): The body name to add the site to.
            value (dict): A set of properties to add to the <site> element.

    Returns:
        None
    """
    # Parse the XML file
    tree = ET.parse(xml_file_path)
    root = tree.getroot()

    # Find the "worldbody" element
    worldbody = root.find("worldbody")

    if worldbody is None:
        raise ValueError("The provided MJCF file does not contain a <worldbody> element.")

    # Iterate through all bodies in the worldbody to find specific bodies and add sites
    logger.info("Adding sites to specific bodies in the MJCF XML file.")
    for body in root.findall(".//body"):
        body_name = body.get("name")
        if body_name in site_info:
            # Get site details for the current body
            details = site_info[body_name]
            # Create the site element and add it to the current body
            ET.SubElement(body, "site", name=f"site_{body_name}", **details)
            logger.info(f"Added site to body: {body_name}")

    # Write the modified tree to the output file
    tree.write(xml_file_path, encoding="utf-8", xml_declaration=True)

    # Call xmllint to prettify the XML file
    subprocess.run(["xmllint", "--format", xml_file_path, "--output", xml_file_path])

def apply_defaults(mjcf_xml_path, defaults_xml_path):
    """Apply default values / options from the given defaults XML file to the given MJCF XML file.

    Args:
        mjcf_xml_path (str): The path to the MJCF XML file.
        defaults_xml_path (str): The path to the defaults XML file.
    """
    # Load the MJCF XML file
    tree = ET.parse(mjcf_xml_path)
    root = tree.getroot()

    # Load the defaults XML file
    defaults_tree = ET.parse(defaults_xml_path)
    defaults_root = defaults_tree.getroot()

    # Apply defaults_root elements as children to root
    # Works with both <mujoco> and <mujocoinclude> root elements
    for i, element in enumerate(defaults_root):
        root.insert(i, element)

    # Save the modified MJCF XML back to the file
    tree.write(mjcf_xml_path, encoding="utf-8", xml_declaration=True)

    # Call xmllint to prettify the XML file
    subprocess.run(["xmllint", "--format", mjcf_xml_path, "--output", mjcf_xml_path])


def merge_xml_files(xml_dict, output_xml_path, model_name):
    """Merges multiple MuJoCo XML files into a single scene file.

    This function combines multiple MuJoCo XML model files into one unified scene file,
    preserving key elements like assets, bodies, actuators, sensors etc. Each model can
    be positioned and articulated differently in the combined scene.

    Args:
        xml_dict (dict): Dictionary mapping XML file paths to positioning attributes.
            Each entry should have:
            - 'pos': tuple of (x,y,z) coordinates
            - 'quat': tuple of (w,x,y,z) quaternion rotation
            - 'articulation_method': str, either 'fixed' or 'free'
                'model1.xml': {'pos': (0,0,0), 'quat': (1,0,0,0), 'articulation_method': 'fixed'},
                'model2.xml': {'pos': (1,1,0), 'quat': (0,1,0,0), 'articulation_method': 'free'}
        output_xml_path (str): File path where the combined XML will be saved.
        model_name (str): Name to be given to the combined model.


    Notes:
        The function merges the following XML elements:

        - compiler, option, default (taken from first file only)
        - asset (merged from all files)
        - worldbody (merged with specified positions and articulations)
        - actuator (merged from all files)
        - sensor (merged from all files if present)
        - tendon (merged from all files if present)
        - contact (merged from all files if present)
    """
    # Parse the first XML file to get the base elements
    first_key = next(iter(xml_dict))
    tree1 = ET.parse(first_key)
    root1 = tree1.getroot()

    # Create a new XML tree for the combined model
    mujoco = ET.Element("mujoco", model=model_name)

    # Combine the <compiler>, <option>, and <default> elements (using the values from the first file)
    for element in root1:
        if element.tag in ["compiler", "option", "default"]:
            mujoco.append(element)

    # Create the <asset> element and merge assets from all files
    asset = ET.SubElement(mujoco, "asset")
    for xml_path in xml_dict.keys():
        tree = ET.parse(xml_path)
        root = tree.getroot()
        if root.find("asset") is not None:
            for element in root.find("asset"):
                asset.append(element)

    # Create the <worldbody> element and merge bodies from all files with specified positions, rotations, and articulation methods
    worldbody = ET.SubElement(mujoco, "worldbody")
    for xml_path, attributes in xml_dict.items():
        tree = ET.parse(xml_path)
        root = tree.getroot()
        if root.find("worldbody") is not None:
            body_elements = root.find("worldbody").findall("body")
            if body_elements:
                first_body = body_elements[0]
                first_body.set("pos", attributes["pos"])
                first_body.set("quat", attributes["quat"])
                if attributes["articulation_method"] == "free":
                    ET.SubElement(first_body, "joint", type="free")
                # Append the modified first body and all other bodies to worldbody
                for body in body_elements:
                    worldbody.append(body)

    # Create the <actuator> element and merge actuators from all files
    actuator = ET.SubElement(mujoco, "actuator")
    for xml_path in xml_dict.keys():
        tree = ET.parse(xml_path)
        root = tree.getroot()
        if root.find("actuator") is not None:
            for element in root.find("actuator"):
                actuator.append(element)

    # Create the <sensor> element and merge sensors from all files (if any)
    sensor = None
    for xml_path in xml_dict.keys():
        tree = ET.parse(xml_path)
        root = tree.getroot()
        if root.find("sensor") is not None:
            if sensor is None:
                sensor = ET.SubElement(mujoco, "sensor")
            for element in root.find("sensor"):
                sensor.append(element)

    # Create the <tendon> element and merge tendons from all files (if any)
    tendon = None
    for xml_path in xml_dict.keys():
        tree = ET.parse(xml_path)
        root = tree.getroot()
        if root.find("tendon") is not None:
            if tendon is None:
                tendon = ET.SubElement(mujoco, "tendon")
            for element in root.find("tendon"):
                tendon.append(element)

    # Create the <contact> element and merge contacts from all files (if any)
    contact = None
    for xml_path in xml_dict.keys():
        tree = ET.parse(xml_path)
        root = tree.getroot()
        if root.find("contact") is not None:
            if contact is None:
                contact = ET.SubElement(mujoco, "contact")
            for element in root.find("contact"):
                contact.append(element)

    # Write the combined XML to the output file
    tree = ET.ElementTree(mujoco)
    tree.write(output_xml_path, encoding="utf-8", xml_declaration=True)

    # Format the output XML file
    subprocess.run(
        ["xmllint", "--format", output_xml_path, "--output", output_xml_path]
    )


def articulate(parent_xml_path, child_xml_path, link_name, output_xml_path, pos="0 0 0", quat="1 0 0 0"):
    """
    Articulates a child model to a parent model at the specified link.
    Uses visual, option, default and size settings from the child model.
    Handles mesh paths considering both explicit paths and meshdir compiler settings.

    Args:
        parent_xml_path (str): Path to the parent MJCF XML file.
        child_xml_path (str): Path to the child MJCF XML file.
        link_name (str): Name of the link in the parent model to which the child model should be articulated.
        output_xml_path (str): Path to the output articulated MJCF XML file.
        pos (str): Position of the child model.
        quat (str): Quaternion orientation of the child model.
    """
    # Convert all paths to absolute for resolution
    parent_xml_path = os.path.abspath(parent_xml_path)
    child_xml_path = os.path.abspath(child_xml_path)
    output_xml_path = os.path.abspath(output_xml_path)

    parent_dir = os.path.dirname(parent_xml_path)
    child_dir = os.path.dirname(child_xml_path)
    output_dir = os.path.dirname(output_xml_path)

    # Load XML files
    parent_tree = ET.parse(parent_xml_path)
    parent_root = parent_tree.getroot()

    child_tree = ET.parse(child_xml_path)
    child_root = child_tree.getroot()

    # Get model names and set combined name
    parent_name = parent_root.get("model", "parent")
    child_name = child_root.get("model", "child")
    parent_root.set("model", f"{parent_name}_{child_name}")

    # Replace/copy configuration elements from child model
    for element_name in ["visual", "option", "default", "size"]:
        child_element = child_root.find(element_name)
        if child_element is not None:
            # Remove existing element from parent if it exists
            parent_element = parent_root.find(element_name)
            if parent_element is not None:
                parent_root.remove(parent_element)
            # Insert child element at beginning of parent
            parent_root.insert(0, child_element)

    # Extract meshdir settings
    def get_meshdir(root, base_dir):
        compiler = root.find(".//compiler")
        if compiler is not None and "meshdir" in compiler.attrib:
            meshdir = compiler.get("meshdir")
            # Convert meshdir to absolute path
            return os.path.normpath(os.path.join(base_dir, meshdir))
        return base_dir

    parent_meshdir = get_meshdir(parent_root, parent_dir)
    child_meshdir = get_meshdir(child_root, child_dir)

    # Find required elements for articulation
    parent_worldbody = parent_root.find("worldbody")
    parent_link = parent_worldbody.find(f'.//body[@name="{link_name}"]')
    if parent_link is None:
        raise ValueError(f"Link {link_name} not found in parent model")

    child_worldbody = child_root.find("worldbody")
    child_root_body = child_worldbody.find("body")
    if child_root_body is None:
        raise ValueError("No root body found in child model")

    # Set position and orientation of child root body
    child_root_body.set("pos", pos)
    child_root_body.set("quat", quat)

    # Function to resolve and update mesh paths
    def update_mesh_paths(asset_elem, meshdir):
        if asset_elem is None:
            return
        mesh_elements = asset_elem.findall("mesh")
        for mesh in mesh_elements:
            if "file" in mesh.attrib:
                # Get absolute path considering meshdir
                mesh_file = mesh.get("file")
                original_mesh_path = os.path.join(meshdir, mesh_file)
                abs_mesh_path = os.path.normpath(original_mesh_path)
                # Convert to relative path from output XML
                rel_mesh_path = os.path.relpath(abs_mesh_path, output_dir)
                mesh.set("file", rel_mesh_path)
                logger.info(f"Updated mesh path: {mesh_file} -> {abs_mesh_path} -> {rel_mesh_path}")

    # Update mesh paths in both parent and child assets
    parent_asset = parent_root.find("asset")
    child_asset = child_root.find("asset")
    update_mesh_paths(parent_asset, parent_meshdir)
    update_mesh_paths(child_asset, child_meshdir)

    # Update or create compiler element with new meshdir
    compiler = parent_root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler")
        parent_root.insert(0, compiler)
    compiler.set("meshdir", "")

    # Append child root body to parent link
    parent_link.append(child_root_body)

    # Merge other elements from child to parent
    for element_name in ["asset", "actuator", "sensor", "tendon", "contact"]:
        child_element = child_root.find(element_name)
        if child_element is not None:
            parent_element = parent_root.find(element_name)
            if parent_element is None:
                parent_element = ET.SubElement(parent_root, element_name)

            # Get existing names to avoid duplicates
            parent_item_names = {item.get("name") for item in parent_element}

            for item in child_element:
                item_name = item.get("name")
                if item_name not in parent_item_names:
                    parent_element.append(item)

    # Write the modified parent XML to output
    parent_tree.write(output_xml_path, encoding="utf-8", xml_declaration=True)

    # Format the output XML file
    subprocess.run(["xmllint", "--format", output_xml_path, "--output", output_xml_path])


def add_trunk_body(xml_file_path, name=None, pos="0 0 0", quat="1 0 0 0"):
    """Adds a top-level "trunk" element below the "worldbody" element.

    Makes every original child of "worldbody" a child of "trunk" instead.

    Args:
        xml_file_path: Path to the input/output MJCF XML file.
        name: Name of the "trunk" body. Defaults to f"{model_name}_trunk".
        pos: Position of the "trunk" body. Defaults to "0 0 0".
        quat: Quaternion orientation of the "trunk" body. Defaults to "1 0 0 0".
    """
    # Parse the XML file
    tree = ET.parse(xml_file_path)
    root = tree.getroot()

    # Find the "worldbody" element
    worldbody = root.find("worldbody")

    if worldbody is None:
        raise ValueError(
            "The provided MJCF file does not contain a <worldbody> element."
        )

    # Create the "trunk" element
    model_name = root.get("model")
    if name is None:
        name = f"{model_name}_trunk"
    trunk = ET.Element("body", name=name, pos=pos, quat=quat)

    # Move all existing children of "worldbody" to be children of "trunk"
    for child in list(worldbody):
        trunk.append(child)

    # Clear the original children of "worldbody"
    worldbody.clear()

    # Insert the "trunk" element into "worldbody"
    worldbody.append(trunk)

    # Write the modified tree to the output file
    tree.write(xml_file_path, encoding="utf-8", xml_declaration=True)

    subprocess.run(["xmllint", "--format", xml_file_path, "--output", xml_file_path])


def exclude_self_collisions(xml_file_path, top_level_body_1=None, top_level_body_2=None, allowed_collision_pairs=[], additional_xml_file_path=None):
    """Excludes self-collisions pairwise between bodies under specified top-level bodies.

    When one of the top-level bodies is None, excludes self-collisions within that top-
    level body. When both are None, excludes self-collisions within the first body
    directly under "worldbody".

    Args:
        xml_file_path: Path to the input/output MJCF XML file.
        top_level_body_1: Name of the first top-level body.
        top_level_body_2: Name of the second top-level body.
        allowed_collision_pairs: List of tuples of body names that should not be
            excluded from collisions. Each body name can be specified as a regular
            expression.
        additional_xml_file_path: Path to an additional XML file. Useful for dealing
            with bodies included by `mujocoinclude`. When specified, the second top-
            level body will be read from this additional XML file.
    """
    # Parse the XML file
    tree = ET.parse(xml_file_path)
    root = tree.getroot()

    # Find the "worldbody" element
    worldbody = root.find("worldbody")

    if worldbody is None:
        raise ValueError(
            "The provided MJCF file does not contain a <worldbody> element."
        )

    # Load the additional XML file if specified
    if additional_xml_file_path is not None:
        tree2 = ET.parse(additional_xml_file_path)
        root2 = tree2.getroot()

    # Collect pairs to exclude
    pairs = []

    if top_level_body_1 is None and top_level_body_2 is None:
        top_level_body_1 = worldbody.find("body").get("name")
    if top_level_body_1 is None:
        top_level_body_1 = top_level_body_2
    elif top_level_body_2 is None:
        top_level_body_2 = top_level_body_1

    tlb1_elem = worldbody.find(f'.//body[@name="{top_level_body_1}"]')
    if additional_xml_file_path is None:
        tlb2_elem = worldbody.find(f'.//body[@name="{top_level_body_2}"]')
    else:
        tlb2_elem = root2.find(f'.//body[@name="{top_level_body_2}"]')

    for body1_elem in [tlb1_elem] + list(tlb1_elem.findall(".//body")):
        for body2_elem in [tlb2_elem] + list(tlb2_elem.findall(".//body")):
            body1_name = body1_elem.get("name")
            body2_name = body2_elem.get("name")
            if body1_name != body2_name and (body2_name, body1_name) not in pairs:
                if any(
                    re.match(body1_re, body1_name) and re.match(body2_re, body2_name)
                    for body1_re, body2_re in allowed_collision_pairs
                ) or any(
                    re.match(body2_re, body1_name) and re.match(body1_re, body2_name)
                    for body1_re, body2_re in allowed_collision_pairs
                ):
                    pass
                else:
                    pairs.append((body1_name, body2_name))

    # Create the "contact" element if it does not exist
    contact = root.find("contact")
    if contact is None:
        contact = ET.SubElement(root, "contact")

    # Iterate over all pairs
    for (body1_name, body2_name) in pairs:
        ET.SubElement(contact, "exclude", body1=body1_name, body2=body2_name)

    # Write the modified tree to the output file
    tree.write(xml_file_path, encoding="utf-8", xml_declaration=True)

    subprocess.run(["xmllint", "--format", xml_file_path, "--output", xml_file_path])


def add_ground(xml_file_path, ground_name='ground', pos="0 0 0", size="1 1 0.1", rgba="0.8 0.8 1 1"):
    """Adds a ground plane to the MJCF XML file.

    Args:
        xml_file_path: Path to the input/output MJCF XML file.
        ground_name: Name of the ground plane geom. Defaults to 'ground'.
        pos: Position of the ground plane. Defaults to "0 0 0".
        size: Size of the ground plane. Defaults to "1 1 0.1".
        rgba: RGBA color of the ground plane. Defaults to "0.8 0.8 1 1".
    """
    # Parse the XML file
    tree = ET.parse(xml_file_path)
    root = tree.getroot()

    # Find the "worldbody" element
    worldbody = root.find("worldbody")

    if worldbody is None:
        raise ValueError(
            "The provided MJCF file does not contain a <worldbody> element."
        )

    # Create the ground plane geom element
    ground_geom = ET.SubElement(
        worldbody,
        "geom",
        {"name": ground_name, "type": "plane", "pos": pos, "size": size, "rgba": rgba},
    )

    # Write the modified tree to the output file
    tree.write(xml_file_path, encoding="utf-8", xml_declaration=True)

    # Format the output XML file
    subprocess.run(["xmllint", "--format", xml_file_path, "--output", xml_file_path])


def extract_part(xml_file_path, output_path, part_name, body_name=None):
    """Extract a part from the MJCF XML file into a <mujocoinclude> file.

    Args:
        xml_file_path (str): Path to the input MJCF XML file.
        output_path (str): Path to the output <mujocoinclude> file.
        part_name (str): Name of the part to extract. Can be "asset", "body", "actuator", "sensor", "tendon", or "contact". When "body" is specified, the body_name argument must be provided.
        body_name (str or None): Name of the body to extract when part_name is "body".

    Returns:
        None
    """

    # Parse the XML file
    tree = ET.parse(xml_file_path)
    root = tree.getroot()

    # Find the part element
    if part_name in ["asset", "actuator", "sensor", "tendon", "contact"]:
        part = root.find(part_name)
    elif part_name == "body":
        if body_name is None:
            raise ValueError(
                "The body_name argument must be provided when part_name is 'body'."
            )
        part = root.find(f'.//body[@name="{body_name}"]')
    else:
        raise ValueError(
            "The part_name argument must be one of 'asset', 'body', 'actuator', 'sensor', 'tendon', or 'contact'."
        )

    if part is None:
        raise ValueError(
            f"The provided MJCF file does not contain a <{part_name}> element."
        )

    # Wrap the part with a <mujocoinclude> root
    mujoco_include = ET.Element('mujocoinclude')
    mujoco_include.append(part)

    # Create a new tree with the <mujocoinclude> as the root
    new_tree = ET.ElementTree(mujoco_include)

    # Write the new tree to the output file
    new_tree.write(output_path, encoding='utf-8', xml_declaration=True)

    # Format the output XML file
    subprocess.run(["xmllint", "--format", output_path, "--output", output_path])

def add_mesh_prefix(xml_file_path, prefix):
    """Add a prefix to all mesh filenames in the MJCF XML file.

    Args:
        xml_file_path (str): Path to the input/output MJCF XML file.
        prefix (str): Prefix to add to all mesh filenames.

    Returns:
        None
    """
    # Parse the XML file
    tree = ET.parse(xml_file_path)
    root = tree.getroot()

    # Find the asset element
    asset = root.find("asset")

    if asset is None:
        raise ValueError(
            "The provided MJCF file does not contain an <asset> element."
        )

    # Iterate over all mesh elements
    for mesh in asset.findall("mesh"):
        if mesh.get("file") is not None:
            mesh.set("file", f"{prefix}/{mesh.get('file')}")

    # Write the modified tree to the output file
    tree.write(xml_file_path, encoding="utf-8", xml_declaration=True)

    subprocess.run(["xmllint", "--format", xml_file_path, "--output", xml_file_path])

def update_geom_collisions(xml_file_path, collision_yaml_path):
    """Update collision properties of an MJCF XML file based on a YAML configuration.

    This function:
    1. Sets all existing geoms to non-collidable if they don't have collision properties already set
    2. Adds collidable geoms specified in the YAML file

    Args:
        xml_file_path (str): Path to the MJCF XML file
        collision_yaml_path (str): Path to YAML file containing collidable geom specifications
    """
    # Parse the XML file
    tree = ET.parse(xml_file_path)
    root = tree.getroot()

    # Load collision specifications from YAML
    with open(collision_yaml_path, 'r') as f:
        collision_specs = yaml.safe_load(f)

    # Set all existing geoms to non-collidable if they don't have collision properties
    for geom in root.findall(".//body//geom"):
        if not any(attr in geom.attrib for attr in ['contype', 'conaffinity', 'group']):
            geom.set('contype', '0')
            geom.set('conaffinity', '0')
            geom.set('group', '1')
            geom.set('density', '0')

    # Add collidable geoms from YAML
    for body_name, geom_spec in collision_specs.items():
        # Find the body element
        body = root.find(f'.//body[@name="{body_name}"]')
        if body is None:
            logger.warning(f"Body {body_name} not found in XML")
            continue

        # Create geom attributes
        geom_attrs = {
            'name': f'col_{body_name}',
            'type': geom_spec['type'],
            'contype': '1',
            'conaffinity': '1',
            'group': '3'  # Using group 3 for collision geoms as in original XML
        }

        # Add type-specific attributes
        if geom_spec['type'] == 'box':
            geom_attrs['size'] = ' '.join(map(str, geom_spec['size']))
            geom_attrs['pos'] = ' '.join(map(str, geom_spec['pos']))
        elif geom_spec['type'] == 'capsule':
            geom_attrs['size'] = ' '.join(map(str, geom_spec['size']))
            geom_attrs['fromto'] = ' '.join(map(str, geom_spec['fromto']))

        # Create and add the geom element
        collision_geom = ET.SubElement(body, 'geom', geom_attrs)

    # Save the modified XML
    tree.write(xml_file_path, encoding='utf-8', xml_declaration=True)

    # Format the XML file
    subprocess.run(['xmllint', '--format', xml_file_path, '--output', xml_file_path])

    logger.info(f"Updated collision properties in {xml_file_path}")
