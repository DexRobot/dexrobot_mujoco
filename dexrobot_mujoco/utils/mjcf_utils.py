import mujoco
import os
import xml.etree.ElementTree as ET
import re
import subprocess
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


def urdf2mjcf(urdf_path, mjcf_dir, mesh_dir):
    """Load a URDF file and save it to an MJCF XML file.

    Args:
        urdf_path (str): The path to the URDF file.
        mjcf_dir (str): The directory to save the output MJCF file.
        mesh_dir (str): The directory containing the mesh files.
    """
    m = mujoco.MjModel.from_xml_path(urdf_path, load_meshes(mesh_dir))
    mujoco.mj_saveLastXML(
        f"{mjcf_dir}/{os.path.splitext(os.path.basename(urdf_path))[0]}.xml", m
    )


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


def add_touch_sensor_sites(xml_file_path):
    """
    Add touch sensor sites to specific bodies in the MJCF XML file based on body names.

    Args:
        xml_file_path (str): Path to the input/output MJCF XML file.

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

    # Define the specific site information for each body
    site_info = {
        "r_f_link1_3": {"pos": "-0.009 0.03 0", "size": "0.01", "type": "sphere"},
        "r_f_link2_4": {"pos": "0.025 0.003 0", "size": "0.01", "type": "sphere"},
        "r_f_link3_4": {"pos": "0.025 0.003 0", "size": "0.01", "type": "sphere"},
        "r_f_link4_4": {"pos": "0.025 0.003 0", "size": "0.01", "type": "sphere"},
        "r_f_link5_4": {"pos": "0.025 0.003 0", "size": "0.01", "type": "sphere"},
    }

    # Iterate through all bodies in the worldbody to find specific bodies and add sites
    logger.warning("Adding touch sensor sites to specific bodies in the MJCF XML file.")
    for body in root.findall(".//body"):
        body_name = body.get("name")
        if body_name in site_info:
            # Get site details for the current body
            details = site_info[body_name]
            # Create the site element and add it to the current body
            ET.SubElement(body, "site", name=f"site_{body_name}", pos=details["pos"], size=details["size"], type=details["type"])
            logger.warning(f"Added touch sensor site to body: {body_name}")

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

    # Apply defaults_root as a child to root
    for i, element in enumerate(defaults_root):
        root.insert(i, element)

    # Save the modified MJCF XML back to the file
    tree.write(mjcf_xml_path, encoding="utf-8", xml_declaration=True)

    # Call xmllint to prettify the XML file
    subprocess.run(["xmllint", "--format", mjcf_xml_path, "--output", mjcf_xml_path])


def merge_xml_files(xml_dict, output_xml_path, model_name):
    """
    Merges multiple MuJoCo XML files into a single scene file.

    Parameters:
    xml_dict (dict): A dictionary mapping XML file paths to a dictionary containing 'pos', 'quat', and 'articulation_method'.
                     Example:
                     {
                         'model1.xml': {'pos': (0, 0, 0), 'quat': (1, 0, 0, 0), 'articulation_method': 'fixed'},
                         'model2.xml': {'pos': (1, 1, 0), 'quat': (0, 1, 0, 0), 'articulation_method': 'free'},
                         ...
                     }
    output_xml_path (str): Path to the output combined XML file.
    model_name (str): Name of the combined model.

    Returns:
    None
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

    Note: we assume that the child model has a single root body. (first body under <worldbody>)

    Parameters:
    parent_xml_path (str): Path to the parent MJCF XML file.
    child_xml_path (str): Path to the child MJCF XML file.
    link_name (str): Name of the link in the parent model to which the child model should be articulated.
    output_xml_path (str): Path to the output articulated MJCF XML file.
    pos (str): Position of the child model.
    quat (str): Quaternion orientation of the child model.

    Returns:
    None
    """
    # Load the parent XML file and locate the specified link
    parent_tree = ET.parse(parent_xml_path)
    parent_root = parent_tree.getroot()
    parent_worldbody = parent_root.find("worldbody")
    parent_link = parent_worldbody.find(f'.//body[@name="{link_name}"]')

    # Load the child XML file and locate the root body
    child_tree = ET.parse(child_xml_path)
    child_root = child_tree.getroot()
    child_worldbody = child_root.find("worldbody")
    child_root_body = child_worldbody.find("body")

    # Override the position and orientation of the child root body
    child_root_body.set("pos", pos)
    child_root_body.set("quat", quat)

    # Append the child root body to the parent link
    parent_link.append(child_root_body)

    # Merge the assets, actuators, sensors, tendons, and contacts from the child model to the parent model (when name duplicates, skip)
    for element_name in ["asset", "actuator", "sensor", "tendon", "contact"]:
        if child_root.find(element_name) is not None:
            if parent_root.find(element_name) is None:
                parent_root.append(ET.Element(element_name))
            child_element = child_root.find(element_name)
            parent_element = parent_root.find(element_name)
            parent_item_names = {item.get("name") for item in parent_element}
            for item in child_element:
                item_name = item.get("name")
                if item_name not in parent_item_names:
                    parent_element.append(item)

    # Write the modified parent XML to the output file
    parent_tree.write(output_xml_path, encoding="utf-8", xml_declaration=True)

    # Format the output XML file
    subprocess.run(
        ["xmllint", "--format", output_xml_path, "--output", output_xml_path]
    )


def add_trunk_body(xml_file_path, pos="0 0 0", quat="1 0 0 0"):
    """
    Adds a top-level "trunk" element below the "worldbody" and makes every original child of "worldbody" a child of "trunk".

    Parameters:
    xml_file_path (str): Path to the input/output MJCF XML file.
    pos (str): Position of the "trunk" body.
    quat (str): Quaternion orientation of the "trunk" body.

    Returns:
    None
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
    trunk = ET.Element("body", name=f"{model_name}_trunk", pos=pos, quat=quat)

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
    """
    Excludes self-collisions pairwise between bodies under top_level_body_1 and top_level_body_2 in the MJCF XML file. When one of the top-level bodies is None, it excludes self-collisions within this top-level body. When both are None, it excludes self-collisions within the first body directly under "worldbody".

    Parameters:
    xml_file_path (str): Path to the input/output MJCF XML file.
    top_level_body_1 (str or None): Name of the first top-level body.
    top_level_body_2 (str or None): Name of the second top-level body.
    allowed_collision_pairs (list): List of tuples of body names that should not be excluded from collisions. Each body name can be specified as a regular expression.
    additional_xml_file_path (str or None): Path to an additional XML file. Useful for dealing with bodies included by `mujocoinclude`. When specified, the second top-level body will be read from this additional XML file.

    Returns:
    None
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
    """
    Adds a ground plane to the MJCF XML file.

    Parameters:
    xml_file_path (str): Path to the input/output MJCF XML file.
    ground_name (str): Name of the ground plane geom.
    pos (str): Position of the ground plane.
    size (str): Size of the ground plane.
    rgba (str): RGBA color of the ground plane.

    Returns:
    None
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
