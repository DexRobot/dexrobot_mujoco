#!/usr/bin/env python3

import os
import argparse
from dexrobot_mujoco.utils.mjcf_utils import extract_part, get_body_names
from loguru import logger


def extract_parts(xml_path, output_dir):
    """Extract all parts from an MJCF XML file into separate mujocoinclude files.

    Args:
        xml_path (str): Path to input MJCF XML file
        output_dir (str): Directory to save extracted parts
    """
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)

    # Get base name of input file without extension
    base_name = os.path.splitext(os.path.basename(xml_path))[0]

    # Parts to extract and their corresponding output filenames
    parts = {
        "asset": f"{base_name}_asset.xml",
        "actuator": f"{base_name}_actuator.xml",
        "sensor": f"{base_name}_sensor.xml",
        "contact": f"{base_name}_contact.xml",
    }

    # Extract each part
    for part_name, output_file in parts.items():
        output_path = os.path.join(output_dir, output_file)
        try:
            extract_part(xml_path, output_path, part_name)
            logger.info(f"Extracted {part_name} to {output_path}")
        except ValueError as e:
            logger.warning(f"Could not extract {part_name}: {str(e)}")

    # Special handling for body - extract first body under worldbody
    try:
        body_name = get_body_names(xml_path)[0]
        body_path = os.path.join(output_dir, f"{base_name}_body.xml")
        # Will raise ValueError if no body is found
        extract_part(xml_path, body_path, "body", body_name=body_name)
        logger.info(f"Extracted body to {body_path}")
    except ValueError as e:
        logger.warning(f"Could not extract body: {str(e)}")


def main():
    parser = argparse.ArgumentParser(description="Extract parts from MJCF XML file")
    parser.add_argument("xml_path", help="Path to input MJCF XML file")
    parser.add_argument(
        "--output-dir",
        "-o",
        default="parts",
        help="Output directory for extracted parts (default: parts)",
    )

    args = parser.parse_args()

    if not os.path.exists(args.xml_path):
        logger.error(f"Input file {args.xml_path} does not exist")
        return 1

    try:
        extract_parts(args.xml_path, args.output_dir)
    except Exception as e:
        logger.error(f"Error extracting parts: {str(e)}")
        return 1

    return 0


if __name__ == "__main__":
    exit(main())
