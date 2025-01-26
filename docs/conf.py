# Configuration file for the Sphinx documentation builder.
import os
import sys
sys.path.insert(0, os.path.abspath('..'))

project = "dexrobot_mujoco"
copyright = "2025, DexRobot"
author = "DexRobot"
release = "0.1.0"

# -- General configuration ---------------------------------------------------

extensions = [
    'sphinx.ext.autodoc',  # Core autodoc functionality
    'sphinx.ext.napoleon',  # Support for Google/NumPy style docstrings
    'sphinx.ext.viewcode',  # Add links to source code
    'sphinx.ext.intersphinx',  # Link to other project's documentation
    'sphinx_autodoc_typehints',  # Better type hints support
]

# Mock imports for dependencies that might not be installed during doc building
autodoc_mock_imports = [
    "rclpy",
    "std_msgs",
    "std_srvs",
    "sensor_msgs",
    "geometry_msgs",
    "mujoco",
    "cv2",
    "numpy",
    "scipy",
    "loguru",
    "flask",
    "pandas",
    "yaml"
]

templates_path = ["_templates"]
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]

# -- HTML output options ---------------------------------------------------
html_theme = "sphinx_rtd_theme"

# Napoleon settings
napoleon_google_docstring = True
napoleon_numpy_docstring = True
napoleon_include_init_with_doc = True
napoleon_include_private_with_doc = False
napoleon_include_special_with_doc = True
napoleon_use_admonition_for_examples = True
napoleon_use_admonition_for_notes = True
napoleon_use_rtype = True

# Autodoc settings
autodoc_default_options = {
    'members': True,
    'member-order': 'bysource',
    'special-members': '__init__',
    'undoc-members': True,
    'exclude-members': '__weakref__'
}

# Type hints settings
always_document_param_types = True
typehints_fully_qualified = False
