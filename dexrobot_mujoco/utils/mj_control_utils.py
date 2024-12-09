import time
import re
import yaml
import mujoco, mujoco.viewer
import numpy as np
import sys
import os
from loguru import logger

from .mjcf_utils import load_meshes


class MJControlWrapper:
    """Wrapper for communicating with Mujoco simulator."""

    def __init__(self, model_path, mesh_dir=None, renderer_dimension=None, seed=0):
        """Initialize the Mujoco control wrapper.

        Args:
            model_path (str): The path to the Mujoco model XML file.
            mesh_dir (str): The path to the directory containing the mesh files. Use None for default mesh relative to XML files.
            renderer_dimension (tuple or None): (width, height) of renderer, if image rendering is needed. When None, rendering is disabled.
        """
        model_path = os.path.abspath(model_path)
        if mesh_dir is None:
            self.model = mujoco.MjModel.from_xml_path(model_path)
        else:
            meshes = load_meshes(mesh_dir)
            self.model = mujoco.MjModel.from_xml_path(model_path, meshes)
        self.data = mujoco.MjData(self.model)
        self.viewer = None
        self.camera = mujoco.MjvCamera()
        self.shadow = False  # whether to reflect
        self.renderer_dimension = renderer_dimension
        if renderer_dimension is not None:
            width, height = renderer_dimension
            self.camera_renderer = mujoco.Renderer(
                self.model, width=width, height=height
            )
        else:
            self.camera_renderer = None
        self.num_actuators = len(self.model.actuator_actnum)
        self.actuator_names = [
            self.model.actuator(j).name for j in range(self.num_actuators)
        ]
        self.config = {}
        self.seed = seed
        np.random.seed(seed)

    def enable_infinite_rotation(self, act_name_re):
        """Modify the model to allow infinite rotation for the specified joint actuators.

        Args:
            act_name_re (str): The regular expression for the actuator names that should allow infinite rotation.
        """
        for act_name in self.actuator_names:
            if re.match(act_name_re, act_name):
                actuator_index = self.actuator_names.index(act_name)
                self.model.actuator_ctrlrange[actuator_index] = [-100, 100]

    def launch_viewer(self, viewer_type, show_ui=True):
        """Launch the Mujoco viewer.

        Args:
            viewer_type (str): The type of the viewer to launch. Can be "active" or "passive".
            show_ui (bool): Whether to show the left and right UI in the viewer.
        """
        if viewer_type == "active":
            self.viewer = mujoco.viewer.launch(self.model, self.data)
        elif viewer_type == "passive":
            self.viewer = mujoco.viewer.launch_passive(
                self.model,
                self.data,
                key_callback=lambda key: self.key_callback(key),
                show_left_ui=show_ui,
                show_right_ui=show_ui,
            )
            self.viewer.cam.distance = self.camera.distance
            self.viewer.cam.elevation = self.camera.elevation
            self.viewer.cam.azimuth = self.camera.azimuth
            self.viewer.cam.lookat[:] = self.camera.lookat

            with self.viewer.lock():
                self.viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0
                self.viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
        else:
            raise ValueError(f"Invalid viewer type: {viewer_type}")

    def step(self, until=None):
        """Step the simulation.

        Args:
            until (float or None): The time to step the simulation until (in terms of simulation time). If None, only step once.
        """
        if until is not None:
            while self.data.time < until:
                mujoco.mj_step(self.model, self.data)
        else:
            mujoco.mj_step(self.model, self.data)
        if self.viewer is not None:
            self.viewer.sync()

    def send_control(self, act_name, value):
        """Send control signal to the specified actuator.

        Args:
            act_name (str): The name of the actuator.
            value (float): The control signal value.
        """
        if act_name in self.actuator_names:
            actuator_index = self.actuator_names.index(act_name)

            # Get the joint ID associated with this actuator
            joint_id = self.model.actuator_trnid[actuator_index, 0]

            # Check if the joint is revolute
            if self.model.jnt_type[joint_id] == mujoco.mjtJoint.mjJNT_HINGE:
                ctrl_range = self.model.actuator_ctrlrange[actuator_index]
                lower_limit, upper_limit = ctrl_range
                middle = (lower_limit + upper_limit) / 2
                wrapped_diff = ((value - middle + np.pi) % (2 * np.pi)) - np.pi
                value = middle + wrapped_diff
                value = np.clip(value, lower_limit, upper_limit)

            self.data.ctrl[actuator_index] = value
        else:
            raise ValueError(f"Actuator {act_name} not found in model.")

    def get_joint_id(self, joint_name):
        """Get the ID of the specified joint.

        Args:
            joint_name (str): The name of the joint.

        Returns:
            int: The joint ID.
        """
        return mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)

    def get_link_id(self, link_name):
        """Get the ID of the specified link.

        Args:
            link_name (str): The name of the link.

        Returns:
            int: The link ID.
        """
        return mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, link_name)

    def set_qpos(self, joint_name, value):
        """Set the position of the specified joint.

        Args:
            joint_name (str): The name of the joint.
            value (float): The position value.
        """
        joint_id = self.get_joint_id(joint_name)
        self.data.qpos[joint_id] = value

    def get_qpos(self, joint_name):
        """Get the position of the specified joint.

        Args:
            joint_name (str): The name of the joint.

        Returns:
            float: The position value.
        """
        joint_id = self.get_joint_id(joint_name)
        return self.data.qpos[joint_id]

    def set_site_xpos(self, site_name: str, pos: np.array):
        """Set the position of the specified site.

        Args:
            site_name (str): The name of the site.
            pos (np.array): The position value.
        """
        site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        self.model.site_pos[site_id] = pos

    def get_site_xpos(self, site_name: str):
        """Get the position of the specified site.

        Args:
            site_name (str): The name of the site.
        """
        site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        return self.model.site_pos[site_id]

    def get_link_pose(self, link_name: str):
        """Get the position and rotation of the specified link.

        Args:
            link_name (str): The name of the link.

        Returns:
            np.array: The position of the link.
        """
        link_id = self.get_link_id(link_name)
        pos = self.data.xpos[link_id]
        quat = self.data.xquat[link_id]
        return pos, quat

    def parse_yaml(self, yaml_path):
        """Parse the YAML file containing the configuration.

        Args:
            yaml_path (str): The path to the YAML file.
        """
        with open(yaml_path, "r") as file:
            config = yaml.safe_load(file)
            self.config = config
        self.set_initial_values()

    def set_initial_values(self):
        """Set the initial values of the simulation."""

        if "initial_qpos" in self.config:
            for joint_name, qpos in self.config["initial_qpos"].items():
                try:
                    self.set_qpos(joint_name, qpos)
                except ValueError as e:
                    logger.error(f"Error setting qpos for {joint_name}: {e}")

        if "initial_ctrl" in self.config:
            for act_name, ctrl in self.config["initial_ctrl"].items():
                try:
                    self.send_control(act_name, ctrl)
                except ValueError as e:
                    logger.error(f"Error setting control for {act_name}: {e}")

        if "camera" in self.config:
            self.set_view_angle(
                self.config["camera"]["lookat"],
                self.config["camera"]["distance"],
                self.config["camera"]["elevation"],
                self.config["camera"]["azimuth"],
            )

        # randomize virtual joint pos
        joint_names_to_randomize = [
            item for sublist in self.config.get("randomized_joints", {}) for item in sublist
        ]
        logger.info(f"Randomizing joints: {joint_names_to_randomize}")
        for joint_name in joint_names_to_randomize:
            joint_id = self.get_joint_id(joint_name)
            lower, upper = self.model.jnt_range[joint_id]
            desired_pos = np.random.rand() * (upper - lower) + lower
            self.set_qpos(joint_name, desired_pos)

    def set_view_angle(self, lookat, distance, elevation, azimuth):
        """Set the view angle of the camera.

        Args:
            lookat (list): The lookat point [x, y, z].
            distance (float): The distance from the camera to the lookat point.
            elevation (float): The elevation angle in degrees.
            azimuth (float): The azimuth angle in degrees.
        """
        self.camera.lookat[:] = lookat
        self.camera.distance = distance
        self.camera.elevation = elevation
        self.camera.azimuth = azimuth

    def render_frame(self):
        """Render a frame using the camera."""
        if self.camera_renderer is not None:
            self.camera_renderer.update_scene(self.data, camera=self.camera)
            return self.camera_renderer.render()
        else:
            return None

    def key_callback(self, keycode):
        """Callback function for handling key presses in the viewer.

        Args:
            keycode (int): The key code of the pressed key.
        """
        if chr(keycode) == "R":
            logger.warning("Resetting simulation...")
            self.reset_simulation()
        elif chr(keycode) == "Q":
            logger.warning("Quitting viewer...")
            self.viewer.close()
        elif chr(keycode) == "S":
            # set viewer visualization options
            self.shadow = not self.shadow
            with self.viewer.lock():
                self.viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = (
                    self.shadow
                )
            logger.warning(f"Setting reflection to {self.shadow}")
        elif chr(keycode) == "C":
            logger.info(
                f"Camera position: {self.viewer.cam.lookat}, distance: {self.viewer.cam.distance}, elevation: {self.viewer.cam.elevation}, azimuth: {self.viewer.cam.azimuth}"
            )

    def reset_simulation(self):
        """Reset the simulation."""
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        time.sleep(0.01)
        self.set_initial_values()
        time.sleep(0.01)
