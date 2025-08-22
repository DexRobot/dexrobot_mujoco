#!/usr/bin/env python3
"""
Standalone demo for TaShan touch sensors without ROS dependencies.

This example demonstrates how to use TaShan sensors with MuJoCo directly,
useful for testing or when ROS integration is not needed.

Requirements:
- Python 3.8
- MuJoCo 3.2.3 (pip install mujoco==3.2.3)
- TaShan sensor library in dexrobot_mujoco/ts_sensor_lib/
- Optional: rerun-sdk for visualization (pip install rerun-sdk)
"""

import time
import numpy as np
import mujoco
from mujoco import viewer
import os
import sys
import argparse
from loguru import logger

# Add dexrobot_mujoco to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from dexrobot_mujoco.wrapper import MJSimWrapper, TSSensorManager
from enum import Enum, auto

# Required rerun import with version check
try:
    import rerun as rr
except (ImportError, TypeError) as e:
    if "TypeError" in str(e) or "not subscriptable" in str(e):
        logger.error("Failed to import rerun due to compatibility issues.")
        logger.error("This demo requires Python 3.8 (for TaShan sensors) and rerun 0.18.x")
        logger.error("Your rerun version is likely too new for Python 3.8.")
        logger.error("Please install the compatible version:")
        logger.error("  pip uninstall rerun-sdk")
        logger.error("  pip install rerun-sdk==0.18.2")
    else:
        logger.error("Rerun is required but not installed.")
        logger.error("Install with: pip install rerun-sdk==0.18.2")
    raise RuntimeError("Failed to import rerun visualization library")

# Check rerun version after successful import
try:
    import importlib.metadata
    try:
        rerun_version = importlib.metadata.version('rerun-sdk')
    except:
        # Fallback for older Python versions
        import pkg_resources
        rerun_version = pkg_resources.get_distribution('rerun-sdk').version

    # Parse version and check if it's 0.18.x
    version_parts = rerun_version.split('.')
    if len(version_parts) < 2 or version_parts[0] != '0' or version_parts[1] != '18':
        logger.error(f"Rerun version {rerun_version} is not supported.")
        logger.error("This demo requires rerun version 0.18.x")
        logger.error("Install with: pip install 'rerun-sdk>=0.18.0,<0.19.0'")
        raise RuntimeError(f"Incompatible rerun version {rerun_version}. Required: 0.18.x")

    logger.info(f"Rerun version {rerun_version} detected")

except Exception as e:
    logger.error(f"Failed to check rerun version: {e}")
    raise


class PinchState(Enum):
    """States for pinch gesture FSM."""
    IDLE = auto()
    MOVING_TO_PREPINCH = auto()
    PREPINCH_READY = auto()
    PINCHING = auto()
    HOLDING_PINCH = auto()
    RELEASING = auto()


class TaShanDemo:
    def __init__(self):
        """Initialize the TaShan sensor demo with box scene."""

        # Initialize TS sensor callback before loading model
        if TSSensorManager.initialize_before_model_load():
            logger.info("✓ TaShan sensor callback initialized successfully!")
        else:
            logger.warning("⚠ TaShan sensor callback initialization failed")
            logger.warning("  Make sure you have Python 3.8 and MuJoCo 3.2.3 installed")

        # Load box scene which includes the hand
        scene_path = os.path.join(
            os.path.dirname(__file__),
            "../dexrobot_mujoco/scenes/box.xml"
        )
        if not os.path.exists(scene_path):
            raise FileNotFoundError(f"Box scene not found at {scene_path}")

        self.controller = MJSimWrapper(scene_path)

        # Set camera position
        self.controller.set_view_angle(
            lookat=[-0.03, 0, 0.85],
            distance=1.2,
            elevation=-12,
            azimuth=-90
        )
        logger.info("Set camera position for optimal pinch gesture viewing")

        # Set up TS sensor manager
        self.ts_manager = None
        try:
            self.ts_manager = TSSensorManager(self.controller)
            self.controller.set_sensor_manager(self.ts_manager)
            if self.ts_manager.ts_sensor_available:
                logger.info(f"✓ Found {len(self.ts_manager.ts_sensor_names)} TS sensors in the model")
            else:
                logger.warning("⚠ No TS sensors found in the model")
        except Exception as e:
            logger.error(f"Failed to initialize TS sensor manager: {e}")

        # Initialize rerun
        rr.init("tashan_pinch_demo", spawn=True)
        logger.info("✓ Rerun visualization initialized")

        # Initialize FSM for pinch gesture
        self.pinch_state = PinchState.IDLE
        self.state_timer = 0.0
        self.state_progress = 0.0

    def get_pinch_config(self):
        """Get pinch gesture configurations."""
        return {
            "initial": {
                "ARTx": -0.16,
                "ARTy": -0.05,
                "ARTz": -0.3,
                "r_f_joint1_2": 0.25,
                "r_f_joint1_3": 0.25,
                "r_f_joint1_4": 0.25,
                "r_f_joint2_2": 1.15,
                "r_f_joint2_3": 0.3,
                "r_f_joint2_4": 0.3,
                "r_f_joint1_1": 1.25  # Keep constant during pinch
            },
            "pinch_deltas": {  # Changes to apply during pinch
                "r_f_joint1_2": 0.1,
                "r_f_joint1_3": 0.1,
                "r_f_joint1_4": 0.1,
                "r_f_joint2_2": 0.1,
                "r_f_joint2_3": 0.1,
                "r_f_joint2_4": 0.1
            }
        }

    def update_pinch_fsm(self, dt):
        """Update pinch gesture using finite state machine.

        Args:
            dt: Time step in seconds
        """
        self.state_timer += dt
        config = self.get_pinch_config()

        if self.pinch_state == PinchState.IDLE:
            # Start moving to prepinch position
            self.pinch_state = PinchState.MOVING_TO_PREPINCH
            self.state_timer = 0.0
            self.state_progress = 0.0
            logger.info("Starting pinch gesture sequence")

        elif self.pinch_state == PinchState.MOVING_TO_PREPINCH:
            # Move to initial position
            duration = 2.0  # 2 seconds to reach position
            self.state_progress = min(1.0, self.state_timer / duration)

            # Interpolate to initial position
            for joint_name, target_value in config["initial"].items():
                act_name = f"act_{joint_name}"
                if act_name in self.controller.actuator_names:
                    # Simple linear interpolation from current to target
                    self.controller.send_control(act_name, target_value * self.state_progress)

            if self.state_progress >= 1.0:
                self.pinch_state = PinchState.PREPINCH_READY
                self.state_timer = 0.0
                logger.info("Reached prepinch position")

        elif self.pinch_state == PinchState.PREPINCH_READY:
            # Hold for a moment before pinching
            if self.state_timer >= 1.0:
                self.pinch_state = PinchState.PINCHING
                self.state_timer = 0.0
                self.state_progress = 0.0
                logger.info("Starting pinch motion")

        elif self.pinch_state == PinchState.PINCHING:
            # Execute pinch motion
            duration = 2.0
            self.state_progress = min(1.0, self.state_timer / duration)

            # Apply initial position plus pinch deltas
            for joint_name, base_value in config["initial"].items():
                act_name = f"act_{joint_name}"
                if act_name in self.controller.actuator_names:
                    # Add delta if this joint participates in pinch
                    delta = config["pinch_deltas"].get(joint_name, 0.0)
                    value = base_value + delta * self.state_progress
                    self.controller.send_control(act_name, value)

            if self.state_progress >= 1.0:
                self.pinch_state = PinchState.HOLDING_PINCH
                self.state_timer = 0.0
                logger.info("Pinch complete, holding position")

        elif self.pinch_state == PinchState.HOLDING_PINCH:
            # Hold pinch for a moment
            if self.state_timer >= 1.5:
                self.pinch_state = PinchState.RELEASING
                self.state_timer = 0.0
                self.state_progress = 1.0
                logger.info("Releasing pinch")

        elif self.pinch_state == PinchState.RELEASING:
            # Release pinch
            duration = 2.0
            self.state_progress = max(0.0, 1.0 - self.state_timer / duration)

            # Apply initial position plus diminishing pinch deltas
            for joint_name, base_value in config["initial"].items():
                act_name = f"act_{joint_name}"
                if act_name in self.controller.actuator_names:
                    delta = config["pinch_deltas"].get(joint_name, 0.0)
                    value = base_value + delta * self.state_progress
                    self.controller.send_control(act_name, value)

            if self.state_progress <= 0.0:
                self.pinch_state = PinchState.PREPINCH_READY
                self.state_timer = 0.0
                logger.info("Released, ready for next pinch")

    def print_sensor_data(self):
        """Print formatted sensor data for thumb and index finger TS sensors."""
        if not self.ts_manager or not self.ts_manager.ts_sensor_available:
            return

        print("\n" + "="*60)
        print("TaShan Sensor Data (Thumb & Index):")
        print("="*60)

        try:
            # Get force data for thumb (1) and index finger (2) only
            finger_names = {1: "Thumb", 2: "Index"}

            for sensor_id, finger_name in finger_names.items():
                try:
                    force = self.ts_manager.get_force_data_by_id(sensor_id)
                    print(f"\n{finger_name} (TS-F-A-{sensor_id}):")
                    print(f"  Normal Force:     {force.normal:8.4f} N")
                    print(f"  Tangential Force: {force.tangential_magnitude:8.4f} N")
                    print(f"  Direction:        {force.tangential_direction:8.2f} deg")
                except Exception:
                    # Sensor might not exist in some models
                    pass
        except Exception as e:
            logger.error(f"Error reading sensor data: {e}")

    def update_rerun_visualization(self):
        """Update rerun visualization with thumb and index finger sensor data."""
        if not self.ts_manager or not self.ts_manager.ts_sensor_available:
            return

        try:
            # Only track thumb (1) and index finger (2)
            finger_sensors = {1: "thumb", 2: "index"}
            total_normal = 0.0
            total_tangential = 0.0

            for sensor_id, finger_name in finger_sensors.items():
                try:
                    force = self.ts_manager.get_force_data_by_id(sensor_id)

                    # Log individual force components
                    rr.log(f"sensors/{finger_name}/normal_force",
                          rr.Scalar(force.normal))
                    rr.log(f"sensors/{finger_name}/tangential_force",
                          rr.Scalar(force.tangential_magnitude))
                    # Convert direction to radians for better plot scaling
                    rr.log(f"sensors/{finger_name}/direction_rad",
                          rr.Scalar(np.radians(force.tangential_direction)))

                    # Create a 3D force vector for visualization
                    # Normal force in Z, tangential in XY plane
                    direction_rad = np.radians(force.tangential_direction)
                    force_vec = np.array([
                        force.tangential_magnitude * np.cos(direction_rad),
                        force.tangential_magnitude * np.sin(direction_rad),
                        force.normal
                    ])

                    # Log as 3D arrow (normalized for visualization)
                    if np.linalg.norm(force_vec) > 0.01:  # Only show if force is significant
                        rr.log(f"3d/{finger_name}/force",
                              rr.Arrows3D(origins=[[0, 0, 0]],
                                         vectors=[force_vec * 0.1]))  # Scale for visibility

                    # Accumulate totals
                    total_normal += force.normal
                    total_tangential += force.tangential_magnitude

                except Exception:
                    # Sensor might not exist in some models
                    pass

            # Log combined force magnitudes for thumb and index only
            rr.log("sensors/thumb_index_total/normal_force", rr.Scalar(total_normal))
            rr.log("sensors/thumb_index_total/tangential_force", rr.Scalar(total_tangential))

        except Exception as e:
            logger.error(f"Error updating rerun visualization: {e}")

    def run_demo(self, duration=30.0):
        """Run the demo with pinch gesture and visualization.

        Args:
            duration: Duration to run the demo in seconds
        """
        logger.info(f"Running TaShan sensor demo with box scene for {duration} seconds...")
        logger.info("Pinch gesture will start automatically")
        logger.info("Press 'R' to reset, 'Q' to quit early")
        logger.info("Press Tab/Shift+Tab to show/hide UI panels\n")

        # Launch viewer using wrapper's method with UI hidden
        self.controller.launch_viewer("passive", show_ui=False)

        start_time = time.time()
        last_print_time = start_time
        last_step_time = start_time

        try:
            while self.controller.viewer.is_running() and (time.time() - start_time) < duration:
                current_time = time.time()
                dt = current_time - last_step_time
                last_step_time = current_time

                # Update pinch gesture FSM
                self.update_pinch_fsm(dt)

                # Step simulation using wrapper
                self.controller.step()

                # Print sensor data every second
                if current_time - last_print_time >= 1.0:
                    self.print_sensor_data()
                    last_print_time = current_time

                # Update rerun visualization
                self.update_rerun_visualization()

                # Maintain real-time simulation
                step_duration = time.time() - current_time
                time_until_next_step = self.controller.model.opt.timestep - step_duration
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)

        except KeyboardInterrupt:
            logger.info("Demo interrupted by user")

        logger.info("\nDemo completed!")


def main():
    """Main function to run the TaShan sensor demo."""
    parser = argparse.ArgumentParser(description="TaShan Touch Sensor Demo with Box Scene")
    parser.add_argument("--duration", type=float, default=60.0,
                       help="Demo duration in seconds (default: 60)")
    args = parser.parse_args()

    # Log system info
    logger.info(f"Python version: {sys.version}")
    logger.info(f"MuJoCo version: {mujoco.__version__}")

    # Create and run demo
    try:
        demo = TaShanDemo()

        # Run demo with pinch gesture
        demo.run_demo(duration=args.duration)

    except Exception as e:
        logger.error(f"Demo failed: {e}")
        raise


if __name__ == "__main__":
    main()
