"""TaShan sensor manager for MuJoCo simulations."""

import os
import sys
import platform
import numpy as np
from loguru import logger
import mujoco
from collections import namedtuple

# Named tuple for semantic force data access
TSForceData = namedtuple('TSForceData', ['normal', 'tangential_magnitude', 'tangential_direction'])


class TSSensorManager:
    """Manager for TaShan touch sensors in MuJoCo simulations.
    
    The TaShan sensors provide 11-dimensional tactile data:
    - Dimension 0: Proximity sensing
    - Dimension 1: Normal force magnitude (N)
    - Dimension 2: Tangential force magnitude (N)
    - Dimension 3: Tangential force direction (rad)
    - Dimensions 4-10: Raw capacitance values (7 channels)
    
    The manager provides both raw data access and semantic APIs that
    extract the force-related dimensions (1-3) for easier use.
    """
    
    @staticmethod
    def initialize_before_model_load():
        """Initialize TS sensor callback before MuJoCo model is loaded.
        
        This must be called before creating the MuJoCo model for TS sensors to work.
        
        Returns:
            bool: True if initialization successful, False otherwise
        """
        try:
            # Detect platform
            system = platform.system().lower()
            
            # Determine platform-specific paths
            current_dir = os.path.dirname(os.path.abspath(__file__))
            if system == 'linux':
                platform_dir = 'linux'
            elif system == 'windows':
                platform_dir = 'win'
            else:
                logger.error(f"TaShan sensor not supported on platform: {system}")
                return False
            
            ts_lib_path = os.path.join(current_dir, "..", "ts_sensor_lib", platform_dir)
            
            # Add the TS sensor library path to Python path for import
            if ts_lib_path not in sys.path:
                sys.path.insert(0, ts_lib_path)
            
            # Import TSensor module and register callback (following reference implementation)
            try:
                import TSensor
                TSensor.register_sensor_callback()
                logger.info(f"TS sensor callback registered successfully via TSensor module")
                return True
            except ImportError as e:
                logger.error(f"Failed to import TSensor module from {ts_lib_path}: {e}")
                return False
            except AttributeError as e:
                logger.error(f"TSensor module does not have register_sensor_callback method: {e}")
                return False
            
        except Exception as e:
            logger.error(f"Failed to initialize TS sensor callback: {e}")
            return False
    
    def __init__(self, mj_wrapper):
        """Initialize the TS sensor manager.
        
        Args:
            mj_wrapper: MJControlWrapper instance
        """
        self.mj = mj_wrapper
        self.ts_sensor_available = False
        self._verify_ts_sensors()
        
    def _verify_ts_sensors(self):
        """Verify that TS sensors are properly initialized in the model."""
        # Count TS sensors in the model
        ts_sensor_count = 0
        self.ts_sensor_names = []
        
        for i in range(self.mj.model.nsensor):
            sensor_name = mujoco.mj_id2name(
                self.mj.model,
                mujoco.mjtObj.mjOBJ_SENSOR,
                i
            )
            
            # Check if it's a TS sensor (TS-F-A-1, TS-F-A-2, etc.)
            if (sensor_name and 
                sensor_name.startswith('TS-F-A-') and
                self.mj.model.sensor_type[i] == mujoco.mjtSensor.mjSENS_USER and
                self.mj.model.sensor_dim[i] == 11):
                ts_sensor_count += 1
                self.ts_sensor_names.append(sensor_name)
                logger.debug(f"Found TS sensor: {sensor_name}")
        
        if ts_sensor_count > 0:
            self.ts_sensor_available = True
            # Sort sensor names for consistent ordering
            self.ts_sensor_names.sort()
            logger.info(f"Found {ts_sensor_count} TS sensors in the model: {self.ts_sensor_names}")
        else:
            logger.warning("No TS sensors found in the model")
    
    def get_raw_sensor_data_by_id(self, sensor_id):
        """Get raw TS sensor data by ID as bytes.
        
        Args:
            sensor_id: Sensor ID (i >= 1)
            
        Returns:
            bytes: Raw 88-byte sensor data (11 * 8 bytes)
                   Dimensions 0-3 are doubles: proximity, normal force, 
                   tangential force magnitude, tangential direction
                   Dimensions 4-10 are raw capacitance values
        """
        if sensor_id < 1:
            raise ValueError(f"Sensor ID must be >= 1, got {sensor_id}")
            
        if not self.ts_sensor_available:
            raise RuntimeError("TS sensor not available")
            
        sensor_name = f"TS-F-A-{sensor_id}"
        
        # Get sensor ID
        mj_sensor_id = mujoco.mj_name2id(
            self.mj.model, 
            mujoco.mjtObj.mjOBJ_SENSOR, 
            sensor_name
        )
        
        if mj_sensor_id < 0:
            raise ValueError(f"TS sensor {sensor_name} not found in model")
            
        # Get sensor data as raw bytes (11 * 8 bytes = 88 bytes)
        sensor_addr = self.mj.model.sensor_adr[mj_sensor_id]
        sensor_data = self.mj.data.sensordata[sensor_addr:sensor_addr+11]
        raw_bytes = sensor_data.tobytes()
        
        # Debug: Log raw data for troubleshooting
        logger.debug(f"TS sensor {sensor_name} raw bytes: {raw_bytes.hex()}")
        
        return raw_bytes
    
    def get_all_raw_sensor_data(self):
        """Get raw data from all available TS sensors.
        
        Returns:
            dict: Dictionary mapping sensor IDs to their raw 88-byte data
        """
        if not self.ts_sensor_available:
            raise RuntimeError("TS sensor not available")
            
        sensor_data = {}
        for sensor_name in self.ts_sensor_names:
            # Extract sensor ID from name (TS-F-A-1 -> 1)
            sensor_id = int(sensor_name.split('-')[-1])
            sensor_data[sensor_id] = self.get_raw_sensor_data_by_id(sensor_id)
            
        return sensor_data
    
    def get_force_data_by_id(self, sensor_id):
        """Get force data from TS sensor by ID.
        
        Args:
            sensor_id: Sensor ID (i >= 1)
            
        Returns:
            TSForceData: Named tuple with fields:
                - normal: Normal force (N)
                - tangential_magnitude: Tangential force magnitude (N)
                - tangential_direction: Tangential force direction (degrees)
        """
        if sensor_id < 1:
            raise ValueError(f"Sensor ID must be >= 1, got {sensor_id}")
            
        if not self.ts_sensor_available:
            raise RuntimeError("TS sensor not available")
            
        sensor_name = f"TS-F-A-{sensor_id}"
        
        # Get sensor ID
        mj_sensor_id = mujoco.mj_name2id(
            self.mj.model, 
            mujoco.mjtObj.mjOBJ_SENSOR, 
            sensor_name
        )
        
        if mj_sensor_id < 0:
            raise ValueError(f"TS sensor {sensor_name} not found in model")
            
        # Get sensor address and extract force data (indices 1, 2, 3)
        sensor_addr = self.mj.model.sensor_adr[mj_sensor_id]
        force_values = self.mj.data.sensordata[sensor_addr+1:sensor_addr+4]
        
        # Return as named tuple for semantic access
        return TSForceData(
            normal=float(force_values[0]),
            tangential_magnitude=float(force_values[1]),
            tangential_direction=float(force_values[2])
        )
    
    def get_all_force_data(self):
        """Get force data from all available TS sensors.
        
        Returns:
            dict: Dictionary mapping sensor IDs to TSForceData named tuples
        """
        if not self.ts_sensor_available:
            raise RuntimeError("TS sensor not available")
            
        force_data = {}
        for sensor_name in self.ts_sensor_names:
            # Extract sensor ID from name (TS-F-A-1 -> 1)
            sensor_id = int(sensor_name.split('-')[-1])
            force_data[sensor_id] = self.get_force_data_by_id(sensor_id)
            
        return force_data