# TaShan Sensor Setup with Python 3.8 and ROS

## The Challenge

TaShan touch sensors require Python 3.8 due to their dependency on `libpython3.8.so.1.0`. However, using Python 3.8 with ROS on modern systems can be challenging since most ROS distributions are built for newer Python versions.

## Solution: Using Conda with RoboStack

This guide provides a workaround using Conda and RoboStack to create a Python 3.8 environment with ROS support.

### Prerequisites

- Conda or Miniconda installed
- TaShan sensor library files in `dexrobot_mujoco/ts_sensor_lib/`

### Setup Steps

1. **Create Python 3.8 Environment with ROS**
   ```bash
   conda create -n tashan_env python=3.8
   conda activate tashan_env

   # Add RoboStack channel
   conda config --add channels conda-forge
   conda config --add channels robostack

   # Install ROS Foxy base
   conda install ros-foxy-ros-base
   ```

2. **Install Required Dependencies**
   ```bash
   # Install specific spdlog version
   conda install spdlog=1.8.2

   # Upgrade numpy via pip
   pip install --upgrade numpy

   # Install MuJoCo 3.2.3
   pip install mujoco==3.2.3

   # Install dexrobot_mujoco with TaShan support
   cd /path/to/dexrobot_mujoco
   pip install -e .[tashan]
   ```

3. **Configure Environment Variables**
   ```bash
   # Ensure LD_LIBRARY_PATH includes conda libraries
   export LD_LIBRARY_PATH=$CONDA_PREFIX/lib:$LD_LIBRARY_PATH

   # Set RMW implementation
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

   # Source ROS setup
   source $CONDA_PREFIX/setup.bash
   ```

4. **Verify Installation**
   ```bash
   # Check Python version
   python --version  # Should show Python 3.8.x

   # Check MuJoCo version
   python -c "import mujoco; print(mujoco.__version__)"  # Should show 3.2.3

   # Check ROS
   ros2 topic list  # Should work without errors
   ```

### Running with TaShan Sensors

```bash
# With ROS support
python nodes/dexrobot_mujoco_ros.py dexrobot_mujoco/models/dexhand021_right_ts_sensor.xml \
    --config config/ball_catching.yaml \
    --enable-ts-sensor

# Standalone (see examples/tashan_standalone_demo.py)
python examples/tashan_standalone_demo.py
```

### Troubleshooting

1. **Library not found errors**
   - Ensure `LD_LIBRARY_PATH` includes `$CONDA_PREFIX/lib`
   - Check that TaShan .so files are in `dexrobot_mujoco/ts_sensor_lib/linux/`

2. **ROS communication issues**
   - Verify `RMW_IMPLEMENTATION` is set correctly
   - Check that all ROS nodes are using the same RMW implementation

3. **Import errors**
   - Make sure you're in the correct conda environment
   - Reinstall packages in the correct order if needed

### Alternative: Standalone Usage

If ROS integration proves too complex, you can use TaShan sensors without ROS. See `examples/tashan_standalone_demo.py` for a minimal example that demonstrates TaShan sensor functionality without ROS dependencies.
