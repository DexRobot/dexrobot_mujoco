from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'dexrobot_mujoco'

# Gather all data files to be installed
config_files = glob('config/*')
launch_files = glob('launch/*')
model_files = glob('dexrobot_mujoco/models/**/*', recursive=True)
scene_files = glob('dexrobot_mujoco/scenes/**/*', recursive=True)

# Filter out __pycache__ and other unwanted files
def filter_files(files):
    return [f for f in files if not any(x in f for x in ['__pycache__', '.pyc', '.pyo'])]

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ROS2 package requirements
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install launch files
        (os.path.join('share', package_name, 'launch'), filter_files(launch_files)),

        # Install config files
        (os.path.join('share', package_name, 'config'), filter_files(config_files)),

        # Install model files
        *[
            (os.path.join('share', package_name, os.path.dirname(f)), [f])
            for f in filter_files(model_files)
        ],

        # Install scene files
        *[
            (os.path.join('share', package_name, os.path.dirname(f)), [f])
            for f in filter_files(scene_files)
        ],
    ],
    install_requires=[
        'setuptools',
        'mujoco>=3.0.0',
        'numpy',
        'pyyaml',
        'loguru',
        'scipy',
        'flask',
        'opencv-python',
        'pandas',
    ],
    extras_require={
        'dev': [
            'pytest',
            'pytest-cov',
            'flake8',
            'black',
            'mypy',
        ],
    },
    zip_safe=True,
    maintainer='DexRobot',
    maintainer_email='lyw@dex-robot.com',
    description='MuJoCo binding for DexRobot with ROS2 integration',
    license='MIT',  # Adjust according to your license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dexrobot_mujoco_node = nodes.dexrobot_mujoco_node:main',
            'urdf2mjcf = scripts.urdf2mjcf:main',
            'create_scene = scripts.create_scene:main',
        ],
    },
    classifiers=[
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Science/Research',
        'Topic :: Scientific/Engineering :: Robotics',
        'License :: OSI Approved :: MIT License',  # Adjust according to your license
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
    ],
    python_requires='>=3.8',
)
