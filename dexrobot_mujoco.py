import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import time
import argparse
import numpy as np
import mujoco
import mujoco.viewer
from dexrobot_urdf.utils.mjcf_utils import load_meshes
from dexrobot_urdf.utils.mj_control_utils import MJControlWrapper


parser = argparse.ArgumentParser()
parser.add_argument('model_path', type=str, help='Path to the Mujoco model XML file')
parser.add_argument('--mesh-dir', type=str, default="dexrobot_urdf/meshes/", help='Path to the directory containing the mesh files')
parser.add_argument('--config-yaml', type=str, help='Path to the YAML file containing the configuration', required=False)
args = parser.parse_args()


class MujocoJointController(Node):
    def __init__(self):
        super().__init__('mujoco_joint_controller')
        self.get_logger().info('Mujoco Joint Controller Node has been started.')

        # Load Mujoco model
        model_path = args.model_path  # Replace with the path to your Mujoco model
        mesh_dir = args.mesh_dir
        self.mj_control_wrapper = MJControlWrapper(model_path, mesh_dir)
        self.mj_control_wrapper.launch_viewer("passive")

        self.num_actuators = len(self.mj_control_wrapper.model.actuator_actnum)
        self.actuator_names = [self.mj_control_wrapper.model.actuator(j).name for j in range(self.num_actuators)]
        self.ctrl_value = np.zeros(self.num_actuators)
        self.start_time = time.time()

        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        # timer for simulation
        self.sim_timer = self.create_timer(0.01, self.forward_sim)

    def forward_sim(self):
        current_time = time.time() - self.start_time
        # Step the simulation to apply the control
        self.mj_control_wrapper.step(until=current_time)

    def joint_state_callback(self, msg: JointState, verbose=False):
        if verbose:
            self.get_logger().info(f'Received Joint States: {msg}')

        for i, name in enumerate(msg.name):
            act_name = f'act_{name}'
            if act_name in self.actuator_names:
                actuator_index = self.actuator_names.index(act_name)
                self.ctrl_value[actuator_index] = msg.position[i]

def main(args=None):
    rclpy.init(args=args)
    node = MujocoJointController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()