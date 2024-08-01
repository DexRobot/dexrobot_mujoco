import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import mujoco
import mujoco.viewer
from dexrobot_urdf.utils.mjcf_utils import load_meshes
from dexrobot_urdf.utils.mj_control_utils import MJControlWrapper
import argparse
import time

parser = argparse.ArgumentParser()
parser.add_argument('model', type=str, help='Path to the Mujoco model XML file')
parser.add_argument('--mesh-dir', type=str, default="dexrobot_urdf/meshes/", help='Path to the directory containing the mesh files')
parser.add_argument('--config-yaml', type=str, help='Path to the YAML file containing the configuration', required=False)
args = parser.parse_args()


class MujocoJointController(Node):
    def __init__(self):
        super().__init__('mujoco_joint_controller')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

        # Load Mujoco model
        model_path = args.model  # Replace with the path to your Mujoco model
        mesh_dir = args.mesh_dir
        config_yaml = args.config_yaml
        self.mj = MJControlWrapper(model_path, mesh_dir)
        if config_yaml is not None:
            self.mj.parse_yaml(config_yaml)
        self.mj.launch_viewer("passive")
        self.get_logger().info('Mujoco Joint Controller Node has been started.')
        self.start_time = time.time()

    def joint_state_callback(self, msg: JointState):
        self.get_logger().info(f'Received Joint States: {msg}')

        for i, name in enumerate(msg.name):
            act_name = f'act_{name}'
            self.mj.send_control(act_name, msg.position[i])

        # Step the simulation to apply the control
        current_time = time.time() - self.start_time
        self.mj.step(current_time)

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
