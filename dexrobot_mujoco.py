import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import mujoco
import mujoco.viewer
from dexrobot_urdf.utils.mujoco_utils import load_meshes
import argparse
import time

parser = argparse.ArgumentParser()
parser.add_argument('model', type=str, help='Path to the Mujoco model XML file')
parser.add_argument('--mesh-dir', type=str, default="dexrobot_urdf/meshes/", help='Path to the directory containing the mesh files')
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
        self.model = mujoco.MjModel.from_xml_path(model_path, load_meshes(mesh_dir))
        self.data = mujoco.MjData(self.model)
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

        self.num_actuators = len(self.model.actuator_actnum)
        self.actuator_names = [self.model.actuator(j).name for j in range(self.num_actuators)]
        self.get_logger().info('Mujoco Joint Controller Node has been started.')
        self.start_time = time.time()

    def joint_state_callback(self, msg: JointState):
        self.get_logger().info(f'Received Joint States: {msg}')

        for i, name in enumerate(msg.name):
            act_name = f'act_{name}'
            if act_name in self.actuator_names:
                actuator_index = self.actuator_names.index(act_name)
                self.data.ctrl[actuator_index] = msg.position[i]

        # Step the simulation to apply the control
        current_time = time.time() - self.start_time
        while self.data.time < current_time:
            mujoco.mj_step(self.model, self.data)

        # Update the viewer
        self.viewer.sync()

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
