import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import mujoco
from utils.urdf_utils import load_meshes

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
        model_path = 'dexrobot_urdf/mjcf/right_hand.xml'  # Replace with the path to your Mujoco model
        mesh_dir = 'dexrobot_urdf/meshes'
        self.model = mujoco.MjModel.from_xml_path(model_path, load_meshes(mesh_dir))
        self.data = mujoco.MjData(self.model)
        self.sim = mujoco.MjSim(self.model)

        self.joint_names = [self.model.joint(j).name for j in range(self.model.njnt)]
        self.get_logger().info('Mujoco Joint Controller Node has been started.')

    def joint_state_callback(self, msg: JointState):
        self.get_logger().info(f'Received Joint States: {msg}')

        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                joint_index = self.joint_names.index(name)
                self.data.ctrl[joint_index] = msg.position[i]

        # Step the simulation to apply the control
        mujoco.mj_step(self.model, self.data)

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
