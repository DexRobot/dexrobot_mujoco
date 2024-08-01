import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import time
import argparse
import numpy as np
import mujoco
import mujoco.viewer
from dexrobot_urdf.utils.mjcf_utils import load_meshes
from dexrobot_urdf.utils.mj_control_vr_utils import MJControlVRWrapper
from flask import Flask, Response
from threading import Thread


parser = argparse.ArgumentParser()
parser.add_argument('model_path', type=str, help='Path to the Mujoco model XML file')
parser.add_argument('--mesh-dir', type=str, default="dexrobot_urdf/meshes/", help='Path to the directory containing the mesh files')
parser.add_argument('--config-yaml', type=str, help='Path to the YAML file containing the configuration', required=False)
args = parser.parse_args()

app = Flask(__name__)

class MujocoVRJointController(Node):
    def __init__(self, enable_vr=True):
        super().__init__('mujocoVR_joint_controller')
        self.get_logger().info('MujocoVR Joint Controller Node has been started.')

        # Load Mujoco model
        model_path = args.model_path  # Replace with the path to your Mujoco model
        mesh_dir = args.mesh_dir
        config_yaml = args.config_yaml
        self.mjVR_control_wrapper = MJControlVRWrapper(model_path, mesh_dir, enable_vr)
        if config_yaml is not None:
             self.mjVR_control_wrapper.parse_yaml(config_yaml)
        self.mjVR_control_wrapper.launch_viewer("passive")

        self.num_actuators = len(self.mjVR_control_wrapper.model.actuator_actnum)
        self.actuator_names = [self.mjVR_control_wrapper.model.actuator(j).name for j in range(self.num_actuators)]
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
        # timer for VR images
        if enable_vr:
            self.vr_timer = self.create_timer(0.1, self.mjVR_control_wrapper.update_vr_images)
            # 启动 Flask 服务器线程
            self.flask_thread = Thread(target=self.run_flask)
            self.flask_thread.start()

    def forward_sim(self):
        current_time = time.time() - self.start_time
        # Step the simulation to apply the control
        self.mjVR_control_wrapper.data.ctrl[:] = self.ctrl_value
        self.mjVR_control_wrapper.step(until=current_time)

    def joint_state_callback(self, msg: JointState, verbose=False):
        if verbose:
            self.get_logger().info(f'Received Joint States: {msg}')

        for i, name in enumerate(msg.name):
            act_name = f'act_{name}'
            if act_name in self.actuator_names:
                actuator_index = self.actuator_names.index(act_name)
                self.ctrl_value[actuator_index] = msg.position[i]

    def run_flask(self):
        if self.mjVR_control_wrapper.enable_vr:
            app.run(host="0.0.0.0", port=5000, threaded=True)

    def on_shutdown(self):
        self.mj_control_vr.stop_simulation()
        self.get_logger().info('Simulation stopped.')

@app.route("/video")
def video():
    """Video streaming route."""
    global node
    return Response(
        node.mjVR_control_wrapper.generate_encoded_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )

def main(args=None):
    rclpy.init(args=args)
    global node
    node = MujocoVRJointController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
