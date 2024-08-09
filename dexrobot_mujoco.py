import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

import time, os
import argparse
import numpy as np
from scipy.spatial.transform import Rotation as R
from flask import Flask, Response
from threading import Thread
from dexrobot_urdf.utils.mj_control_utils import MJControlWrapper
from dexrobot_urdf.utils.mj_control_vr_utils import MJControlVRWrapper



parser = argparse.ArgumentParser()
parser.add_argument('model_path', type=str, help='Path to the Mujoco model XML file')
parser.add_argument('--config-yaml', type=str, help='Path to the YAML file containing the configuration', required=False)
parser.add_argument('--replay-csv', default=None, type=str, help='Path to the CSV file to replay. The CSV file should contain joint names as column headers and joint positions as rows.')
args = parser.parse_args()

app = Flask(__name__)

class MujocoJointController(Node):
    def __init__(self, enable_dynamics=True, enable_vr=False):
        super().__init__('mujoco_joint_controller')
        self.get_logger().info('Mujoco Joint Controller Node has been started.')

        # Load Mujoco model
        model_path = args.model_path  # Replace with the path to your Mujoco model
        self.mj = MJControlVRWrapper(os.path.join(os.getcwd(), model_path), enable_dynamics=enable_dynamics, enable_vr=enable_vr)
        # Enable infinite rotation for the actuator
        self.mj.enable_infinite_rotation("act_r_a_joint\d+")

        # adjust initial pos and camera pose
        if args.config_yaml is not None:
            self.mj.parse_yaml(args.config_yaml)
        self.mj.launch_viewer("passive")
        
        # timer for simulation
        self.start_time = time.time()
        self.sim_timer = self.create_timer(0.01, self.forward_sim)

        # timer for VR images
        if enable_vr:
            self.vr_timer = self.create_timer(0.05, self.mj.update_vr_images)
            # 启动 Flask 服务器线程
            self.flask_thread = Thread(target=self.run_flask)
            self.flask_thread.start()

        if args.replay_csv is None:
            self.subscription = self.create_subscription(
                JointState,
                'joint_states',
                self.joint_state_callback,
                10
            )
            # hand pose subscription
            self.hand_pose_subscription = self.create_subscription(Pose, 'manus_tracker_right', self.hand_pose_callback, 10)

            self.replay_csv = None
            self.replay_timer = None
        else:
            self.subscription = None
            import pandas as pd
            self.replay_csv = pd.read_csv(args.replay_csv)
            self.replay_timer = self.create_timer(0.1, self.forward_replay)
            self.joint_step_counter = 0

    def forward_sim(self):
        current_time = time.time() - self.start_time
        # Step the simulation to apply the control
        self.mj.data.ctrl[:] = 0.1* np.sin(current_time)
        self.mj.data.ctrl[:3] = 2 * np.sin(current_time)
        self.mj.step(until=current_time)

    def forward_replay(self):
        if args.replay_csv is not None:
            for act_name in self.mj.actuator_names:
                joint_name = act_name.replace('act_', '')
                if joint_name in self.replay_csv.columns:
                    if self.joint_step_counter < len(self.replay_csv):
                        self.mj.send_control(act_name, self.replay_csv[joint_name][self.joint_step_counter])
                    else:
                        exit(0)
            self.joint_step_counter += 1

    def hand_pose_callback(self, msg:Pose, verbose=True):
        position_raw = np.array([msg.position.x, msg.position.y, msg.position.z])
        orientation = np.array([msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z])
        rpy = R.from_quat(orientation, scalar_first=True).as_euler("xyz", degrees=False)

        if verbose:
            self.get_logger().info(f"pos={position_raw}, rpy={rpy}")
        #TODO: magic numbers to adjust the tracker pose (in reality) to the hand pose (in mujoco)
        position = 1.2 * (position_raw + np.array([0.65, -0.05, -0.15]))
        # send control
        self.mj.send_control("A_ARTx", position[0])
        self.mj.send_control("A_ARTy", position[1])
        self.mj.send_control("A_ARTz", position[2])
        self.mj.send_control("A_ARRx", rpy[0])
        self.mj.send_control("A_ARRy", rpy[1])
        self.mj.send_control("A_ARRz", rpy[2])
        
    def joint_state_callback(self, msg: JointState, verbose=False):
        if verbose:
            self.get_logger().info(f'Received Joint States: {msg}')

        for i, name in enumerate(msg.name):
            act_name = f'act_{name}'
            if act_name in self.mj.actuator_names:
                self.mj.send_control(act_name, msg.position[i])

    def run_flask(self):
        if self.mj.enable_vr:
            app.run(host="0.0.0.0", port=5000, threaded=True)

    def on_shutdown(self):
        self.mj_control_vr.stop_simulation()
        self.get_logger().info('Simulation stopped.')

@app.route("/video")
def video():
    """Video streaming route."""
    global node
    return Response(
        node.mj.generate_encoded_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )

def main(args=None):
    rclpy.init(args=args)
    global node 
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
