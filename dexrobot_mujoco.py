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

def float_list(x):
    return np.array(list(map(float, x.split(','))))

parser = argparse.ArgumentParser()
parser.add_argument('model_path', type=str, help='Path to the Mujoco model XML file')
parser.add_argument('--config-yaml', type=str, help='Path to the YAML file containing the configuration', required=False)
parser.add_argument('--replay-csv', default=None, type=str, help='Path to the CSV file to replay. The CSV file should contain joint names as column headers and joint positions as rows.')
parser.add_argument('--hand-pose-topic', default=None, type=str, help='Topic name for hand pose. When left blank, this node will subscribe to joint positions from the topic "joint_states". When set, this node will subscribe to hand pose from the specified topic and try to set the hand pose using 6 DoF arbitrary joint in the model (skipping IK).')
parser.add_argument('--position-magnifiers', type=float_list, default=[2.5, 2., .8], help='Position magnifiers for the hand pose control')
args = parser.parse_args()

app = Flask(__name__)

class MujocoJointController(Node):
    def __init__(self, enable_vr=False):
        super().__init__('mujoco_joint_controller')
        self.get_logger().info('Mujoco Joint Controller Node has been started.')

        # Load Mujoco model
        model_path = args.model_path  # Replace with the path to your Mujoco model
        self.mj = MJControlVRWrapper(os.path.join(os.getcwd(), model_path), enable_vr=enable_vr)
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
            self.joint_state_subscription = self.create_subscription(
                JointState,
                'joint_states',
                self.joint_state_callback,
                10
            )
            if args.hand_pose_topic is not None:
                self.hand_pose_subscription = self.create_subscription(Pose, args.hand_pose_topic, self.hand_pose_callback, 10)
            else:
                self.hand_pose_subscription = None

            self.replay_csv = None
            self.replay_timer = None
        else:
            self.joint_state_subscription = None
            self.hand_pose_subscription = None
            import pandas as pd
            self.replay_csv = pd.read_csv(args.replay_csv)
            self.replay_timer = self.create_timer(0.1, self.forward_replay)
            self.joint_step_counter = 0

        self.hand_position_offset = None

    def forward_sim(self):
        current_time = time.time() - self.start_time
        # Step the simulation to apply the control
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
        raw_position = np.array([msg.position.x, msg.position.y, msg.position.z])
        orientation = np.array([msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z])

        if self.hand_position_offset is None:
            if time.time() - self.start_time < 2.:
                return
            self.hand_position_offset = raw_position

        position = args.position_magnifiers * (raw_position - self.hand_position_offset)

        if verbose:
            self.get_logger().info(f"target_pos={position}, target_orientation={orientation}")

        # send control
        rpy = R.from_quat(orientation, scalar_first=True).as_euler('xyz')
        self.mj.send_control("act_ARTx", position[0])
        self.mj.send_control("act_ARTy", position[1])
        self.mj.send_control("act_ARTz", position[2])
        self.mj.send_control("act_ARRx", rpy[0])
        self.mj.send_control("act_ARRy", rpy[1])
        self.mj.send_control("act_ARRz", rpy[2])

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
