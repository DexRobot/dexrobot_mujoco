import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseArray

import time, os
import argparse
import numpy as np
import yaml
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
parser.add_argument('--output-formats', type=str, nargs='+', default=['ros'], help='Output formats. Supported formats are "ros" and "csv".')
parser.add_argument('--output-csv-path', type=str, default=None, help='Path to the output CSV file. Required when "csv" is included in the output formats.')
parser.add_argument('--enable-vr', action='store_true', help='Enable VR mode')
args = parser.parse_args()

app = Flask(__name__)

class MujocoJointController(Node):
    def __init__(
        self,
        enable_vr=False,
        output_formats=['ros'],
        output_csv_path=None,
    ):
        """
        Node for controlling the joints of a Mujoco model using ROS2 messages.

        Args:
            enable_vr (bool): Whether to enable VR mode.
            output_formats (list): List of output formats. Supported formats are 'ros' and 'csv'. When 'csv' is included, `output_csv_path` must be specified.
            output_csv_path (str): Path to the output CSV file. Must be specified when 'csv' is included in `output_formats`.
        """
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

        # joints/bodies names/ids to output, and buffer for the joint states reference
        if args.config_yaml is not None:
            with open(args.config_yaml, 'r') as f:
                config = yaml.safe_load(f)
            joint_names_to_track = config['tracked_joints']
            self.tracked_joint_names = [name for name in joint_names_to_track if self.mj.get_joint_id(name) != -1]
            body_names_to_track = config['tracked_bodies']
            self.tracked_body_names = [name for name in body_names_to_track if self.mj.get_link_id(name) != -1]
        else:
            self.tracked_joint_names = []
            self.tracked_body_names = []
        self.tracked_joint_ids = [self.mj.get_joint_id(name) for name in self.tracked_joint_names]
        self.tracked_body_ids = [self.mj.get_link_id(name) for name in self.tracked_body_names]
        self.tracked_joint_states_ref = {name: 0. for name in self.tracked_joint_names}

        # timer, publisher and buffer for publishing / logging data
        self.output_formats = output_formats
        if self.output_formats:
            self.output_timer = self.create_timer(0.01, self.output_data)
        else:
            self.output_timer = None
        if 'ros' in self.output_formats:
            self.joint_state_publisher = self.create_publisher(JointState, 'joint_states_actual', 10)
            self.body_pose_publisher = self.create_publisher(PoseArray, 'body_poses_actual', 10)
        else:
            self.joint_state_publisher = None
            self.body_pose_publisher = None
        if 'csv' in self.output_formats:
            if output_csv_path is not None:
                self.csv_buffer = []
                self.csv_path = output_csv_path
                with open(self.csv_path, 'w') as f:
                    f.write(','.join(
                        ['timestamp'] +
                        [f'{name}_pos' for name in self.tracked_joint_names] +
                        [f'{name}_vel' for name in self.tracked_joint_names] +
                        [f'{name}_pos' for name in self.tracked_body_names] +
                        [f'{name}_quat' for name in self.tracked_body_names]
                    ) + '\n')
            else:
                self.get_logger().error('Output CSV path must be specified when "csv" is included in the output formats.')
                exit(1)
        else:
            self.csv_buffer = None
            self.csv_path = None

    def forward_sim(self):
        """Forward the simulator until the current time."""
        current_time = time.time() - self.start_time
        # Step the simulation to apply the control
        self.mj.step(until=current_time)

    def forward_replay(self):
        """Forward the simulation using the replay CSV file."""
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
        """Callback function for the hand pose subscriber. Only useful when the free-floating hand model is used.

        Args:
            msg (Pose): The Pose message containing the target position and orientation of the hand.
            verbose (bool): Whether to print the target position and orientation.
        """
        raw_position_ref = np.array([msg.position.x, msg.position.y, msg.position.z])
        orientation_ref = np.array([msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z])

        if self.hand_position_offset is None:
            if time.time() - self.start_time < 2.:
                return
            self.hand_position_offset = raw_position_ref

        position_ref = args.position_magnifiers * (raw_position_ref - self.hand_position_offset)
        rpy_ref = R.from_quat(orientation_ref, scalar_first=True).as_euler('xyz')

        if verbose:
            self.get_logger().info(f"target_pos={position_ref}, target_orientation={orientation_ref}")

        # send control
        self.mj.send_control("act_ARTx", position_ref[0])
        self.mj.send_control("act_ARTy", position_ref[1])
        self.mj.send_control("act_ARTz", position_ref[2])
        self.mj.send_control("act_ARRx", rpy_ref[0])
        self.mj.send_control("act_ARRy", rpy_ref[1])
        self.mj.send_control("act_ARRz", rpy_ref[2])

        # save to buffer
        if 'ARTx' in self.tracked_joint_states_ref:
            self.tracked_joint_states_ref['ARTx'] = position_ref[0]
        if 'ARTy' in self.tracked_joint_states_ref:
            self.tracked_joint_states_ref['ARTy'] = position_ref[1]
        if 'ARTz' in self.tracked_joint_states_ref:
            self.tracked_joint_states_ref['ARTz'] = position_ref[2]
        if 'ARRx' in self.tracked_joint_states_ref:
            self.tracked_joint_states_ref['ARRx'] = rpy_ref[0]
        if 'ARRy' in self.tracked_joint_states_ref:
            self.tracked_joint_states_ref['ARRy'] = rpy_ref[1]
        if 'ARRz' in self.tracked_joint_states_ref:
            self.tracked_joint_states_ref['ARRz'] = rpy_ref[2]


    def joint_state_callback(self, msg: JointState, verbose=False):
        """Callback function for the joint state subscriber. Responsible for sending control to both arm actuators (when applicable) and finger joint actuators.

        Args:
            msg (JointState): The JointState message containing the joint names and positions.
            verbose (bool): Whether to print the received joint states.
        """
        if verbose:
            self.get_logger().info(f'Received Joint States: {msg}')

        for name, pos in zip(msg.name, msg.position):
            # send control
            act_name = f'act_{name}'
            if act_name in self.mj.actuator_names:
                self.mj.send_control(act_name, pos)
            # save to buffer
            if name in self.tracked_joint_names:
                self.tracked_joint_states_ref[name] = pos

    def output_data(self):
        """Output the data to the specified formats."""
        # Gather actual joint states and body poses
        joint_pos = self.mj.data.qpos[self.tracked_joint_ids]
        joint_vel = self.mj.data.qvel[self.tracked_joint_ids]
        body_pos = self.mj.data.xpos[self.tracked_body_ids]
        body_quat = self.mj.data.xquat[self.tracked_body_ids]

        if 'ros' in self.output_formats:
            # Publish the actual joint states
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = self.tracked_joint_names
            joint_state_msg.position = list(joint_pos)
            joint_state_msg.velocity = list(joint_vel)
            self.joint_state_publisher.publish(joint_state_msg)

            # Publish the actual body poses
            body_pose_msg = PoseArray()
            body_pose_msg.header.stamp = self.get_clock().now().to_msg()
            for pos, quat in zip(body_pos, body_quat):
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = pos
                pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z = quat
                body_pose_msg.poses.append(pose)
            self.body_pose_publisher.publish(body_pose_msg)

        if 'csv' in self.output_formats:
            self.csv_buffer.append([self.get_clock().now().nanoseconds, *joint_pos, *joint_vel, *body_pos, *body_quat])
            if len(self.csv_buffer) >= 100:
                with open(self.csv_path, 'a') as f:
                    for row in self.csv_buffer:
                        f.write(','.join(map(str, row)) + '\n')
                self.csv_buffer.clear()

    def run_flask(self):
        """Run the Flask server for video streaming."""
        if self.mj.enable_vr:
            app.run(host="0.0.0.0", port=5000, threaded=True)

    def on_shutdown(self):
        """Clean up the node."""
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

def main():
    rclpy.init()
    global node
    node = MujocoJointController(
        enable_vr=args.enable_vr,
        output_formats=args.output_formats,
        output_csv_path=args.output_csv_path,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
