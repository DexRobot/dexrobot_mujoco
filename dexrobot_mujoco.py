import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseArray

import time, os
import argparse
from loguru import logger
import numpy as np
import yaml
import subprocess
from scipy.spatial.transform import Rotation as R
from flask import Flask, Response
import requests, signal
from threading import Thread
import cv2
from dexrobot_urdf.utils.mj_control_utils import MJControlWrapper
from dexrobot_urdf.utils.mj_control_vr_utils import MJControlVRWrapper


def float_list(x):
    return np.array(list(map(float, x.split(","))))


class MujocoJointController(Node):
    def __init__(
        self,
        model_path,
        config_yaml=None,
        replay_csv=None,
        hand_pose_topic=None,
        position_magnifiers=[2.5, 2.0, 0.8],
        output_formats=['ros'],
        output_csv_path=None,
        output_bag_path=None,
        output_mp4_path=None,
        additional_bag_topics=[],
        enable_vr=False,
        seed=0,
    ):
        """
        Node for controlling the joints of a Mujoco model using ROS2 messages.

        Args:
            model_path (str): Path to the Mujoco model XML file.
            config_yaml (str): Path to the YAML file containing the configuration. If provided, the YAML file should
                include details such as tracked joints and bodies, and other simulation parameters.
            replay_csv (str): Path to the CSV file to replay. The CSV file should contain joint names as column headers
                and joint positions as rows. If provided, this will override ROS-based input sources.
            hand_pose_topic (str): Topic name for hand pose. When left blank, this node will subscribe to joint
                positions from the topic "joint_states". When set, this node will subscribe to hand pose from the
                specified topic and try to set the hand pose using 6 DoF arbitrary joint in the model (skipping IK).
            position_magnifiers (list of float): Position magnifiers for the hand pose control. These factors will scale
                the hand pose coordinates received from the `hand_pose_topic`.
            output_formats (list of str): List of output formats. Supported formats: 'ros', 'csv', 'mp4'. When 'csv' is included, `output_csv_path` must be specified. When 'mp4' is included, `output_mp4_path` must be specified.
            output_csv_path (str): Path to the output CSV file. Must be specified when 'csv' is included in `output_formats`.
            output_bag_path (str): Path to the output bag file.
            additional_bag_topics (list of str): Additional topics to record in the bag file (other than those related to Mujoco itself).
            output_mp4_path (str): Path to the output MP4 file. Must be specified when 'mp4' is included in `output_formats`.
            enable_vr (bool): Whether to enable VR mode. When enabled, VR control images will be updated, and the Flask
                server will run to provide a video stream.
        """
        super().__init__("mujoco_joint_controller")
        self.get_logger().info("Mujoco Joint Controller Node has been started.")

        self.app = Flask(__name__)
        self.model_path = model_path
        self.config_yaml = config_yaml
        self.replay_csv = replay_csv
        self.hand_pose_topic = hand_pose_topic
        self.position_magnifiers = position_magnifiers
        self.output_formats = output_formats
        self.output_csv_path = output_csv_path
        self.enable_vr = enable_vr

        # Load Mujoco model
        self.mj = MJControlVRWrapper(
            os.path.join(os.getcwd(), self.model_path),
            enable_vr=self.enable_vr,
            renderer_dimension=(640, 480) if "mp4" in self.output_formats else None,
            seed=seed,
        )
        self.mj.enable_infinite_rotation("act_r_a_joint\d+")

        # adjust initial pos and camera pose
        if self.config_yaml is not None:
            self.mj.parse_yaml(self.config_yaml)
        self.mj.launch_viewer("passive")

        # timer for simulation
        self.start_time = time.time()
        self.sim_timer = self.create_timer(0.01, self.forward_sim)

        # timer for VR images
        if self.enable_vr:
            self.vr_timer = self.create_timer(0.05, self.mj.update_vr_images)
            # Start Flask server thread
            self.flask_thread = Thread(target=self.run_flask)
            self.flask_thread.start()

        if self.replay_csv is None:
            self.joint_state_subscription = self.create_subscription(
                JointState, "joint_states", self.joint_state_callback, 10
            )
            if self.hand_pose_topic is not None:
                self.hand_pose_subscription = self.create_subscription(
                    Pose, self.hand_pose_topic, self.hand_pose_callback, 10
                )
            else:
                self.hand_pose_subscription = None

            self.replay_csv = None
            self.replay_timer = None
        else:
            self.joint_state_subscription = None
            self.hand_pose_subscription = None
            import pandas as pd

            self.replay_csv = pd.read_csv(self.replay_csv)
            self.replay_timer = self.create_timer(0.1, self.forward_replay)
            self.joint_step_counter = 0

        self.hand_position_offset = None

        # joints/bodies names/ids to output, and buffer for the joint states reference
        if self.config_yaml is not None:
            with open(self.config_yaml, "r") as f:
                config = yaml.safe_load(f)
            joint_names_to_track = [
                item for sublist in config["tracked_joints"] for item in sublist
            ]
            logger.warning(f"Tracked joints: {joint_names_to_track}")
            self.tracked_joint_names = [
                name
                for name in joint_names_to_track
                if self.mj.get_joint_id(name) != -1
            ]
            body_names_to_track = [
                item for sublist in config["tracked_bodies"] for item in sublist
            ]
            self.tracked_body_names = [
                name for name in body_names_to_track if self.mj.get_link_id(name) != -1
            ]
        else:
            self.tracked_joint_names = []
            self.tracked_body_names = []
        self.tracked_joint_ids = [
            self.mj.get_joint_id(name) for name in self.tracked_joint_names
        ]
        self.tracked_body_ids = [
            self.mj.get_link_id(name) for name in self.tracked_body_names
        ]
        self.tracked_joint_states_ref = {name: 0.0 for name in self.tracked_joint_names}

        # timer, publisher and buffer for publishing / logging data
        if self.output_formats:
            self.output_timer = self.create_timer(0.01, self.output_data)
        else:
            self.output_timer = None
        if "ros" in self.output_formats:
            self.joint_state_publisher = self.create_publisher(
                JointState, "joint_states_actual", 10
            )
            self.body_pose_publisher = self.create_publisher(
                PoseArray, "body_poses_actual", 10
            )
        else:
            self.joint_state_publisher = None
            self.body_pose_publisher = None
        if "csv" in self.output_formats:
            if self.output_csv_path is not None:
                self.csv_buffer = []
                self.csv_path = self.output_csv_path
                with open(self.csv_path, "w") as f:
                    f.write(
                        ",".join(
                            ["timestamp"]
                            + [f"{name}_pos" for name in self.tracked_joint_names]
                            + [f"{name}_vel" for name in self.tracked_joint_names]
                            + [f"{name}_pos" for name in self.tracked_body_names]
                            + [f"{name}_quat" for name in self.tracked_body_names]
                        )
                        + "\n"
                    )
            else:
                self.get_logger().error(
                    'Output CSV path must be specified when "csv" is included in the output formats.'
                )
                exit(1)
        else:
            self.csv_buffer = None
            self.csv_path = None

        if "ros" in output_formats and output_bag_path is not None:
            topics = [
                "joint_states",
                "joint_states_actual",
                "body_poses_actual",
            ] + additional_bag_topics
            command = f'ros2 bag record -o {output_bag_path} {" ".join(topics)}'
            self.rosbag_process = subprocess.Popen(command, shell=True)
            self.get_logger().info(f"Recording to bag file: {output_bag_path}")
        else:
            self.rosbag_process = None

        if "mp4" in output_formats:
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self.fps = 20.0
            self.last_video_frame_time = time.time()
            self.video_writer = cv2.VideoWriter(
                output_mp4_path, fourcc, self.fps, self.mj.renderer_dimension
            )
        else:
            self.fps = None
            self.last_video_frame_time = None
            self.video_writer = None

    def forward_sim(self):
        """Forward the simulator until the current time."""
        current_time = time.time() - self.start_time
        # Step the simulation to apply the control
        self.mj.step(until=current_time)

    def forward_replay(self):
        """Forward the simulation using the replay CSV file."""
        if self.replay_csv is not None:
            for act_name in self.mj.actuator_names:
                joint_name = act_name.replace("act_", "")
                if joint_name in self.replay_csv.columns:
                    if self.joint_step_counter < len(self.replay_csv):
                        self.mj.send_control(
                            act_name,
                            self.replay_csv[joint_name][self.joint_step_counter],
                        )
                    else:
                        exit(0)
            self.joint_step_counter += 1

    def hand_pose_callback(self, msg: Pose, verbose=True):
        """Callback function for the hand pose subscriber."""
        raw_position_ref = np.array([msg.position.x, msg.position.y, msg.position.z])
        orientation_ref = np.array(
            [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z]
        )

        if self.hand_position_offset is None:
            if time.time() - self.start_time < 2.0:
                return
            self.hand_position_offset = raw_position_ref

        position_ref = self.position_magnifiers * (
            raw_position_ref - self.hand_position_offset
        )
        rpy_ref = R.from_quat(orientation_ref, scalar_first=True).as_euler("xyz")

        if verbose:
            self.get_logger().info(
                f"target_pos={position_ref}, target_orientation={orientation_ref}"
            )

        # send control
        self.mj.send_control("act_ARTx", position_ref[0])
        self.mj.send_control("act_ARTy", position_ref[1])
        self.mj.send_control("act_ARTz", position_ref[2])
        self.mj.send_control("act_ARRx", rpy_ref[0])
        self.mj.send_control("act_ARRy", rpy_ref[1])
        self.mj.send_control("act_ARRz", rpy_ref[2])

        # save to buffer
        if "ARTx" in self.tracked_joint_states_ref:
            self.tracked_joint_states_ref["ARTx"] = position_ref[0]
        if "ARTy" in self.tracked_joint_states_ref:
            self.tracked_joint_states_ref["ARTy"] = position_ref[1]
        if "ARTz" in self.tracked_joint_states_ref:
            self.tracked_joint_states_ref["ARTz"] = position_ref[2]
        if "ARRx" in self.tracked_joint_states_ref:
            self.tracked_joint_states_ref["ARRx"] = rpy_ref[0]
        if "ARRy" in self.tracked_joint_states_ref:
            self.tracked_joint_states_ref["ARRy"] = rpy_ref[1]
        if "ARRz" in self.tracked_joint_states_ref:
            self.tracked_joint_states_ref["ARRz"] = rpy_ref[2]

    def joint_state_callback(self, msg: JointState, verbose=False):
        """Callback function for the joint state subscriber."""
        if verbose:
            self.get_logger().info(f"Received Joint States: {msg}")

        for name, pos in zip(msg.name, msg.position):
            # send control
            act_name = f"act_{name}"
            if act_name in self.mj.actuator_names:
                self.mj.send_control(act_name, pos)
            # save to buffer
            if name in self.tracked_joint_names:
                self.tracked_joint_states_ref[name] = pos

    def output_data(self):
        """Output the data to the specified formats."""
        joint_pos = self.mj.data.qpos[self.tracked_joint_ids]
        joint_vel = self.mj.data.qvel[self.tracked_joint_ids]
        body_pos = self.mj.data.xpos[self.tracked_body_ids]
        body_quat = self.mj.data.xquat[self.tracked_body_ids]

        if "ros" in self.output_formats:
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = self.tracked_joint_names
            joint_state_msg.position = list(joint_pos)
            joint_state_msg.velocity = list(joint_vel)
            self.joint_state_publisher.publish(joint_state_msg)

            body_pose_msg = PoseArray()
            body_pose_msg.header.stamp = self.get_clock().now().to_msg()
            for pos, quat in zip(body_pos, body_quat):
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = pos
                (
                    pose.orientation.w,
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                ) = quat
                body_pose_msg.poses.append(pose)
            self.body_pose_publisher.publish(body_pose_msg)

        if "csv" in self.output_formats:
            self.csv_buffer.append(
                [
                    self.get_clock().now().nanoseconds,
                    *joint_pos,
                    *joint_vel,
                    *body_pos,
                    *body_quat,
                ]
            )
            if len(self.csv_buffer) >= 100:
                with open(self.csv_path, "a") as f:
                    for row in self.csv_buffer:
                        f.write(",".join(map(str, row)) + "\n")
                self.csv_buffer.clear()

        if "mp4" in self.output_formats:
            if time.time() - self.last_video_frame_time >= 1.0 / self.fps:
                frame = self.mj.render_frame()
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                self.video_writer.write(frame_bgr)
                self.last_video_frame_time = time.time()



    def run_flask(self):
        """Run the Flask server for video streaming."""
        return
        @self.app.route("/video")
        def video():
            return Response(
                self.mj.generate_encoded_frames(),
                mimetype="multipart/x-mixed-replace; boundary=frame",
            )

        @self.app.route("/shutdown", methods=["POST"])
        def shutdown():
            func = requests.environ.get("werkzeug.server.shutdown")
            if func is None:
                raise RuntimeError("Not running with the Werkzeug Server")
            func()
            return "Server shutting down"

        self.app.run(host="0.0.0.0", port=5000, threaded=True)

    def on_shutdown(self):
        """Clean up the node."""
        if self.rosbag_process is not None:
            os.killpg(os.getpgid(self.rosbag_process.pid), signal.SIGINT)
        if self.video_writer is not None:
            self.video_writer.release()
        requests.post("http://localhost:5000/shutdown")
        self.get_logger().info("Simulation stopped.")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "model_path", type=str, help="Path to the Mujoco model XML file"
    )
    parser.add_argument(
        "--config-yaml",
        type=str,
        help="Path to the YAML file containing the configuration",
        required=False,
    )
    parser.add_argument(
        "--replay-csv", default=None, type=str, help="Path to the CSV file to replay."
    )
    parser.add_argument(
        "--hand-pose-topic", default=None, type=str, help="Topic name for hand pose."
    )
    parser.add_argument(
        "--position-magnifiers",
        type=float_list,
        default=[2.5, 2.0, 0.8],
        help="Position magnifiers for the hand pose control",
    )
    parser.add_argument(
        "--output-formats", type=str, nargs="*", default=["ros"], help="Output formats."
    )
    parser.add_argument(
        "--output-csv-path", type=str, default=None, help="Path to the output CSV file."
    )
    parser.add_argument(
        "--output-bag-path", type=str, default=None, help="Path to the output bag file."
    )
    parser.add_argument(
        "--output-mp4-path", type=str, default=None, help="Path to the output MP4 file."
    )
    parser.add_argument(
        "--additional-bag-topics",
        type=str,
        nargs="+",
        default=[],
        help="Additional topics to record in the bag file (other than those related to mujoco itself).",
    )
    parser.add_argument("--enable-vr", action="store_true", help="Enable VR mode")
    parser.add_argument("--seed", type=int, default=0, help="Seed for the simulation")
    args = parser.parse_args()

    rclpy.init()

    node = MujocoJointController(
        model_path=args.model_path,
        config_yaml=args.config_yaml,
        replay_csv=args.replay_csv,
        hand_pose_topic=args.hand_pose_topic,
        position_magnifiers=args.position_magnifiers,
        output_formats=args.output_formats,
        output_csv_path=args.output_csv_path,
        output_bag_path=args.output_bag_path,
        output_mp4_path=args.output_mp4_path,
        additional_bag_topics=args.additional_bag_topics,
        enable_vr=args.enable_vr,
        seed=args.seed,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.error("KeyboardInterrupt, shutting down MujocoJointController...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
