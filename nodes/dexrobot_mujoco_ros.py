from ros_compat import ROSNode
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Float32MultiArray, Float64MultiArray
from std_srvs.srv import Trigger
import cv2
import mujoco

import time, os
import argparse
from loguru import logger
import numpy as np
import yaml
import subprocess
import pandas as pd
from scipy.spatial.transform import Rotation as R
from flask import Flask, Response, request
import requests
import signal
from threading import Thread
import cv2
from dexrobot_mujoco.utils.mj_control_vr_utils import MJControlVRWrapper
from dexrobot_mujoco.utils.angle_utils import adjust_angles

def float_list(x):
    return np.array(list(map(float, x.split(","))))


class MujocoJointController(ROSNode):
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
        renderer_dimension=None,
        seed=0,
        enable_ts_sensor=False,
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
                positions from the topic "joint_commands". When set, this node will subscribe to hand pose from the
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
            renderer_dimension (tuple): Renderer dimension as width,height (e.g. 640,480).
            enable_ts_sensor (bool): Whether to enable TS sensor data publishing. When enabled, will publish to 
                /left_hand/touch_sensors and/or /right_hand/touch_sensors topics.
        """
        super().__init__("mujoco_joint_controller")
        self.logger.info("Mujoco Joint Controller Node has been started.")

        self.app = Flask(__name__)
        self.model_path = model_path
        self.config_yaml = config_yaml
        self.replay_csv = replay_csv
        self.hand_pose_topic = hand_pose_topic
        self.position_magnifiers = position_magnifiers
        self.output_formats = output_formats
        self.output_csv_path = output_csv_path
        self.output_mp4_path = output_mp4_path
        self.output_bag_path = output_bag_path
        self.enable_vr = enable_vr
        self.enable_ts_sensor = enable_ts_sensor

        # Load Mujoco model
        self.mj = MJControlVRWrapper(
            os.path.join(os.getcwd(), self.model_path),
            enable_vr=self.enable_vr,
            renderer_dimension=renderer_dimension if renderer_dimension else ((640, 480) if "mp4" in self.output_formats else None),
            seed=seed,
        )

        # Determine hand type(s) from actual bodies in the loaded model
        self.hand_types = []
        
        # Check for left and right hand bodies in the model
        left_hand_found = False
        right_hand_found = False
        
        for i in range(self.mj.model.nbody):
            body_name = mujoco.mj_id2name(self.mj.model, mujoco.mjtObj.mjOBJ_BODY, i)
            if body_name:
                body_name_lower = body_name.lower()
                if 'left_hand' in body_name_lower or 'l_f_link' in body_name_lower:
                    left_hand_found = True
                elif 'right_hand' in body_name_lower or 'r_f_link' in body_name_lower:
                    right_hand_found = True
        
        if left_hand_found:
            self.hand_types.append('left')
        if right_hand_found:
            self.hand_types.append('right')
        
        # Log detected hands
        if self.hand_types:
            self.logger.info(f"Detected hands in model: {', '.join(self.hand_types)}")
        else:
            self.logger.warning("No hand bodies detected in model")

        # adjust initial pos and camera pose
        if self.config_yaml is not None:
            self.mj.parse_yaml(self.config_yaml)
        self.mj.launch_viewer("passive")

        # timer for simulation
        self.sim_timer = self.create_timer(0.01, self.forward_sim)

        # timer for VR images
        if self.enable_vr:
            self.vr_timer = self.create_timer(0.05, self.mj.update_vr_images)
            # Start Flask server thread
            self.flask_thread = Thread(target=self.run_flask)
            self.flask_thread.start()

        # Add screenshot service
        self.screenshot_service = self.create_service(
            Trigger, 'save_screenshot', self.save_screenshot_callback
        )

        if self.replay_csv is None:
            self.joint_state_subscription = self.create_subscription(
                JointState, "joint_commands", self.joint_state_callback, 10
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
            self.replay_csv = pd.read_csv(self.replay_csv)
            self.replay_timer = self.create_timer(0.1, self.forward_replay)
            self.joint_step_counter = 0

        self.hand_position_offset = None

        # joints/bodies names/ids to output, and buffer for the joint states reference
        if self.config_yaml is not None:
            with open(self.config_yaml, "r") as f:
                config = yaml.safe_load(f)
            joint_names_to_track = [
                item for sublist in config.get("tracked_joints", []) for item in sublist
            ]
            logger.info(f"Tracked joints: {joint_names_to_track}")
            self.tracked_joint_names = [
                name
                for name in joint_names_to_track
                if self.mj.get_joint_id(name) != -1
            ]
            body_names_to_track = [
                item for sublist in config.get("tracked_bodies", []) for item in sublist
            ]
            self.tracked_body_names = [
                name for name in body_names_to_track if self.mj.get_link_id(name) != -1
            ]
            sensor_names_to_track = [
                item for sublist in config.get("tracked_sensors", []) for item in sublist
            ]
            self.tracked_sensor_names = sensor_names_to_track
        else:
            self.tracked_joint_names = []
            self.tracked_body_names = []
            self.tracked_sensor_names = []
        self.tracked_joint_ids = [
            self.mj.get_joint_id(name) for name in self.tracked_joint_names
        ]
        self.tracked_joint_qpos_idx = [
            self.mj.get_qpos_addr(name) for name in self.tracked_joint_names
        ]
        self.tracked_joint_qvel_idx = [
            self.mj.get_qvel_addr(name) for name in self.tracked_joint_names
        ]
        self.tracked_body_ids = [
            self.mj.get_link_id(name) for name in self.tracked_body_names
        ]
        self.tracked_joint_states_ref = {name: 0.0 for name in self.tracked_joint_names}

        # Setup TS sensor mapping if enabled
        if self.enable_ts_sensor:
            self.logger.info("TS sensor publishing enabled")
            self._setup_ts_sensor_mapping()
        else:
            self.logger.info("TS sensor publishing disabled")

        # timer, publisher and buffer for publishing / logging data
        if self.output_formats:
            self.output_timer = self.create_timer(0.01, self.output_data)
        else:
            self.output_timer = None
        if "ros" in self.output_formats:
            self.joint_state_publisher = self.create_publisher(
                JointState, "joint_states", 10
            )
            self.body_pose_publisher = self.create_publisher(
                PoseArray, "body_poses", 10
            )
            self.touch_sensor_publisher = self.create_publisher(
                Float32MultiArray, "touch_sensors", 10
            )
            
            # Setup hand-specific touch sensor publishers
            self.ts_sensor_publishers = {}
            for hand_type in self.hand_types:
                topic_name = f"/{hand_type}_hand/touch_sensors"
                self.ts_sensor_publishers[hand_type] = self.create_publisher(
                    Float64MultiArray, topic_name, 10
                )
                sensor_type = "TS" if self.enable_ts_sensor else "MuJoCo"
                self.logger.info(f"Created {sensor_type} sensor publisher for topic: {topic_name}")
        else:
            self.joint_state_publisher = None
            self.body_pose_publisher = None
            self.touch_sensor_publisher = None
            self.ts_sensor_publishers = {}
        if "csv" in self.output_formats:
            if self.output_csv_path is not None:
                self.csv_columns = [
                    'timestamp',
                    *[f'{name}_pos' for name in self.tracked_joint_names],
                    *[f'{name}_vel' for name in self.tracked_joint_names],
                    *[f'{name}_pos' for name in self.tracked_body_names],
                    *[f'{name}_quat' for name in self.tracked_body_names],
                    *[f'{name}' for name in self.tracked_sensor_names],
                ]
                self.csv_data_count = 0
                self.csv_buffer = pd.DataFrame(columns=self.csv_columns)
                self.csv_path = self.output_csv_path
            else:
                self.logger.error(
                    'Output CSV path must be specified when "csv" is included in the output formats.'
                )
                exit(1)
        else:
            self.csv_columns = None
            self.csv_data_count = 0
            self.csv_buffer = None
            self.csv_path = None

        if "ros" in output_formats and output_bag_path is not None:
            topics = [
                "joint_commands",
                "joint_states",
                "body_poses",
            ] + additional_bag_topics
            command = f'ros2 bag record -o {output_bag_path} {" ".join(topics)}'
            self.rosbag_process = subprocess.Popen(command, shell=True, preexec_fn=os.setpgrp)
            self.logger.info(f"Recording to bag file: {output_bag_path}")
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

        # save previous angle measurements
        self.rpy_prev = np.zeros(3)

    def forward_sim(self):
        """Forward the simulator until the current time."""
        current_time = time.time() - self.mj.start_time
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
            if time.time() - self.mj.start_time < 2.0:
                return
            self.hand_position_offset = raw_position_ref

        position_ref = self.position_magnifiers * (raw_position_ref - self.hand_position_offset)
        rpy_ref = R.from_quat(orientation_ref, scalar_first=True).as_euler('XYZ')
        rpy_ref = adjust_angles(rpy_ref, self.rpy_prev)
        self.rpy_prev = rpy_ref

        if verbose:
            self.logger.info(
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
            self.logger.info(f"Received Joint States: {msg}")

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
        joint_pos = self.mj.data.qpos[self.tracked_joint_qpos_idx]
        joint_vel = self.mj.data.qvel[self.tracked_joint_qvel_idx]
        body_pos = self.mj.data.xpos[self.tracked_body_ids]
        body_quat = self.mj.data.xquat[self.tracked_body_ids]
        all_sensor_data = []
        for sensor_name in self.tracked_sensor_names:
            sensor_data = self.mj.data.sensor(sensor_name).data.astype(np.double)
            all_sensor_data.extend(sensor_data)

        if "ros" in self.output_formats:
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_ros_time().to_msg()
            joint_state_msg.name = self.tracked_joint_names
            joint_state_msg.position = list(joint_pos)
            joint_state_msg.velocity = list(joint_vel)
            self.joint_state_publisher.publish(joint_state_msg)

            body_pose_msg = PoseArray()
            body_pose_msg.header.stamp = self.get_ros_time().to_msg()
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

            touch_data_array = Float32MultiArray()
            touch_data_array.data = all_sensor_data
            self.touch_sensor_publisher.publish(touch_data_array)

            # Publish TS sensor data if enabled, otherwise publish normal touch sensor data
            if self.enable_ts_sensor:
                self._publish_ts_sensor_data()
            else:
                # Publish normal MuJoCo touch sensor data to hand-specific topics
                self._publish_mujoco_touch_sensor_data()

        if 'csv' in self.output_formats:
            self.csv_buffer.loc[self.csv_data_count] = [self.get_clock().now().nanoseconds, *joint_pos, *joint_vel, *body_pos, *body_quat, *all_sensor_data]
            self.csv_data_count += 1

        if "mp4" in self.output_formats:
            if time.time() - self.last_video_frame_time >= 1.0 / self.fps:
                frame = self.mj.render_frame()
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                self.video_writer.write(frame_bgr)
                self.last_video_frame_time = time.time()

    def save_screenshot_callback(self, request, response):
        """Service callback to save the current viewer frame as an image."""
        try:
            frame = self.mj.render_frame()
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            timestamp = self.get_ros_time().now()
            filename = f"mujoco_screenshot_{timestamp}.png"
            cv2.imwrite(filename, frame_bgr)

            response.success = True
            response.message = f"Screenshot saved as {filename}"
        except Exception as e:
            response.success = False
            response.message = f"Failed to save screenshot: {str(e)}"

        return response

    def run_flask(self):
        """Run the Flask server for video streaming."""
        @self.app.route("/video")
        def video():
            return Response(
                self.mj.generate_encoded_frames(),
                mimetype="multipart/x-mixed-replace; boundary=frame",
            )

        @self.app.route("/shutdown", methods=["POST"])
        def shutdown():
            shutdown_server = request.environ.get("werkzeug.server.shutdown")
            if shutdown_server is None:
                raise RuntimeError("Not running with the Werkzeug Server")
            shutdown_server()
            return "Server shutting down..."

        self.app.run(host="0.0.0.0", port=5000, threaded=True)

    def on_shutdown(self):
        """Clean up the node."""
        if self.csv_buffer is not None:
            self.csv_buffer.to_csv(self.csv_path)
            self.logger.info(f"Saved data to {self.csv_path}")
        if self.rosbag_process is not None:
            os.killpg(os.getpgid(self.rosbag_process.pid), signal.SIGINT)
            self.logger.info("Stopped recording to bag file.")
        if self.video_writer is not None:
            self.video_writer.release()
            self.logger.info(f"Saved video to {self.output_mp4_path}")
        if self.enable_vr:
            requests.post("http://127.0.0.1:5000/shutdown")
            self.logger.info("Stopped the Flask server.")
        self.logger.info('Simulation stopped.')


    def _setup_ts_sensor_mapping(self):
        """Setup TS sensor mapping for available hands."""
        self.ts_sensor_mapping = {}
        
        # Detect which hands actually have TS sensors in the model
        available_hand_types = []
        for hand_type in self.hand_types:
            hand_prefix = 'l' if hand_type == 'left' else 'r'
            test_sensor_name = f"TS-F-A-{hand_prefix}_f_link1_pad"
            try:
                self.mj.model.sensor(test_sensor_name).id
                available_hand_types.append(hand_type)
                self.logger.info(f"Found TS sensors for {hand_type} hand")
            except:
                self.logger.info(f"No TS sensors found for {hand_type} hand")
        
        # Setup TS sensor mapping for hands with TS sensors
        for hand_type in available_hand_types:
            hand_prefix = 'l' if hand_type == 'left' else 'r'
            self.ts_sensor_mapping[hand_type] = []
            
            # Collect all TS sensors for this hand (5 fingers)
            for finger_num in range(1, 6):
                ts_sensor_name = f"TS-F-A-{hand_prefix}_f_link{finger_num}_pad"
                rf_sensor_name = f"rf_{hand_prefix}_f_link{finger_num}_pad"
                
                try:
                    ts_sensor_id = self.mj.model.sensor(ts_sensor_name).id
                    rf_sensor_id = self.mj.model.sensor(rf_sensor_name).id
                    
                    self.ts_sensor_mapping[hand_type].append({
                        'ts_sensor_id': ts_sensor_id,
                        'ts_sensor_addr': self.mj.model.sensor_adr[ts_sensor_id],
                        'ts_sensor_dim': 11,  # TS sensor has 11 dimensions
                        'rf_sensor_id': rf_sensor_id
                    })
                except Exception as e:
                    self.logger.warning(f"Could not find TS sensors for {hand_type} hand finger {finger_num}: {e}")
        
        # Update hand_types to only include hands with TS sensors if TS is enabled
        if self.enable_ts_sensor:
            self.hand_types = available_hand_types

    def _publish_ts_sensor_data(self):
        """Publish TS sensor data in dextactisim format (21 dimensions total)."""
        for hand_type in self.hand_types:
            if hand_type not in self.ts_sensor_mapping:
                continue
                
            # Replicate dextactisim format exactly:
            # sdata = Data.sensordata[[1,2, 12,13, 23,24, 34,35, 45,46] + user1_data_id_range]
            # Total: 10 fixed indices + 11 TS sensor values = 21 dimensions
            
            all_sensor_data = []
            
            # Part 1: 10 elements from fixed indices [1,2, 12,13, 23,24, 34,35, 45,46]
            # These correspond to force data from multiple fingers
            fixed_indices = [1, 2, 12, 13, 23, 24, 34, 35, 45, 46]
            for idx in fixed_indices:
                if idx < len(self.mj.data.sensordata):
                    all_sensor_data.append(float(self.mj.data.sensordata[idx]))
                else:
                    all_sensor_data.append(0.0)  # Safety fallback
            
            # Part 2: 11 elements from first TS sensor (complete TS-F-A data)
            # [0] Proximity, [1] Normal force, [2] Tangential force, [3] Direction, [4-10] Capacitance F1-F7
            if self.ts_sensor_mapping[hand_type]:
                first_ts_info = self.ts_sensor_mapping[hand_type][0]
                ts_addr = first_ts_info['ts_sensor_addr']
                ts_dim = first_ts_info['ts_sensor_dim']  # Should be 11
                
                # Get all 11 TS sensor values
                for i in range(ts_dim):
                    ts_value = self.mj.data.sensordata[ts_addr + i]
                    all_sensor_data.append(float(ts_value))
            else:
                # Safety fallback: add 11 zeros if no TS sensor found
                all_sensor_data.extend([0.0] * 11)
            
            # Total should be 21 elements (10 + 11)
            # Create Float64MultiArray message
            touch_msg = Float64MultiArray()
            touch_msg.data = all_sensor_data
            
            # Publish the message
            if hand_type in self.ts_sensor_publishers:
                self.ts_sensor_publishers[hand_type].publish(touch_msg)
    
    def _publish_mujoco_touch_sensor_data(self):
        """Publish normal MuJoCo touch sensor data to hand-specific topics."""
        # Get normal MuJoCo sensor data that's already collected
        all_sensor_data = []
        for sensor_name in self.tracked_sensor_names:
            sensor_data = self.mj.data.sensor(sensor_name).data.astype(np.double)
            all_sensor_data.extend(sensor_data)
        
        # Publish to each hand's topic if sensor data is available
        if all_sensor_data:
            for hand_type in self.hand_types:
                touch_msg = Float64MultiArray()
                touch_msg.data = all_sensor_data
                
                if hand_type in self.ts_sensor_publishers:
                    self.ts_sensor_publishers[hand_type].publish(touch_msg)

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
    parser.add_argument(
        "--renderer-dimension",
        type=float_list,
        default=None,
        help="Renderer dimension as width,height (e.g. 640,480)",
    )
    parser.add_argument("--seed", type=int, default=0, help="Seed for the simulation")
    parser.add_argument("--enable-ts-sensor", action="store_true", help="Enable TS sensor data publishing to /left_hand/touch_sensors and /right_hand/touch_sensors topics")
    args = parser.parse_args()

    # Convert renderer dimension string to tuple or None
    renderer_dim = tuple(map(int, args.renderer_dimension)) if args.renderer_dimension is not None else None

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
        renderer_dimension=renderer_dim,
        seed=args.seed,
        enable_ts_sensor=args.enable_ts_sensor,
    )

    try:
        node.spin()
    except KeyboardInterrupt:
        logger.error("KeyboardInterrupt, shutting down MujocoJointController...")
    finally:
        node.on_shutdown()
        node.shutdown()


if __name__ == "__main__":
    main()
