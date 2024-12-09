import mujoco
import numpy as np
import cv2
import time, threading
from .mj_control_utils import MJControlWrapper


class MJControlVRWrapper(MJControlWrapper):
    def __init__(self, model_path, enable_vr=True, **kwargs):
        super().__init__(model_path, **kwargs)
        self.enable_vr = enable_vr
        if self.enable_vr:
            self.latest_frame = None
            self.width = 1920
            self.height = 1080
            self.model.vis.global_.offwidth = 1920
            self.model.vis.global_.offheight = 1080

            self.renderer = mujoco.Renderer(
                self.model, height=self.height, width=self.width
            )
            self.left_camera = mujoco.MjvCamera()
            self.right_camera = mujoco.MjvCamera()

    def update_vr_images(self):
        if self.enable_vr:
            self.sync_camera_pose_for_vr()
            self.renderer.update_scene(self.data, camera=self.left_camera)
            image_1 = self.renderer.render()
            self.renderer.update_scene(self.data, camera=self.right_camera)
            image_2 = self.renderer.render()
            blended_image = cv2.addWeighted(image_1, 0.5, image_2, 0.5, 0)
            blended_image = cv2.cvtColor(blended_image, cv2.COLOR_RGB2BGR)
            combined_image = np.concatenate((image_1, image_2), axis=1)
            combined_image = cv2.cvtColor(combined_image, cv2.COLOR_RGB2BGR)
            self.latest_frame = combined_image

    def sync_camera_pose_for_vr(self, offset=0.1):
        """Sync the camera pose for VR."""
        lookat, azimuth, elevation, distance = (
            self.viewer.cam.lookat,
            self.viewer.cam.azimuth,
            self.viewer.cam.elevation,
            self.viewer.cam.distance,
        )
        delta_azimuth = np.arctan(offset / 2 / (distance*np.cos(np.deg2rad(elevation))))

        self.left_camera.lookat = lookat
        self.left_camera.elevation = elevation
        self.left_camera.distance = np.sqrt(distance**2 + offset**2 / 4)
        self.left_camera.azimuth = azimuth + delta_azimuth

        self.right_camera.lookat = lookat
        self.right_camera.elevation = elevation
        self.right_camera.distance = np.sqrt(distance**2 + offset**2 / 4)
        self.right_camera.azimuth = azimuth - delta_azimuth

    def start_simulation(self):
        """Starts the simulation in a separate thread."""
        self.simulation_thread = threading.Thread(target=self.run_simulation)
        self.simulation_thread.start()

    def run_simulation(self):
        """Simulation loop."""
        while self.running:
            self.step()  # Perform simulation steps

    def stop_simulation(self):
        """Stops the simulation."""
        self.running = False
        self.simulation_thread.join()

    def generate_encoded_frames(self):
        """Generate encoded frames for streaming."""
        while True:
            if self.latest_frame is not None:
                ret, buffer = cv2.imencode(".jpg", self.latest_frame)
                frame = buffer.tobytes()
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
                )
            time.sleep(0.05)  # Frame rate control
