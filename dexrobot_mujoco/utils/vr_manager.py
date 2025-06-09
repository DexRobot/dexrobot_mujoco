"""VR manager for MuJoCo simulations."""

import time
import cv2
import numpy as np
from loguru import logger
import mujoco


class VRManager:
    """Manager for VR functionality in MuJoCo simulations."""
    
    def __init__(self, mj_wrapper):
        """Initialize the VR manager.
        
        Args:
            mj_wrapper: MJControlWrapper instance
        """
        self.mj = mj_wrapper
        self.latest_frame = None
        
        # VR image dimensions (from original implementation)
        self.width = 1920
        self.height = 1080
        
        # Update model visualization settings
        self.mj.model.vis.global_.offwidth = self.width
        self.mj.model.vis.global_.offheight = self.height
        
        # Initialize renderer for VR
        self.renderer = mujoco.Renderer(
            self.mj.model, 
            height=self.height, 
            width=self.width
        )
        
        # Stereo cameras for VR
        self.left_camera = mujoco.MjvCamera()
        self.right_camera = mujoco.MjvCamera()
        
        logger.info("VR manager initialized")
    
    def sync_camera_pose_for_vr(self, offset=0.1):
        """Sync the camera pose for VR stereo rendering.
        
        Args:
            offset: Horizontal offset between left and right cameras
        """
        if self.mj.viewer is None:
            return
            
        lookat, azimuth, elevation, distance = (
            self.mj.viewer.cam.lookat,
            self.mj.viewer.cam.azimuth,
            self.mj.viewer.cam.elevation,
            self.mj.viewer.cam.distance,
        )
        delta_azimuth = np.arctan(offset / 2 / (distance * np.cos(np.deg2rad(elevation))))

        self.left_camera.lookat = lookat
        self.left_camera.elevation = elevation
        self.left_camera.distance = np.sqrt(distance**2 + offset**2 / 4)
        self.left_camera.azimuth = azimuth + delta_azimuth

        self.right_camera.lookat = lookat
        self.right_camera.elevation = elevation
        self.right_camera.distance = np.sqrt(distance**2 + offset**2 / 4)
        self.right_camera.azimuth = azimuth - delta_azimuth
    
    def update_vr_images(self):
        """Update VR stereo images."""
        self.sync_camera_pose_for_vr()
        
        # Render left eye view
        self.renderer.update_scene(self.mj.data, camera=self.left_camera)
        image_1 = self.renderer.render()
        
        # Render right eye view
        self.renderer.update_scene(self.mj.data, camera=self.right_camera)
        image_2 = self.renderer.render()
        
        # Create side-by-side stereo image
        combined_image = np.concatenate((image_1, image_2), axis=1)
        combined_image = cv2.cvtColor(combined_image, cv2.COLOR_RGB2BGR)
        self.latest_frame = combined_image
    
    def generate_encoded_frames(self):
        """Generate encoded frames for streaming.
        
        Yields:
            bytes: Encoded JPEG frames for streaming
        """
        while True:
            if self.latest_frame is not None:
                ret, buffer = cv2.imencode(".jpg", self.latest_frame)
                frame = buffer.tobytes()
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
                )
            time.sleep(0.05)  # Frame rate control