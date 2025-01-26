================
VR Visualization
================

This section covers the VR visualization capabilities of the DexRobot MuJoCo node.

Overview
-------

The VR visualization system provides:

- Stereoscopic rendering
- Web-based visualization
- Real-time streaming
- Adjustable viewpoints

Configuration
-----------

Basic Setup
^^^^^^^^^

Enable VR visualization:

.. code-block:: bash

    python nodes/dexrobot_mujoco_ros.py model.xml \
        --enable-vr \
        --renderer-dimension 1920,1080

Parameters:
- ``enable-vr``: Activates VR mode
- ``renderer-dimension``: Resolution (width,height)

Default Settings:
- Frame rate: 20 FPS
- Server port: 5000
- Stereo offset: 0.1m

Implementation Details
------------------

VR Wrapper Class
^^^^^^^^^^^^^
The VR functionality is implemented in ``MJControlVRWrapper``:

.. code-block:: python

    class MJControlVRWrapper(MJControlWrapper):
        def __init__(self, model_path, enable_vr=True, **kwargs):
            super().__init__(model_path, **kwargs)
            self.enable_vr = enable_vr
            if self.enable_vr:
                self.width = 1920
                self.height = 1080
                self.renderer = mujoco.Renderer(
                    self.model,
                    height=self.height,
                    width=self.width
                )
                self.left_camera = mujoco.MjvCamera()
                self.right_camera = mujoco.MjvCamera()

Camera System
^^^^^^^^^^

Camera synchronization:

.. code-block:: python

    def sync_camera_pose_for_vr(self, offset=0.1):
        """Sync the camera pose for VR.

        Args:
            offset: Stereo camera separation (meters)
        """
        lookat, azimuth, elevation, distance = (
            self.viewer.cam.lookat,
            self.viewer.cam.azimuth,
            self.viewer.cam.elevation,
            self.viewer.cam.distance,
        )

        # Calculate stereo camera positions
        delta_azimuth = np.arctan(
            offset / 2 / (distance*np.cos(np.deg2rad(elevation)))
        )

        # Configure cameras
        self.left_camera.lookat = lookat
        self.left_camera.elevation = elevation
        self.left_camera.distance = np.sqrt(
            distance**2 + offset**2 / 4
        )
        self.left_camera.azimuth = azimuth + delta_azimuth

        self.right_camera.lookat = lookat
        self.right_camera.elevation = elevation
        self.right_camera.distance = np.sqrt(
            distance**2 + offset**2 / 4
        )
        self.right_camera.azimuth = azimuth - delta_azimuth

Image Generation
^^^^^^^^^^^^^

Update VR images:

.. code-block:: python

    def update_vr_images(self):
        """Update the stereo images for VR."""
        if self.enable_vr:
            # Sync camera poses
            self.sync_camera_pose_for_vr()

            # Render left eye
            self.renderer.update_scene(
                self.data,
                camera=self.left_camera
            )
            image_1 = self.renderer.render()

            # Render right eye
            self.renderer.update_scene(
                self.data,
                camera=self.right_camera
            )
            image_2 = self.renderer.render()

            # Create combined view
            blended_image = cv2.addWeighted(
                image_1, 0.5,
                image_2, 0.5,
                0
            )
            blended_image = cv2.cvtColor(
                blended_image,
                cv2.COLOR_RGB2BGR
            )

            # Store for streaming
            self.latest_frame = blended_image

Web Streaming
-----------

Flask Server
^^^^^^^^^^

The visualization is served through Flask:

.. code-block:: python

    def run_flask(self):
        """Run the Flask server for video streaming."""
        @self.app.route("/video")
        def video():
            return Response(
                self.generate_encoded_frames(),
                mimetype="multipart/x-mixed-replace; boundary=frame"
            )

        @self.app.route("/shutdown", methods=["POST"])
        def shutdown():
            shutdown_server = request.environ.get(
                "werkzeug.server.shutdown"
            )
            if shutdown_server is None:
                raise RuntimeError(
                    "Not running with the Werkzeug Server"
                )
            shutdown_server()
            return "Server shutting down..."

        self.app.run(host="0.0.0.0", port=5000, threaded=True)

Frame Generation
^^^^^^^^^^^^^

Continuous frame streaming:

.. code-block:: python

    def generate_encoded_frames(self):
        """Generate encoded frames for streaming."""
        while True:
            if self.latest_frame is not None:
                ret, buffer = cv2.imencode(".jpg", self.latest_frame)
                frame = buffer.tobytes()
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n"
                    + frame + b"\r\n"
                )
            time.sleep(0.05)  # 20 FPS

Usage
----

Access Visualization
^^^^^^^^^^^^^^^^^

1. Launch node with VR enabled:

   .. code-block:: bash

       python nodes/dexrobot_mujoco_ros.py model.xml --enable-vr

2. Access visualization:
   - Open browser to ``http://localhost:5000/video``
   - Or embed in custom interface:

     .. code-block:: html

         <img src="http://localhost:5000/video" />

3. Shutdown:
   - Use Ctrl+C in terminal
   - Or send POST to ``/shutdown``

Best Practices
------------

Performance
^^^^^^^^^
1. Monitor frame rate
2. Adjust resolution as needed
3. Consider network bandwidth
4. Optimize image encoding

Resource Management
^^^^^^^^^^^^^^^^
1. Clean up server on exit
2. Monitor memory usage
3. Handle connection errors
4. Implement timeouts

Camera Configuration
^^^^^^^^^^^^^^^^^
1. Adjust stereo offset for comfort
2. Set appropriate view distances
3. Configure camera angles
4. Consider field of view

Next Steps
---------

- Review :doc:`data_recording` for recording VR sessions
- Check the API reference for detailed class documentation
