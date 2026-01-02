"""
Main Spatial AI Playground Application

Uses Rerun for visualization and C++ core for heavy computation.
"""

import cv2
import numpy as np
from typing import Optional, Dict, Any
from pathlib import Path

try:
    import rerun as rr
    HAS_RERUN = True
except ImportError:
    HAS_RERUN = False
    print("Warning: Rerun not installed. Visualization disabled.")
    print("Install with: pip install rerun-sdk")

try:
    from . import _core as cpp
    HAS_CPP = True
except ImportError:
    HAS_CPP = False
    print("Warning: C++ core not built. Using Python-only mode.")


class SpatialPlayground:
    """Main application class for Spatial AI experiments."""

    def __init__(
        self,
        app_name: str = "Spatial AI Playground",
        config: Optional[Dict[str, Any]] = None
    ):
        self.app_name = app_name
        self.config = config or {}
        self.camera = None
        self.tracker = None
        self.frame_count = 0
        self.running = False

        # Depth estimator (Python-based)
        self.depth_estimator = None

        # Initialize Rerun
        if HAS_RERUN:
            rr.init(app_name, spawn=True)

    def setup_camera(self, camera_type: str = "webcam", **kwargs) -> bool:
        """Set up camera input source."""
        if camera_type == "webcam":
            if HAS_CPP:
                self.camera = cpp.WebcamCamera()
                config_str = kwargs.get("config", "0")
                return self.camera.open(config_str)
            else:
                # Python fallback
                device_id = kwargs.get("device_id", 0)
                self.camera = cv2.VideoCapture(device_id)
                return self.camera.isOpened()
        return False

    def setup_tracker(self, tracker_type: str = "simple_vo") -> bool:
        """Set up SLAM tracker."""
        if HAS_CPP:
            self.tracker = cpp.create_tracker(tracker_type)
            return self.tracker is not None
        else:
            print("C++ core not available. Tracker disabled.")
            return False

    def setup_depth_estimator(self, model_name: str = "depth_anything_v2"):
        """Set up monocular depth estimation model."""
        try:
            if model_name == "depth_anything_v2":
                from transformers import pipeline
                self.depth_estimator = pipeline(
                    task="depth-estimation",
                    model="depth-anything/Depth-Anything-V2-Small-hf"
                )
                print(f"Loaded depth estimator: {model_name}")
            else:
                print(f"Unknown depth estimator: {model_name}")
        except Exception as e:
            print(f"Failed to load depth estimator: {e}")

    def get_frame(self):
        """Get frame from camera."""
        if HAS_CPP and hasattr(self.camera, 'get_frame'):
            return self.camera.get_frame()
        elif self.camera is not None:
            ret, frame = self.camera.read()
            if ret:
                return {"rgb": frame, "valid": True}
        return {"valid": False}

    def estimate_depth(self, rgb: np.ndarray) -> Optional[np.ndarray]:
        """Estimate depth from RGB image."""
        if self.depth_estimator is None:
            return None

        try:
            from PIL import Image
            img = Image.fromarray(cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB))
            result = self.depth_estimator(img)
            depth = np.array(result["depth"])
            return depth
        except Exception as e:
            print(f"Depth estimation failed: {e}")
            return None

    def log_to_rerun(
        self,
        frame: np.ndarray,
        pose: Optional[np.ndarray] = None,
        depth: Optional[np.ndarray] = None,
        points: Optional[np.ndarray] = None
    ):
        """Log data to Rerun visualization."""
        if not HAS_RERUN:
            return

        # Log camera image
        rr.log("world/camera/image", rr.Image(frame[..., ::-1]))  # BGR to RGB

        # Log camera pose
        if pose is not None:
            translation = pose[:3, 3]
            rotation = pose[:3, :3]
            rr.log(
                "world/camera",
                rr.Transform3D(translation=translation, mat3x3=rotation)
            )

        # Log depth
        if depth is not None:
            rr.log("world/camera/depth", rr.DepthImage(depth))

        # Log point cloud
        if points is not None and len(points) > 0:
            positions = np.array([p.position for p in points])
            colors = np.array([p.color for p in points])
            rr.log(
                "world/points",
                rr.Points3D(positions, colors=colors)
            )

    def run_once(self) -> bool:
        """Process one frame. Returns False if should stop."""
        frame_data = self.get_frame()

        if not frame_data.get("valid", False):
            return True  # Continue trying

        if HAS_CPP and hasattr(frame_data, 'rgb'):
            rgb = np.array(frame_data.rgb)
        else:
            rgb = frame_data.get("rgb")

        if rgb is None:
            return True

        # Track
        pose = None
        if self.tracker is not None and HAS_CPP:
            result = self.tracker.track(frame_data)
            pose = np.array(result.pose)

        # Estimate depth
        depth = None
        if self.depth_estimator is not None:
            depth = self.estimate_depth(rgb)

        # Visualize
        self.log_to_rerun(rgb, pose, depth)

        self.frame_count += 1

        # Check for quit (ESC key in OpenCV window)
        if cv2.waitKey(1) == 27:
            return False

        return True

    def run(self):
        """Main run loop."""
        print(f"Starting {self.app_name}...")
        self.running = True

        try:
            while self.running:
                if not self.run_once():
                    break
        except KeyboardInterrupt:
            print("\nStopped by user")
        finally:
            self.stop()

    def stop(self):
        """Stop and clean up."""
        self.running = False

        if HAS_CPP and hasattr(self.camera, 'close'):
            self.camera.close()
        elif self.camera is not None:
            self.camera.release()

        cv2.destroyAllWindows()
        print(f"Processed {self.frame_count} frames")


def main():
    """Entry point for command line."""
    import argparse

    parser = argparse.ArgumentParser(description="Spatial AI Playground")
    parser.add_argument("--camera", type=str, default="webcam", help="Camera type")
    parser.add_argument("--device", type=int, default=0, help="Camera device ID")
    parser.add_argument("--tracker", type=str, default="simple_vo", help="Tracker type")
    parser.add_argument("--depth", action="store_true", help="Enable depth estimation")
    args = parser.parse_args()

    app = SpatialPlayground()

    if not app.setup_camera(args.camera, device_id=args.device):
        print("Failed to open camera")
        return

    if args.tracker:
        app.setup_tracker(args.tracker)

    if args.depth:
        app.setup_depth_estimator()

    app.run()


if __name__ == "__main__":
    main()
