"""
Monocular Gaussian SLAM Demo

Uses Depth Anything V2 for depth estimation and optical flow for tracking.
Accumulates 3D points as a stepping stone to full Gaussian splatting.
"""
import sys
sys.path.insert(0, 'build/Release')

import cv2
import numpy as np
import torch
from transformers import pipeline
from PIL import Image
import time

class SimpleTracker:
    """Simple optical flow based tracker"""
    def __init__(self):
        self.prev_gray = None
        self.prev_points = None
        self.pose = np.eye(4, dtype=np.float32)

    def track(self, gray, K):
        """Track camera motion, return 4x4 pose matrix"""
        if self.prev_gray is None:
            self.prev_gray = gray
            self.prev_points = cv2.goodFeaturesToTrack(gray, 500, 0.01, 10)
            return self.pose.copy(), False

        if self.prev_points is None or len(self.prev_points) < 8:
            self.prev_points = cv2.goodFeaturesToTrack(gray, 500, 0.01, 10)
            self.prev_gray = gray
            return self.pose.copy(), False

        # Track with optical flow
        curr_points, status, _ = cv2.calcOpticalFlowPyrLK(
            self.prev_gray, gray, self.prev_points, None
        )

        if curr_points is None:
            self.prev_gray = gray
            return self.pose.copy(), False

        # Filter good points
        good_old = self.prev_points[status == 1]
        good_new = curr_points[status == 1]

        if len(good_old) < 8:
            self.prev_points = cv2.goodFeaturesToTrack(gray, 500, 0.01, 10)
            self.prev_gray = gray
            return self.pose.copy(), False

        # Estimate essential matrix
        E, mask = cv2.findEssentialMat(good_new, good_old, K, cv2.RANSAC, 0.999, 1.0)

        if E is None:
            self.prev_gray = gray
            self.prev_points = good_new.reshape(-1, 1, 2)
            return self.pose.copy(), False

        # Recover pose
        _, R, t, mask = cv2.recoverPose(E, good_new, good_old, K)

        # Create delta transform
        delta = np.eye(4, dtype=np.float32)
        delta[:3, :3] = R.astype(np.float32)
        delta[:3, 3] = (t * 0.05).flatten().astype(np.float32)  # Scale factor

        self.pose = self.pose @ delta

        # Redetect if too few points
        if len(good_new) < 100:
            new_pts = cv2.goodFeaturesToTrack(gray, 500 - len(good_new), 0.01, 10)
            if new_pts is not None:
                self.prev_points = np.vstack([good_new.reshape(-1, 1, 2), new_pts])
            else:
                self.prev_points = good_new.reshape(-1, 1, 2)
        else:
            self.prev_points = good_new.reshape(-1, 1, 2)

        self.prev_gray = gray
        return self.pose.copy(), True


class GaussianMap:
    """Simple 3D point accumulator (proto-Gaussian map)"""
    def __init__(self, max_points=100000):
        self.points = []  # [N, 3] positions
        self.colors = []  # [N, 3] RGB colors
        self.max_points = max_points

    def add_frame(self, rgb, depth, pose, K, subsample=8):
        """Add points from a depth frame"""
        h, w = depth.shape

        # Create pixel grid (subsampled)
        u = np.arange(0, w, subsample)
        v = np.arange(0, h, subsample)
        u, v = np.meshgrid(u, v)
        u = u.flatten()
        v = v.flatten()

        # Get depth values
        d = depth[v, u]
        valid = (d > 0.1) & (d < 10.0)  # Valid depth range

        u, v, d = u[valid], v[valid], d[valid]

        if len(u) == 0:
            return

        # Unproject to 3D (camera frame)
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]

        x = (u - cx) * d / fx
        y = (v - cy) * d / fy
        z = d

        points_cam = np.stack([x, y, z], axis=1)

        # Transform to world frame
        R = pose[:3, :3]
        t = pose[:3, 3]
        points_world = (R @ points_cam.T).T + t

        # Get colors
        colors = rgb[v, u] / 255.0

        self.points.append(points_world)
        self.colors.append(colors)

        # Limit total points
        if self.get_point_count() > self.max_points:
            self._downsample()

    def _downsample(self):
        """Keep only max_points"""
        all_points = np.vstack(self.points)
        all_colors = np.vstack(self.colors)

        indices = np.random.choice(len(all_points), self.max_points, replace=False)
        self.points = [all_points[indices]]
        self.colors = [all_colors[indices]]

    def get_points(self):
        if not self.points:
            return np.zeros((0, 3)), np.zeros((0, 3))
        return np.vstack(self.points), np.vstack(self.colors)

    def get_point_count(self):
        return sum(len(p) for p in self.points)


def create_topdown_view(points, colors, pose, size=400, scale=50):
    """Create a top-down view of the point cloud and camera"""
    img = np.zeros((size, size, 3), dtype=np.uint8)
    center = size // 2

    if len(points) > 0:
        # Project points to XZ plane (top-down)
        x = (points[:, 0] * scale + center).astype(int)
        z = (points[:, 2] * scale + center).astype(int)

        # Filter to visible range
        valid = (x >= 0) & (x < size) & (z >= 0) & (z < size)
        x, z = x[valid], z[valid]
        c = (colors[valid] * 255).astype(np.uint8)

        # Draw points
        for i in range(len(x)):
            cv2.circle(img, (x[i], z[i]), 1, c[i].tolist(), -1)

    # Draw camera
    cam_pos = pose[:3, 3]
    cam_x = int(cam_pos[0] * scale + center)
    cam_z = int(cam_pos[2] * scale + center)

    # Camera direction
    cam_dir = pose[:3, 2]  # Z axis of camera
    dir_x = int((cam_pos[0] + cam_dir[0] * 0.5) * scale + center)
    dir_z = int((cam_pos[2] + cam_dir[2] * 0.5) * scale + center)

    cv2.circle(img, (cam_x, cam_z), 5, (0, 255, 0), -1)
    cv2.line(img, (cam_x, cam_z), (dir_x, dir_z), (0, 255, 255), 2)

    return img


def main():
    print("=" * 60)
    print("Monocular Gaussian SLAM Demo")
    print("=" * 60)

    # Load depth model
    print("\nLoading Depth Anything V2...")
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    depth_pipe = pipeline(
        'depth-estimation',
        model='depth-anything/Depth-Anything-V2-Small-hf',
        device=device
    )
    print(f"Depth model loaded on {device}")

    # Open webcam
    print("\nOpening webcam...")
    try:
        import _core
        cam = _core.create_webcam()
        if not cam.open():
            raise RuntimeError("Failed to open")
        use_core = True
        intr = cam.get_intrinsics()
        K = np.array([
            [intr.fx, 0, intr.cx],
            [0, intr.fy, intr.cy],
            [0, 0, 1]
        ], dtype=np.float32)
        width, height = intr.width, intr.height
        print(f"Using C++ webcam: {width}x{height}")
    except Exception as e:
        print(f"Using OpenCV webcam: {e}")
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        use_core = False
        width, height = 640, 480
        # Assume typical webcam intrinsics
        fx = fy = width * 1.2
        K = np.array([
            [fx, 0, width/2],
            [0, fy, height/2],
            [0, 0, 1]
        ], dtype=np.float32)

    # Initialize tracker and map
    tracker = SimpleTracker()
    gaussian_map = GaussianMap(max_points=50000)

    print("\nControls:")
    print("  Q - Quit")
    print("  R - Reset map")
    print("  S - Save point cloud")
    print("\nMove the camera slowly to build the map...")

    frame_count = 0
    fps_time = time.time()
    fps = 0

    try:
        while True:
            # Capture frame
            if use_core:
                frame = cam.get_frame()
                if not frame.valid():
                    continue
                rgb_data = frame.get_rgb_copy()
                shape = frame.rgb_shape()
                rgb = np.array(rgb_data, dtype=np.uint8).reshape(shape)
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            else:
                ret, bgr = cap.read()
                if not ret:
                    continue
                rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

            # Track camera
            pose, tracking_ok = tracker.track(gray, K)

            # Estimate depth
            pil_img = Image.fromarray(rgb)
            depth_result = depth_pipe(pil_img)
            depth = np.array(depth_result['depth']).astype(np.float32)

            # Normalize depth to reasonable range (DA2 gives relative depth)
            depth = depth / depth.max() * 5.0  # Scale to ~5m max

            # Resize depth to match RGB
            depth = cv2.resize(depth, (rgb.shape[1], rgb.shape[0]))

            # Add to map if tracking is good
            if tracking_ok and frame_count % 5 == 0:  # Every 5th frame
                gaussian_map.add_frame(rgb, depth, pose, K, subsample=16)

            # Visualize depth
            depth_vis = (depth / depth.max() * 255).astype(np.uint8)
            depth_colored = cv2.applyColorMap(depth_vis, cv2.COLORMAP_INFERNO)

            # Create top-down map view
            points, colors = gaussian_map.get_points()
            topdown = create_topdown_view(points, colors, pose)

            # Combine views
            h, w = bgr.shape[:2]
            topdown_resized = cv2.resize(topdown, (h, h))

            # Stack: RGB | Depth | TopDown
            depth_resized = cv2.resize(depth_colored, (w, h))
            combined = np.hstack([bgr, depth_resized, topdown_resized])

            # Add info overlay
            fps = 0.9 * fps + 0.1 * (1.0 / (time.time() - fps_time + 1e-6))
            fps_time = time.time()

            status = "TRACKING" if tracking_ok else "LOST"
            color = (0, 255, 0) if tracking_ok else (0, 0, 255)

            cv2.putText(combined, f"Frame {frame_count} | {fps:.1f} FPS | {status}",
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            cv2.putText(combined, f"Points: {gaussian_map.get_point_count()}",
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            pos = pose[:3, 3]
            cv2.putText(combined, f"Pos: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})",
                       (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

            # Labels
            cv2.putText(combined, "RGB", (10, h-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
            cv2.putText(combined, "Depth", (w+10, h-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
            cv2.putText(combined, "Top-Down Map", (2*w+10, h-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

            cv2.imshow('Monocular Gaussian SLAM', combined)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('r'):
                tracker = SimpleTracker()
                gaussian_map = GaussianMap()
                print("Map reset!")
            elif key == ord('s'):
                pts, cols = gaussian_map.get_points()
                np.savez('pointcloud.npz', points=pts, colors=cols)
                print(f"Saved {len(pts)} points to pointcloud.npz")

            frame_count += 1

    except KeyboardInterrupt:
        pass
    finally:
        if use_core:
            cam.close()
        else:
            cap.release()
        cv2.destroyAllWindows()

        pts, cols = gaussian_map.get_points()
        print(f"\nProcessed {frame_count} frames")
        print(f"Accumulated {len(pts)} 3D points")


if __name__ == "__main__":
    main()
