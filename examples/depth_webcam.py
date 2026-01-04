"""
Real-time depth estimation from webcam using Depth Anything V2
"""
import sys
sys.path.insert(0, 'build/Release')

import cv2
import numpy as np
import torch
from transformers import pipeline
from PIL import Image

def main():
    print("Loading Depth Anything V2...")
    depth_pipe = pipeline(
        'depth-estimation',
        model='depth-anything/Depth-Anything-V2-Small-hf',
        device='cuda' if torch.cuda.is_available() else 'cpu'
    )
    print(f"Model loaded on {depth_pipe.device}")

    # Try to use our C++ webcam, fall back to OpenCV
    try:
        import _core
        cam = _core.create_webcam()
        if not cam.open():
            raise RuntimeError("Failed to open webcam via _core")
        use_core = True
        intrinsics = cam.get_intrinsics()
        print(f"Using C++ webcam: {intrinsics.width}x{intrinsics.height}")
    except Exception as e:
        print(f"Falling back to OpenCV webcam: {e}")
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        use_core = False

    print("\nPress Q to quit, S to save frame")
    frame_count = 0

    try:
        while True:
            # Capture frame
            if use_core:
                frame = cam.get_frame()
                if not frame.valid():
                    continue
                rgb_data = frame.get_rgb_copy()
                shape = frame.rgb_shape()
                img = np.array(rgb_data, dtype=np.uint8).reshape(shape)
                img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            else:
                ret, img_bgr = cap.read()
                if not ret:
                    continue
                img = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

            # Run depth estimation
            pil_img = Image.fromarray(img)
            result = depth_pipe(pil_img)
            depth = np.array(result['depth'])

            # Normalize depth for visualization
            depth_normalized = (depth - depth.min()) / (depth.max() - depth.min() + 1e-6)
            depth_colored = cv2.applyColorMap(
                (depth_normalized * 255).astype(np.uint8),
                cv2.COLORMAP_INFERNO
            )

            # Resize depth to match input
            depth_colored = cv2.resize(depth_colored, (img_bgr.shape[1], img_bgr.shape[0]))

            # Side by side display
            combined = np.hstack([img_bgr, depth_colored])

            # Add FPS counter
            cv2.putText(combined, f'Frame {frame_count}', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(combined, 'RGB', (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(combined, 'Depth (Depth Anything V2)', (img_bgr.shape[1] + 10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            cv2.imshow('Depth Anything V2 - Webcam', combined)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                cv2.imwrite(f'depth_frame_{frame_count}.png', combined)
                print(f"Saved depth_frame_{frame_count}.png")

            frame_count += 1

    except KeyboardInterrupt:
        pass
    finally:
        if use_core:
            cam.close()
        else:
            cap.release()
        cv2.destroyAllWindows()
        print(f"\nProcessed {frame_count} frames")

if __name__ == "__main__":
    main()
