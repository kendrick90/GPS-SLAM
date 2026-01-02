#!/usr/bin/env python3
"""
Spatial AI Playground - Webcam Demo

This script demonstrates basic webcam capture and visualization using Rerun.
It serves as a starting point for the Spatial AI Playground, allowing you to
verify your webcam setup and Rerun installation before integrating with the
full C++ SLAM pipeline.

Requirements:
    - Python 3.8+
    - opencv-python (cv2)
    - rerun-sdk

Install dependencies:
    pip install opencv-python rerun-sdk

Usage:
    python demo_webcam.py

Controls:
    - Press 'ESC' or 'q' to exit
"""

import sys
import time

try:
    import cv2
except ImportError:
    print("Error: OpenCV not found. Install with: pip install opencv-python")
    sys.exit(1)

try:
    import rerun as rr
except ImportError:
    print("Error: Rerun not found. Install with: pip install rerun-sdk")
    sys.exit(1)


def setup_rerun():
    """Initialize the Rerun recording session."""
    # Initialize Rerun with application name
    rr.init("spatial_ai_webcam_demo", spawn=True)

    # Log a description of what this recording contains
    rr.log(
        "readme",
        rr.TextDocument(
            """
# Spatial AI Playground - Webcam Demo

This demo captures frames from your webcam and logs them to Rerun.

## Visualization
- `/camera/image`: Live webcam feed
- `/camera/info`: Frame metadata

## Controls
Press 'ESC' or 'q' in the OpenCV window to exit.
            """.strip(),
            media_type=rr.MediaType.MARKDOWN,
        ),
    )


def open_webcam(camera_index=0):
    """
    Open a webcam for capture.

    Args:
        camera_index: The camera device index (default: 0 for primary webcam)

    Returns:
        cv2.VideoCapture object or None if failed
    """
    print(f"Opening webcam (index: {camera_index})...")

    # Try to open the webcam
    cap = cv2.VideoCapture(camera_index)

    if not cap.isOpened():
        print(f"Error: Could not open webcam at index {camera_index}")
        return None

    # Get and display camera properties
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)

    print(f"Webcam opened successfully:")
    print(f"  Resolution: {width}x{height}")
    print(f"  FPS: {fps}")

    return cap


def log_frame_to_rerun(frame, frame_count, timestamp):
    """
    Log a webcam frame to Rerun for visualization.

    Args:
        frame: BGR image from OpenCV (numpy array)
        frame_count: Current frame number
        timestamp: Timestamp in seconds
    """
    # Set the current time for this frame
    rr.set_time_seconds("time", timestamp)
    rr.set_time_sequence("frame", frame_count)

    # Convert BGR (OpenCV format) to RGB (Rerun format)
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Log the image to Rerun
    rr.log("camera/image", rr.Image(frame_rgb))

    # Log frame metadata as text
    height, width = frame.shape[:2]
    rr.log(
        "camera/info",
        rr.TextLog(
            f"Frame {frame_count}: {width}x{height} @ t={timestamp:.3f}s",
            level=rr.TextLogLevel.INFO,
        ),
    )


def main():
    """Main function - runs the webcam demo loop."""
    print("=" * 50)
    print("Spatial AI Playground - Webcam Demo")
    print("=" * 50)
    print()

    # Initialize Rerun visualization
    print("Initializing Rerun...")
    setup_rerun()

    # Open the webcam
    cap = open_webcam(camera_index=0)
    if cap is None:
        return 1

    print()
    print("Starting capture loop...")
    print("Press 'ESC' or 'q' to exit")
    print("-" * 50)

    frame_count = 0
    start_time = time.time()

    try:
        # Main capture loop
        while True:
            # Capture a frame from the webcam
            ret, frame = cap.read()

            if not ret:
                print("Error: Failed to capture frame")
                break

            # Calculate timestamp relative to start
            timestamp = time.time() - start_time

            # Log the frame to Rerun
            log_frame_to_rerun(frame, frame_count, timestamp)

            # Display the frame in an OpenCV window (optional, for quick preview)
            cv2.imshow("Webcam Demo (Press ESC or 'q' to exit)", frame)

            # Print progress every 100 frames
            if frame_count % 100 == 0:
                fps = frame_count / max(timestamp, 0.001)
                print(f"Frame {frame_count}: {fps:.1f} FPS")

            frame_count += 1

            # Check for keyboard input (ESC key = 27, 'q' = 113)
            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord('q'):
                print()
                print("Exit requested by user")
                break

    except KeyboardInterrupt:
        print()
        print("Interrupted by Ctrl+C")

    finally:
        # Cleanup
        print()
        print("-" * 50)
        print("Cleaning up...")

        # Release the webcam
        cap.release()

        # Close OpenCV windows
        cv2.destroyAllWindows()

        # Print summary
        elapsed = time.time() - start_time
        avg_fps = frame_count / max(elapsed, 0.001)
        print(f"Captured {frame_count} frames in {elapsed:.2f} seconds")
        print(f"Average FPS: {avg_fps:.1f}")
        print()
        print("Rerun viewer should remain open for playback.")
        print("Close the Rerun viewer window when done.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
