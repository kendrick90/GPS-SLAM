"""
Command-line interface for Spatial AI Playground.
"""

import argparse
import sys


def main():
    """Main CLI entry point."""
    parser = argparse.ArgumentParser(
        description="Spatial AI Playground - SLAM and Neural Rendering Experiments"
    )
    subparsers = parser.add_subparsers(dest="command", help="Available commands")

    # Run command
    run_parser = subparsers.add_parser("run", help="Run the playground")
    run_parser.add_argument(
        "--camera", type=str, default="webcam",
        help="Camera type (webcam, realsense, azure_kinect)"
    )
    run_parser.add_argument(
        "--device", type=int, default=0,
        help="Camera device ID"
    )
    run_parser.add_argument(
        "--tracker", type=str, default="simple_vo",
        help="Tracker type (simple_vo, orb, icp)"
    )
    run_parser.add_argument(
        "--depth", action="store_true",
        help="Enable monocular depth estimation"
    )
    run_parser.add_argument(
        "--depth-model", type=str, default="depth_anything_v2",
        help="Depth estimation model"
    )
    run_parser.add_argument(
        "--config", type=str,
        help="Path to YAML config file"
    )

    # Info command
    info_parser = subparsers.add_parser("info", help="Show system information")

    # Version command
    version_parser = subparsers.add_parser("version", help="Show version")

    args = parser.parse_args()

    if args.command == "run":
        from .app import SpatialPlayground

        app = SpatialPlayground()

        if not app.setup_camera(args.camera, device_id=args.device):
            print("Failed to open camera")
            sys.exit(1)

        if args.tracker:
            app.setup_tracker(args.tracker)

        if args.depth:
            app.setup_depth_estimator(args.depth_model)

        app.run()

    elif args.command == "info":
        print("Spatial AI Playground")
        print("=" * 40)

        # Check C++ core
        try:
            from . import _core
            print("C++ Core: Available")
        except ImportError:
            print("C++ Core: Not built")

        # Check Rerun
        try:
            import rerun
            print(f"Rerun: {rerun.__version__}")
        except ImportError:
            print("Rerun: Not installed")

        # Check PyTorch
        try:
            import torch
            print(f"PyTorch: {torch.__version__}")
            print(f"CUDA available: {torch.cuda.is_available()}")
            if torch.cuda.is_available():
                print(f"CUDA device: {torch.cuda.get_device_name(0)}")
        except ImportError:
            print("PyTorch: Not installed")

        # Check OpenCV
        try:
            import cv2
            print(f"OpenCV: {cv2.__version__}")
        except ImportError:
            print("OpenCV: Not installed")

    elif args.command == "version":
        from . import __version__
        print(f"spatial-ai-playground {__version__}")

    else:
        parser.print_help()


if __name__ == "__main__":
    main()
