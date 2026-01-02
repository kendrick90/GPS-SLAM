"""
Spatial AI Playground

A modular platform for experimenting with spatial AI, SLAM, depth estimation,
segmentation, and neural rendering techniques.
"""

__version__ = "0.1.0"

# Try to import C++ core module
try:
    from spatial_ai_playground._core import (
        CameraIntrinsics,
        IMUSample,
        FrameData,
        Camera,
        WebcamCamera,
        TrackingStatus,
        TrackingResult,
        MapPoint,
        Tracker,
        SimpleVO,
        create_webcam,
        create_tracker,
    )
    HAS_CORE = True
except ImportError:
    HAS_CORE = False

from .app import SpatialPlayground

__all__ = [
    "__version__",
    "HAS_CORE",
    "SpatialPlayground",
    # C++ types (if available)
    "CameraIntrinsics",
    "IMUSample",
    "FrameData",
    "Camera",
    "WebcamCamera",
    "TrackingStatus",
    "TrackingResult",
    "MapPoint",
    "Tracker",
    "SimpleVO",
    "create_webcam",
    "create_tracker",
]
