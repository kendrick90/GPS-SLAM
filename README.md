# Spatial AI Playground

A modular platform for experimenting with spatial AI, SLAM, depth estimation, segmentation, and neural rendering techniques.

## Vision

Spatial AI Playground is designed to be a flexible experimentation platform where different computer vision and 3D reconstruction techniques can be easily combined and compared. Instead of a monolithic SLAM system, it provides a plugin architecture where:

- **Input sources** (webcam, RGB-D cameras, VR headsets) are interchangeable
- **Depth estimation** models (Depth Anything, Metric3D, MonST3R) can be swapped
- **Tracking methods** (ICP, feature-based, learning-based) are modular
- **Scene representations** (TSDF, surfels, Gaussians, meshes) are pluggable
- **Renderers** (Gaussian splatting, triangle splatting, mesh) work with different representations
- **Viewers** (OpenCV, 3D visualizer, VR) can display results

## Supported Techniques

### Input Sources
- Webcam / NDI virtual camera (for phone streaming)
- Azure Kinect RGB-D
- Intel RealSense
- Video files
- Quest 3 (planned)

### Depth Estimation
- Depth Anything V2
- Metric3D V2
- MonST3R / DUSt3R
- ZoeDepth

### Segmentation
- Segment Anything (SAM, SAM2)
- Grounded SAM (text-prompted)

### Features
- DINOv2
- SuperPoint / LightGlue
- CLIP

### SLAM Components
- **Trackers**: ICP, feature-based, photometric, IMU-fused
- **Mappers**: TSDF (InfiniTAM), surfels, point cloud, Gaussian, neural SDF
- **Optimizers**: Bundle adjustment, pose graph optimization
- **Loop Closure**: Bag of words, learned descriptors

### Renderers
- 3D Gaussian Splatting
- Triangle Splatting
- Mesh rasterization
- TSDF raycast

## Architecture

```
spatial-ai-playground/
├── core/                    # Core framework
│   ├── include/sap/
│   │   ├── core/           # Frame, Plugin, Config
│   │   └── interfaces/     # IInputSource, ITracker, IMapper, IRenderer, etc.
│   └── src/
├── plugins/                 # All plugins by type
│   ├── input/              # webcam, azure_kinect, realsense
│   ├── depth/              # depth_anything, metric3d
│   ├── segmentation/       # sam
│   ├── features/           # dinov2, superpoint
│   ├── slam/               # infinitam_tsdf, wildgs_slam
│   ├── rendering/          # gaussian_splatting, triangle_splatting
│   └── viewers/            # opencv_viewer, pangolin_viewer
├── legacy/                  # Original GPS-SLAM code (preserved)
├── apps/                    # Applications
├── python/                  # Python package
└── configs/                 # YAML configurations
```

## Quick Start

### RGB-D SLAM (Azure Kinect)
```yaml
# configs/rgbd_slam.yaml
input:
  plugin: azure_kinect
  depth_mode: 2  # WFOV
slam:
  tracker: icp_depth
  mapper: tsdf
rendering:
  plugin: gaussian_splatting
viewer:
  plugin: opencv_viewer
```

### RGB-Only SLAM (Webcam + Depth Estimation)
```yaml
# configs/rgb_slam.yaml
input:
  plugin: webcam
  device_id: 0
depth:
  plugin: depth_anything_v2
slam:
  tracker: feature_based
  mapper: gaussian
rendering:
  plugin: gaussian_splatting
viewer:
  plugin: opencv_viewer
```

## Building

### Prerequisites
- CMake 3.21+
- CUDA 12.0+
- Python 3.10+
- PyTorch 2.0+

### Build
```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --config Release -j8
```

### Python Package
```bash
pip install -e .
```

## Creating Plugins

### C++ Plugin
```cpp
#include "sap/interfaces/input_source.h"

class MyCamera : public sap::IInputSource {
    // Implement interface methods
};

SAP_REGISTER_PLUGIN(input, my_camera, MyCamera);
```

### Python Plugin
```python
from spatial_ai_playground.plugins.base import DepthEstimatorBase, register_plugin

@register_plugin("depth", "my_depth")
class MyDepthEstimator(DepthEstimatorBase):
    def estimate(self, frame):
        # Return depth prediction
        pass
```

## Legacy Code

The original GPS-SLAM implementation (InfiniTAM + Gaussian Splatting hybrid) is preserved in the `legacy/` directory and wrapped as plugins for backward compatibility. The original paper:

> **Gaussian-plus-SDF SLAM: High-fidelity 3D Reconstruction at 150+ fps**
> Zhexi Peng, Kun Zhou, Tianjia Shao
> [Paper](https://arxiv.org/abs/2509.11574)

## Contributing

Contributions welcome! The plugin architecture makes it easy to add new:
- Input sources for different cameras/sensors
- Depth estimation models
- SLAM tracking/mapping methods
- Rendering techniques
- Visualization tools

## Acknowledgments

This project builds on and integrates work from:
- [GPS-SLAM](https://github.com/MisEty/GPS-SLAM) - Original Gaussian+SDF SLAM (fork origin)
- [InfiniTAM](https://github.com/victorprad/InfiniTAM) - TSDF fusion
- [gsplat](https://github.com/nerfstudio-project/gsplat) - Gaussian splatting
- [Depth Anything](https://github.com/LiheYoung/Depth-Anything) - Monocular depth
- [WildGS-SLAM](https://github.com/GradientSpaces/WildGS-SLAM) - Monocular Gaussian SLAM
- [Segment Anything](https://github.com/facebookresearch/segment-anything) - Segmentation

## License

This project is freely shared. Use it however you like.
