# Spatial AI Playground - Architecture Guide

## 1. High-Level Architecture: "The Python-Driven C++ Engine"

Do **not** try to write the GUI in C++ (Qt/ImGui) and embed Python. It is much easier to write the "Application" in Python and import your C++ SLAM system as a high-performance native extension.

### Architecture Layers

| Layer | Technology | Responsibility |
|-------|------------|----------------|
| Layer 1 (Hardware/Core) | C++ (Visual Studio 2022 / MSVC) | RealSense/Webcam capture, feature extraction, heavy SLAM loop |
| Layer 2 (Bindings) | Nanobind | Exposes C++ classes to Python (faster than pybind11) |
| Layer 3 (Logic/Playground) | Python | Initializes C++ engine, manages sessions, triggers ML tasks |
| Layer 4 (Visualization) | Rerun.io | 3D visualization without custom OpenGL code |

---

## 2. Recommended Tech Stack

| Component | Technology | Why? |
|-----------|------------|------|
| Build System | CMake + vcpkg | Only sane way to manage C++ deps on Windows |
| Bindings | Nanobind | Faster compile, smaller binaries than pybind11 |
| Sensors | librealsense2 / OpenCV | C++ APIs for low-latency capture |
| Math | Eigen | Standard for C++ linear algebra |
| Visualization | Rerun | Logs 3D data from C++ or Python to standalone viewer |
| SLAM Core | Custom / ORB-SLAM3 | Start simple, wrap complex libs later |

---

## 3. Project Structure

```
SpatialPlayground/
├── vcpkg.json                 # Dependency manifest
├── CMakeLists.txt             # Main build config
├── pyproject.toml             # Python build config (scikit-build-core)
├── src/
│   ├── bindings/
│   │   └── module.cpp         # Nanobind module entry point
│   ├── sensors/
│   │   ├── camera.h           # Abstract base class
│   │   └── webcam.h           # Webcam implementation
│   └── slam/
│       └── tracker.cpp        # C++ SLAM logic
├── python/
│   └── spatial_ai_playground/ # Python package
│       ├── __init__.py
│       └── app.py             # Main entry point with Rerun
├── external/                  # Git submodules (forked research code)
└── legacy/                    # Preserved original code
```

---

## 4. Implementation Pattern (The "Bridge")

### A. C++ Side (tracker.cpp)
Write SLAM system to accept raw pointers or simple structs:

```cpp
class SlimSlamSystem {
public:
    void process_frame(const cv::Mat& image, double timestamp) {
        // Heavy C++ SLAM processing
        current_pose_ = ...;
    }
    Eigen::Matrix4f get_current_pose() { return current_pose_; }
};
```

### B. Binding Side (module.cpp)
Use Nanobind to wrap. Auto-converts cv::Mat to numpy:

```cpp
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/eigen.h>

namespace nb = nanobind;

NB_MODULE(_core, m) {
    nb::class_<SlimSlamSystem>(m, "SlimSlamSystem")
        .def(nb::init<>())
        .def("process_frame", [](SlimSlamSystem& s,
             nb::ndarray<uint8_t, nb::shape<nb::any, nb::any, 3>> frame) {
            cv::Mat img(frame.shape(0), frame.shape(1), CV_8UC3, frame.data());
            s.process_frame(img, 0.0);
        })
        .def("get_pose", &SlimSlamSystem::get_current_pose);
}
```

### C. Python/Visualization Side
Log to Rerun instead of building custom viewer:

```python
import spatial_ai_playground._core as cpp_slam
import rerun as rr
import cv2

rr.init("Spatial Playground", spawn=True)

slam = cpp_slam.SlimSlamSystem()
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret: break

    slam.process_frame(frame)
    pose = slam.get_pose()

    rr.log("world/camera", rr.Transform3D(translation=pose[:3, 3]))
    rr.log("world/camera/image", rr.Image(frame))
```

---

## 5. Windows-Specific Advice

### Use vcpkg in Manifest Mode
Create `vcpkg.json` in root. Add eigen3, opencv, rerun-cpp-sdk, nanobind.
Configure CMake with:
```
-DCMAKE_TOOLCHAIN_FILE=C:/path/to/vcpkg/scripts/buildsystems/vcpkg.cmake
```

### Compiler
- Use MSVC (Visual Studio 2022) over MinGW
- Python on Windows is built with MSVC; mixing ABIs causes crashes

### Sensor Drivers
- RealSense SDK installs via vcpkg
- Do NOT use pyrealsense2 Python library if modifying capture pipeline
- Wrap C++ library yourself for full control

### Debug vs Release
- **Critical**: Cannot mix Debug C++ with Release Python on Windows
- Always build extension in `RelWithDebInfo` for speed with debuggability

---

## 6. Workflow Summary

1. Edit C++ logic in `src/`
2. Run `pip install .` (or CMake build)
3. Run Python script to visualize in Rerun
4. Iterate: attach VS Debugger if crashes occur
