# Online SLAM with Azure Kinect - Status Notes

**Date:** 2026-01-01
**Status:** WSL2 USB streaming issue with depth sensor

## What's Working

- Azure Kinect SDK installed (libk4a 1.4.1)
- USB passthrough via usbipd-win
- Device detection and initialization
- Color camera streaming (works in WSL2)
- AzureKinectEngine implementation complete
- online_slam executable built and runs
- TSDF engine initializes correctly
- Calibration data retrieved from sensor

## Current Issue

**Depth sensor USB bulk transfers fail in WSL2**

```
libusb: error [submit_bulk_transfer] submiturb failed error -1 errno=12
```

- Color frames arrive at 30fps
- Depth frames return null
- This is a known WSL2/usbipd limitation with Azure Kinect

## Files Created/Modified

| File | Description |
|------|-------------|
| `scripts/install_azure_kinect_ubuntu2204+.sh` | SDK installer |
| `InfiniTAM/InputSource/AzureKinectEngine.h` | Sensor interface |
| `InfiniTAM/InputSource/AzureKinectEngine.cpp` | Implementation |
| `InfiniTAM/cmake/UseAzureKinect.cmake` | CMake module |
| `slam/TsdfFusion/CLIEngine.h/cpp` | Added live streaming |
| `online_slam.cpp` | Online SLAM entry point |
| `CMakeLists.txt` | Added k4a linking |

## Build Commands

```bash
cd ~/GPS-SLAM/build_k4a

# Set environment
export PATH=/usr/bin:/bin:/usr/local/cuda-12.4/bin:$PATH
source ~/miniconda3/etc/profile.d/conda.sh
conda activate gps-slam-new
export CPATH=$CONDA_PREFIX/include:/usr/local/cuda-12.4/include:$CPATH

# Build
cmake ..
make -j4
```

## After Reboot - Steps to Retry

### 1. In Windows PowerShell (Admin)

```powershell
# List devices
usbipd list

# Bind Azure Kinect (if not already bound)
usbipd bind --force --busid 8-1
usbipd bind --force --busid 8-2

# Attach to WSL
usbipd attach --wsl --busid 8-1
usbipd attach --wsl --busid 8-2
```

### 2. In WSL

```bash
# Check devices visible
lsusb | grep -i kinect

# Increase USB buffer memory
sudo sh -c 'echo 1000 > /sys/module/usbcore/parameters/usbfs_memory_mb'

# Verify permissions (should be 0666)
ls -la /dev/bus/usb/002/

# Test online SLAM
cd ~/GPS-SLAM/build_k4a
source ~/miniconda3/etc/profile.d/conda.sh
conda activate gps-slam-new
./online_slam --max-frames 30
```

### 3. udev Rules (already set up)

Location: `/etc/udev/rules.d/99-k4a.rules`

```
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="097c", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="097d", MODE="0666"
```

## Alternative: MKV Playback Mode

If live streaming continues to fail, use Windows to record MKV files:

**On Windows:**
```
k4arecorder.exe -l 30 -c 720p -d NFOV_UNBINNED recording.mkv
```

**In WSL:** Add MKV playback support using `k4a_playback_*` API functions.

## Test on Windows First

Before continuing WSL debugging, verify the Azure Kinect works on Windows:

1. Open Azure Kinect Viewer (`k4aviewer.exe`)
2. Confirm RGB and Depth streams work
3. Try recording an MKV file
4. Check Device Manager for any USB issues

## online_slam Usage

```bash
./online_slam [config.yaml] [options]

Options:
  --depth-mode <0-3>   0=NFOV_UNBINNED, 1=NFOV_2X2BINNED, 2=WFOV_UNBINNED, 3=WFOV_2X2BINNED
  --color-res <0-5>    0=720P, 1=1080P, 2=1440P, 3=1536P, 4=2160P, 5=3072P
  --fps <5|15|30>      Frame rate
  --no-align           Don't align depth to color
  --max-frames <N>     Stop after N frames (0=unlimited)
  --output <dir>       Output directory
```

## Native Windows Build (Recommended for Production)

Since WSL2 USB passthrough is unreliable for Azure Kinect, building natively on Windows is recommended.

### Prerequisites

1. **Visual Studio 2022** with C++ desktop development workload
2. **CUDA Toolkit 12.x** - https://developer.nvidia.com/cuda-downloads
3. **Azure Kinect SDK 1.4.1** - https://github.com/microsoft/Azure-Kinect-Sensor-SDK/releases
4. **vcpkg** (recommended) or manual dependency installation

### Install Dependencies with vcpkg

```powershell
# Install vcpkg
git clone https://github.com/Microsoft/vcpkg.git C:\vcpkg
C:\vcpkg\bootstrap-vcpkg.bat

# Install dependencies
C:\vcpkg\vcpkg install eigen3:x64-windows
C:\vcpkg\vcpkg install opencv4:x64-windows
C:\vcpkg\vcpkg install yaml-cpp:x64-windows
C:\vcpkg\vcpkg install protobuf:x64-windows
C:\vcpkg\vcpkg install freeglut:x64-windows
C:\vcpkg\vcpkg install pangolin:x64-windows
```

### Get libtorch (PyTorch C++)

Download from https://pytorch.org/get-started/locally/ - select:
- PyTorch Build: Stable
- OS: Windows
- Package: LibTorch
- Language: C++/Java
- Compute Platform: CUDA 12.4

Extract to `C:\libtorch`

### Build Commands

```powershell
# Open "x64 Native Tools Command Prompt for VS 2022"
cd GPS-SLAM
mkdir build
cd build

# Configure with CMake
cmake .. -G "Visual Studio 17 2022" -A x64 ^
    -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake ^
    -DCMAKE_PREFIX_PATH=C:/libtorch ^
    -DWITH_AZUREKINECT=ON

# Build
cmake --build . --config Release

# Run
Release\online_slam.exe --max-frames 100
```

### Manual Dependency Locations

If not using vcpkg, set these environment variables:
- `K4A_ROOT` = `C:\Program Files\Azure Kinect SDK v1.4.1`
- `OPENCV_DIR` = path to OpenCV cmake files
- `Torch_DIR` = path to libtorch cmake files

### Troubleshooting Windows Build

| Issue | Solution |
|-------|----------|
| k4a.lib not found | Set `K4A_ROOT` environment variable |
| libtorch not found | Add to `CMAKE_PREFIX_PATH` |
| DLL not found at runtime | DLLs auto-copied by CMake, check build output |
| CUDA not detected | Ensure CUDA bin is in PATH |

## Next Steps

1. ~~Verify Azure Kinect works on Windows~~ (works)
2. ~~If Windows works, try again in WSL after fresh reboot~~ (WSL depth still fails)
3. ~~If WSL depth still fails, implement MKV playback mode~~ (or just use Windows)
4. **Build natively on Windows** ← Current approach
