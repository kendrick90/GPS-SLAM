#!/bin/bash
# GPS-SLAM Environment Setup Script
# This script sets up the conda environment and installs activation hooks

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

echo "=== GPS-SLAM Environment Setup ==="
echo "Project directory: $PROJECT_DIR"

# Check for conda
if ! command -v conda &> /dev/null; then
    echo "ERROR: conda not found. Please install Miniconda or Anaconda first."
    echo "Download from: https://docs.conda.io/en/latest/miniconda.html"
    exit 1
fi

# Check for system dependencies
echo ""
echo "Checking system dependencies..."

missing_deps=()

if [ ! -d "/usr/local/cuda-11.8" ] && [ ! -d "/usr/local/cuda" ]; then
    missing_deps+=("CUDA Toolkit 11.8")
fi

if ! command -v gcc-11 &> /dev/null; then
    missing_deps+=("gcc-11")
fi

if ! command -v g++-11 &> /dev/null; then
    missing_deps+=("g++-11")
fi

if [ ! -f "/usr/include/GL/gl.h" ]; then
    missing_deps+=("OpenGL development libraries")
fi

if [ ${#missing_deps[@]} -ne 0 ]; then
    echo ""
    echo "WARNING: Missing system dependencies:"
    for dep in "${missing_deps[@]}"; do
        echo "  - $dep"
    done
    echo ""
    echo "Install with:"
    echo "  sudo apt install cuda-toolkit-11-8 gcc-11 g++-11 libgl1-mesa-dev libglx-dev libopengl-dev freeglut3-dev libopencv-dev libprotobuf-dev protobuf-compiler libboost-all-dev"
    echo ""
    read -p "Continue anyway? [y/N] " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Create conda environment
echo ""
echo "Creating conda environment 'gps-slam'..."
conda env create -f "$PROJECT_DIR/environment.yml" --force

# Get conda env path
CONDA_ENV_PATH=$(conda info --envs | grep gps-slam | awk '{print $NF}')

# Install activation hooks
echo ""
echo "Installing activation hooks..."
mkdir -p "$CONDA_ENV_PATH/etc/conda/activate.d"
mkdir -p "$CONDA_ENV_PATH/etc/conda/deactivate.d"
cp "$SCRIPT_DIR/conda/activate.sh" "$CONDA_ENV_PATH/etc/conda/activate.d/cuda.sh"
cp "$SCRIPT_DIR/conda/deactivate.sh" "$CONDA_ENV_PATH/etc/conda/deactivate.d/cuda.sh"
chmod +x "$CONDA_ENV_PATH/etc/conda/activate.d/cuda.sh"
chmod +x "$CONDA_ENV_PATH/etc/conda/deactivate.d/cuda.sh"

echo ""
echo "=== Setup Complete ==="
echo ""
echo "To use the environment:"
echo "  conda activate gps-slam"
echo ""
echo "To build GPS-SLAM:"
echo "  mkdir -p build && cd build"
echo "  cmake .."
echo "  make -j\$(nproc)"
echo ""
