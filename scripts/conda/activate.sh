#!/bin/bash
# CUDA 11.8 environment setup for GPS-SLAM
# This script is sourced when the gps-slam conda environment is activated

# Detect CUDA installation path
if [ -d "/usr/local/cuda-11.8" ]; then
    export CUDA_HOME=/usr/local/cuda-11.8
elif [ -d "/usr/local/cuda" ]; then
    export CUDA_HOME=/usr/local/cuda
fi

if [ -n "$CUDA_HOME" ]; then
    export PATH=$CUDA_HOME/bin:$PATH
    export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH
    export CUDACXX=$CUDA_HOME/bin/nvcc
fi

# Use GCC 11 for CUDA compatibility (CUDA 11.8 requires GCC <= 11)
if [ -x "/usr/bin/gcc-11" ]; then
    export CC=/usr/bin/gcc-11
    export CXX=/usr/bin/g++-11
    export CUDAHOSTCXX=/usr/bin/g++-11
fi
