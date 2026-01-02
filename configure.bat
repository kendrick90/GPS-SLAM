@echo off
REM Spatial AI Playground - Configure Script
REM Uses Visual Studio 2022 generator with vcpkg toolchain

call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvarsall.bat" x64

set CUDA_PATH=C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.6
set PATH=C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.6\bin;%PATH%

cd /d "%~dp0"
if exist build rmdir /s /q build
mkdir build
cd build

REM Set vcpkg toolchain file path
if defined VCPKG_ROOT (
    set VCPKG_TOOLCHAIN=%VCPKG_ROOT%/scripts/buildsystems/vcpkg.cmake
    echo Using vcpkg from VCPKG_ROOT: %VCPKG_ROOT%
) else (
    set VCPKG_TOOLCHAIN=C:/vcpkg/scripts/buildsystems/vcpkg.cmake
    echo Using default vcpkg path: C:/vcpkg
)

cmake .. -DCMAKE_TOOLCHAIN_FILE="%VCPKG_TOOLCHAIN%" ^
    -G "Visual Studio 17 2022" -A x64 ^
    -DCMAKE_CUDA_COMPILER="C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v12.6/bin/nvcc.exe" ^
    -DCUDAToolkit_ROOT="C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v12.6" ^
    -DCMAKE_PREFIX_PATH="C:/libtorch/libtorch" ^
    -DSAP_BUILD_PYTHON=ON ^
    -DSAP_BUILD_LEGACY=ON

pause
