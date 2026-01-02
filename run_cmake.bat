@echo off
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvarsall.bat" x64

set CUDA_PATH=C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.6
set PATH=C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.6\bin;%PATH%

cd /d "%~dp0"
if exist build rmdir /s /q build
mkdir build
cd build

cmake .. -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake ^
    -G "Visual Studio 17 2022" -A x64 ^
    -DCMAKE_CUDA_COMPILER="C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v12.6/bin/nvcc.exe" ^
    -DCUDAToolkit_ROOT="C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v12.6" ^
    -DCMAKE_PREFIX_PATH="C:/libtorch/libtorch" > cmake_log.txt 2>&1

type cmake_log.txt
echo.
echo CMAKE_RESULT: %ERRORLEVEL%
