@echo off
REM Spatial AI Playground - Run GPS-SLAM Script
REM Sets up environment and runs online_slam.exe

REM Set up Visual Studio environment
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvarsall.bat" x64 >nul 2>&1

REM CUDA paths
set CUDA_PATH=C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.6
set PATH=%CUDA_PATH%\bin;%PATH%

REM LibTorch paths
set PATH=C:\libtorch\libtorch\lib;%PATH%

REM Azure Kinect SDK
set PATH=C:\Program Files\Azure Kinect SDK v1.4.1\sdk\windows-desktop\amd64\release\bin;%PATH%

cd /d "%~dp0"

echo Starting GPS-SLAM with Azure Kinect...
echo Running: build\Release\online_slam.exe --config configs\azure_kinect.yaml
if not exist build\Release\online_slam.exe (
    echo ERROR: online_slam.exe not found!
    exit /b 1
)
build\Release\online_slam.exe --config configs\azure_kinect.yaml %*
echo Exit code: %ERRORLEVEL%
