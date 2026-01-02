@echo off
REM Spatial AI Playground - Build Script
REM Uses MSBuild via cmake --build for Visual Studio 2022 projects

call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvarsall.bat" x64

set CUDA_PATH=C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.6
set PATH=C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.6\bin;%PATH%

cd /d "%~dp0"

REM Check if build directory exists
if not exist build (
    echo ERROR: Build directory not found. Please run configure.bat first.
    pause
    exit /b 1
)

REM Build using cmake --build which invokes MSBuild for Visual Studio generators
echo Building Spatial AI Playground in Release configuration...
cmake --build build --config Release --parallel 8

if %ERRORLEVEL% neq 0 (
    echo Build failed with error code %ERRORLEVEL%
    pause
    exit /b %ERRORLEVEL%
)

echo Build completed successfully.
pause
