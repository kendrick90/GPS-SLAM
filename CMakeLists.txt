cmake_minimum_required(VERSION 3.21)
project(spatial-ai-playground VERSION 0.1.0 LANGUAGES CXX)

# ============================================================================
# Spatial AI Playground Build Configuration
# ============================================================================
# Architecture: Python-driven with C++ core
# - Layer 1: C++ SLAM/Vision core (this CMake)
# - Layer 2: Nanobind bindings for Python
# - Layer 3: Python application (see python/)
# - Layer 4: Rerun visualization
# ============================================================================

option(SAP_BUILD_PYTHON "Build Python bindings via Nanobind" ON)
option(SAP_BUILD_LEGACY "Build legacy GPS-SLAM executables" ON)
option(SAP_BUILD_TESTS "Build tests" OFF)

# Prevent Windows.h from defining min/max macros
if(WIN32)
    add_definitions(-DNOMINMAX)
endif()

set(GPU_RUNTIME "CUDA" )
set(OPENSPLAT_MAX_CUDA_COMPATIBILITY OFF CACHE BOOL "Build for maximum CUDA device compatibility")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# Support both ThirdLibs (legacy) and conda-installed packages
if(EXISTS ${PROJECT_SOURCE_DIR}/ThirdLibs/libtorch)
    list(APPEND CMAKE_PREFIX_PATH ${PROJECT_SOURCE_DIR}/ThirdLibs/libtorch)
endif()
if(EXISTS ${PROJECT_SOURCE_DIR}/ThirdLibs/install)
    list(APPEND CMAKE_PREFIX_PATH ${PROJECT_SOURCE_DIR}/ThirdLibs/install)
endif()
# Conda environment paths (set by conda activate)
if(DEFINED ENV{CONDA_PREFIX})
    list(APPEND CMAKE_PREFIX_PATH $ENV{CONDA_PREFIX})
endif()

# Read version
file(READ "VERSION" APP_VERSION)

# Read git commit
set(GIT_REV "")
execute_process(COMMAND git rev-parse --short HEAD
                WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}" 
                OUTPUT_VARIABLE GIT_REV 
                ERROR_QUIET)
string(REGEX REPLACE "\n$" "" GIT_REV "${GIT_REV}")
if (NOT "${GIT_REV}" STREQUAL "")
    set(DAPP_VERSION "${APP_VERSION} (git commit ${GIT_REV})")
    set(DAPP_REVISION "${GIT_REV}")
else()
    set(DAPP_VERSION "${APP_VERSION}")
    set(DAPP_REVISION "dev")
endif()

message("CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")

add_compile_options("-DAPP_VERSION=\"${DAPP_VERSION}\"")
add_compile_options("-DAPP_REVISION=\"${DAPP_REVISION}\"")

# Don't complain about the override from NANOFLANN_BUILD_EXAMPLES
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
# Use time-of-extraction for FetchContent'ed files modification time
if (CMAKE_VERSION VERSION_GREATER_EQUAL "3.24.0")
    cmake_policy(SET CMP0135 NEW)
endif()
# Suppress warning #20012-D (nvcc and glm)
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -diag-suppress=20012)

include(FetchContent)
FetchContent_Declare(nlohmann_json
    URL https://github.com/nlohmann/json/archive/refs/tags/v3.11.3.zip
)
set(NANOFLANN_BUILD_EXAMPLES OFF)
set(NANOFLANN_BUILD_TESTS OFF)
FetchContent_Declare(nanoflann
    URL https://github.com/jlblancoc/nanoflann/archive/refs/tags/v1.5.5.zip
)
FetchContent_Declare(cxxopts
    URL https://github.com/jarro2783/cxxopts/archive/refs/tags/v3.2.0.zip
)
FetchContent_MakeAvailable(nlohmann_json nanoflann cxxopts)
FetchContent_Declare(glm
    URL https://github.com/g-truc/glm/archive/refs/tags/1.0.1.zip
)
FetchContent_MakeAvailable(glm)

# indicators - progress bars (try find_package first, then FetchContent)
find_package(indicators QUIET)
if(NOT indicators_FOUND)
    message(STATUS "indicators not found, fetching from GitHub...")
    FetchContent_Declare(indicators
        GIT_REPOSITORY https://github.com/p-ranav/indicators.git
        GIT_TAG v2.3
    )
    set(INDICATORS_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(indicators)
endif()

# tinyply - PLY file parser (try find_package first, then FetchContent)
find_package(tinyply QUIET)
if(NOT tinyply_FOUND)
    message(STATUS "tinyply not found, fetching from GitHub...")
    FetchContent_Declare(tinyply
        GIT_REPOSITORY https://github.com/ddiakopoulos/tinyply.git
        GIT_TAG 2.3.4
    )
    set(SHARED_LIB OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(tinyply)
endif()

# tensorboard_logger (optional - has protobuf version conflicts on Windows)
option(USE_TENSORBOARD "Enable TensorBoard logging" OFF)
if(USE_TENSORBOARD)
    find_package(tensorboard_logger QUIET)
    if(NOT tensorboard_logger_FOUND)
        message(STATUS "tensorboard_logger not found, fetching from GitHub...")
        FetchContent_Declare(tensorboard_logger
            GIT_REPOSITORY https://github.com/RustingSword/tensorboard_logger.git
            GIT_TAG master
        )
        FetchContent_MakeAvailable(tensorboard_logger)
    endif()
else()
    message(STATUS "TensorBoard logging disabled (USE_TENSORBOARD=OFF)")
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif()

find_package(CUDAToolkit)
if (NOT CUDAToolkit_FOUND)
    message(WARNING "CUDA toolkit not found, building with CPU support only")
    set(GPU_RUNTIME "CPU")
else()
    if (OPENSPLAT_MAX_CUDA_COMPATIBILITY)
        execute_process(COMMAND "${CUDAToolkit_NVCC_EXECUTABLE}" --list-gpu-arch 
                OUTPUT_VARIABLE LIST_GPU_ARCH 
                ERROR_QUIET)
    endif()

    if(NOT LIST_GPU_ARCH AND OPENSPLAT_MAX_CUDA_COMPATIBILITY)
        message(WARNING "Cannot compile for max CUDA compatibility, nvcc does not support --list-gpu-arch")
        SET(OPENSPLAT_MAX_CUDA_COMPATIBILITY OFF)
    endif()
    if(NOT OPENSPLAT_MAX_CUDA_COMPATIBILITY)
        if(NOT CMAKE_CUDA_ARCHITECTURES)
            # CUDA 13+ dropped compute_70 (Volta), use 75+ (Turing and newer)
            SET(CMAKE_CUDA_ARCHITECTURES 75;80;86;89;90)
        endif()
    else()
        # Build for maximum compatibility
        # https://arnon.dk/matching-sm-architectures-arch-and-gencode-for-various-nvidia-cards/
        set(CMAKE_CUDA_ARCHITECTURES "")

        # Extract list of arch and gencodes
        string(REPLACE "\r" "" LIST_GPU_ARCH ${LIST_GPU_ARCH})
        string(REPLACE "\n" ";" LIST_GPU_ARCH ${LIST_GPU_ARCH})

        execute_process(COMMAND "${CUDAToolkit_NVCC_EXECUTABLE}" --list-gpu-code 
            OUTPUT_VARIABLE LIST_GPU_CODE 
            ERROR_QUIET)
        string(REPLACE "\r" "" LIST_GPU_CODE ${LIST_GPU_CODE})
        string(REPLACE "\n" ";" LIST_GPU_CODE ${LIST_GPU_CODE})

        list(GET LIST_GPU_CODE 0 TARGET_GPU_CODE)
        set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -arch=${TARGET_GPU_CODE}")
        
        set(IDX 0)
        foreach(GPU_ARCH ${LIST_GPU_ARCH})
            string(REGEX MATCH "compute_([0-9]+)" GPU_ARCH_VERSION "${GPU_ARCH}")
            list(APPEND CMAKE_CUDA_ARCHITECTURES "${CMAKE_MATCH_1}")
            list(GET LIST_GPU_CODE ${IDX} GPU_CODE)
            set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -gencode=arch=${GPU_ARCH},code=${GPU_CODE}")
            math(EXPR IDX "${IDX}+1")
        endforeach()
        message("Set CUDA flags: " ${CMAKE_CUDA_FLAGS})
    endif()
    # Set torch cuda architecture list
    set(TORCH_CUDA_ARCH_LIST ${CMAKE_CUDA_ARCHITECTURES})
    list(TRANSFORM TORCH_CUDA_ARCH_LIST REPLACE "([0-9])([0-9])" "\\1.\\2")
    string(REPLACE ";" " " TORCH_CUDA_ARCH_LIST "${TORCH_CUDA_ARCH_LIST}")
    message(STATUS "** Updated TORCH_CUDA_ARCH_LIST to ${TORCH_CUDA_ARCH_LIST} **")
endif()


set(CMAKE_CXX_STANDARD 17)
enable_language(${GPU_RUNTIME})
set(CMAKE_${GPU_RUNTIME}_STANDARD 17)
set(${GPU_RUNTIME}_STANDARD 17)

if (NOT WIN32 AND NOT APPLE)
    set(STDPPFS_LIBRARY stdc++fs)
endif()

##################### InfiniTAM ############################
INCLUDE(${PROJECT_SOURCE_DIR}/legacy/InfiniTAM/cmake/UseGLUT.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/legacy/InfiniTAM/cmake/UseOpenGL.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/legacy/InfiniTAM/cmake/UseAzureKinect.cmake)
add_subdirectory(legacy/InfiniTAM)


##################### LIBTORCH and OpenCV ############################
# Find OpenCV BEFORE Torch to avoid protobuf target conflicts
# (OpenCV from vcpkg uses vcpkg's protobuf, Torch ships its own)
find_package(OpenCV REQUIRED)
set(OpenCV_LIBS opencv_core opencv_imgproc opencv_highgui opencv_calib3d)

# Workaround: CUDA 12.x removed nvToolsExt as a separate library (now header-only NVTX3)
# LibTorch's cmake expects CUDA::nvToolsExt, so create an empty interface target if missing
if(NOT TARGET CUDA::nvToolsExt)
    add_library(CUDA::nvToolsExt INTERFACE IMPORTED)
    # NVTX3 is header-only in CUDA 12+, just need the include path
    if(CUDAToolkit_INCLUDE_DIRS)
        set_target_properties(CUDA::nvToolsExt PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${CUDAToolkit_INCLUDE_DIRS}")
    endif()
    message(STATUS "Created CUDA::nvToolsExt interface target for CUDA 12+ compatibility")
endif()

find_package(Torch REQUIRED)

##################### My LIBS ############################
# Legacy ThirdLibs support
if(EXISTS "${CMAKE_SOURCE_DIR}/ThirdLibs/install/lib")
    set(THIRDLIBS_DIR "${CMAKE_SOURCE_DIR}/ThirdLibs/install/lib")
    link_directories(${THIRDLIBS_DIR})
endif()

# yaml-cpp
find_package(yaml-cpp QUIET)
if(yaml-cpp_FOUND)
    message(STATUS "Found yaml-cpp via find_package")
else()
    # Legacy: check ThirdLibs
    if(EXISTS "${CMAKE_SOURCE_DIR}/ThirdLibs/install/include/yaml-cpp")
        set(YAML_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/ThirdLibs/install/include/yaml-cpp")
        message(STATUS "Using yaml-cpp from ThirdLibs")
    endif()
endif()

# Protobuf - use the one from Torch/Caffe2, skip explicit find if already defined
if(NOT TARGET protobuf::libprotobuf)
    find_package(Protobuf REQUIRED)
    message(STATUS "Protobuf_VERSION: ${Protobuf_VERSION}")
    message(STATUS "Protobuf_INCLUDE_DIRS: ${Protobuf_INCLUDE_DIRS}")
    message(STATUS "Protobuf_LIBRARIES: ${Protobuf_LIBRARIES}")
else()
    message(STATUS "Using protobuf from Torch/Caffe2")
    # Get protobuf libraries from the target
    get_target_property(Protobuf_LIBRARIES protobuf::libprotobuf LOCATION)
endif()

# eigen
find_package(Eigen3 REQUIRED NO_MODULE)

# Pangolin
find_package(Pangolin 0.8 REQUIRED)

# Boost (for remote_viewer networking)
find_package(Boost COMPONENTS system QUIET)
if(Boost_FOUND)
    message(STATUS "Found Boost: ${Boost_VERSION}")
else()
    message(STATUS "Boost not found - remote_viewer will not be built")
endif()

# NVML (nvidia-ml) - platform specific
if(WIN32)
    # On Windows, NVML is nvml.lib in the CUDA toolkit
    find_library(NVML_LIBRARY nvml PATHS "${CUDAToolkit_LIBRARY_DIR}" NO_DEFAULT_PATH)
    if(NOT NVML_LIBRARY)
        set(NVML_LIBRARY "") # Make optional
        message(STATUS "NVML not found - GPU monitoring disabled")
    endif()
else()
    find_library(NVML_LIBRARY nvidia-ml PATHS /usr/lib/x86_64-linux-gnu)
endif()

##############################################################
#####################   gsplat rasterizer
set(RASTERIZER_LIBS rasterizer)
file(GLOB RASTERIZER_SRC_FILES ${PROJECT_SOURCE_DIR}/legacy/gsplat/rasterizer/*.cu ${PROJECT_SOURCE_DIR}/legacy/gsplat/rasterizer/*.cuh)

add_library(rasterizer ${RASTERIZER_SRC_FILES})
target_link_libraries(rasterizer PUBLIC cuda)
target_include_directories(rasterizer PRIVATE
    ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
    ${TORCH_INCLUDE_DIRS}
)
target_link_libraries(rasterizer PUBLIC glm::glm-header-only)
set_target_properties(rasterizer PROPERTIES LINKER_LANGUAGE CXX)


##############################################################
#####################   gsplat c++ binding
file(GLOB GSPLAT_SRC_FILES ${PROJECT_SOURCE_DIR}/legacy/gsplat/*.cpp)
add_library(gsplat ${GSPLAT_SRC_FILES})
install(TARGETS gsplat DESTINATION bin)
set_property(TARGET gsplat PROPERTY CXX_STANDARD 17)
target_include_directories(gsplat PRIVATE
    ${PROJECT_SOURCE_DIR}/legacy/gsplat/rasterizer
    ${PROJECT_SOURCE_DIR}/include
    ${GPU_INCLUDE_DIRS}
)
target_link_libraries(gsplat PUBLIC ${STDPPFS_LIBRARY} ${GPU_LIBRARIES} ${RASTERIZER_LIBS} ${TORCH_LIBRARIES} ${OpenCV_LIBS})

target_link_libraries(gsplat PUBLIC
    nlohmann_json::nlohmann_json
    cxxopts::cxxopts
    nanoflann::nanoflann
)
if (NOT WIN32)
    target_link_libraries(gsplat PUBLIC pthread)
endif()
target_compile_definitions(gsplat PRIVATE USE_CUDA)


file(GLOB OURS_SRC_FILES ${PROJECT_SOURCE_DIR}/src/*.cpp)
file(GLOB SLAM_SRC_FILES ${PROJECT_SOURCE_DIR}/legacy/slam/*.cpp ${PROJECT_SOURCE_DIR}/legacy/slam/TsdfFusion/*.cpp)
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -march=native")

##############################################################
#####################   ours implementation of slam train
add_executable(slam_trainer slam_trainer.cpp ${OURS_SRC_FILES} ${SLAM_SRC_FILES})
target_link_libraries(slam_trainer PUBLIC
        gsplat
        Eigen3::Eigen
        indicators::indicators
        tinyply
        $<$<BOOL:${USE_TENSORBOARD}>:tensorboard_logger>
        yaml-cpp::yaml-cpp
        ${Protobuf_LIBRARIES}
        InputSource
        ITMLib
        MiniSlamGraphLib
        ORUtils
        FernRelocLib
        cuda
        ${NVML_LIBRARY}
        ${OPENGL_LIBRARY}
        ${GLUT_LIBRARY}
        ${K4A_LIBRARIES}
)
if(USE_TENSORBOARD)
    target_compile_definitions(slam_trainer PRIVATE USE_TENSORBOARD)
endif()
target_include_directories(slam_trainer PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/legacy/slam
    ${PROJECT_SOURCE_DIR}/legacy/gsplat
    ${PROJECT_SOURCE_DIR}/legacy/InfiniTAM
    ${tinyply_SOURCE_DIR}/source
)

#####################   ours implementation of remote viewer
if(Boost_FOUND)
    add_executable(remote_viewer remote_viewer.cpp ${OURS_SRC_FILES} ${SLAM_SRC_FILES})
    target_link_libraries(remote_viewer PUBLIC
            gsplat
            Eigen3::Eigen
            indicators::indicators
            tinyply
            Boost::system
            $<$<BOOL:${USE_TENSORBOARD}>:tensorboard_logger>
            yaml-cpp::yaml-cpp
            ${Protobuf_LIBRARIES}
            InputSource
            ITMLib
            MiniSlamGraphLib
            ORUtils
            FernRelocLib
            cuda
            ${NVML_LIBRARY}
            ${OPENGL_LIBRARY}
            ${GLUT_LIBRARY}
            ${K4A_LIBRARIES}
    )
    target_include_directories(remote_viewer PRIVATE
        ${Boost_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/legacy/slam
        ${PROJECT_SOURCE_DIR}/legacy/gsplat
        ${PROJECT_SOURCE_DIR}/legacy/InfiniTAM
        ${tinyply_SOURCE_DIR}/source
    )
    if(USE_TENSORBOARD)
        target_compile_definitions(remote_viewer PRIVATE USE_TENSORBOARD)
    endif()
else()
    message(STATUS "Skipping remote_viewer (requires Boost)")
endif()

#####################   online SLAM with Azure Kinect
add_executable(online_slam online_slam.cpp ${OURS_SRC_FILES} ${SLAM_SRC_FILES})
target_link_libraries(online_slam PUBLIC
        gsplat
        Eigen3::Eigen
        indicators::indicators
        tinyply
        $<$<BOOL:${USE_TENSORBOARD}>:tensorboard_logger>
        yaml-cpp::yaml-cpp
        ${Protobuf_LIBRARIES}
        InputSource
        ITMLib
        MiniSlamGraphLib
        ORUtils
        FernRelocLib
        cuda
        ${NVML_LIBRARY}
        ${OPENGL_LIBRARY}
        ${GLUT_LIBRARY}
        ${K4A_LIBRARIES}
)
if(USE_TENSORBOARD)
    target_compile_definitions(online_slam PRIVATE USE_TENSORBOARD)
endif()
target_include_directories(online_slam PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/legacy/slam
    ${PROJECT_SOURCE_DIR}/legacy/gsplat
    ${PROJECT_SOURCE_DIR}/legacy/InfiniTAM
    ${tinyply_SOURCE_DIR}/source
)

# The following code block is suggested to be used on Windows.
# According to https://github.com/pytorch/pytorch/issues/25457,
# the DLLs need to be copied to avoid memory errors.
if (MSVC)
    file(GLOB TORCH_DLLS "${TORCH_INSTALL_PREFIX}/lib/*.dll")
    file(GLOB OPENCV_DLL "${OPENCV_DIR}/x64/vc16/bin/opencv_world490.dll")
    # Azure Kinect DLLs
    if(K4A_FOUND AND K4A_DLL_DIR)
        file(GLOB K4A_DLLS "${K4A_DLL_DIR}/*.dll")
    endif()
    set(DLLS_TO_COPY ${TORCH_DLLS} ${OPENCV_DLL} ${K4A_DLLS})
    # Copy DLLs to all executable output directories
    set(TARGETS_TO_COPY slam_trainer online_slam)
    if(Boost_FOUND)
        list(APPEND TARGETS_TO_COPY remote_viewer)
    endif()
    foreach(TARGET_NAME ${TARGETS_TO_COPY})
        add_custom_command(TARGET ${TARGET_NAME}
            POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${DLLS_TO_COPY}
            $<TARGET_FILE_DIR:${TARGET_NAME}>)
    endforeach()
endif (MSVC)

add_compile_definitions(GLOG_USE_GLOG_EXPORT)

# ============================================================================
# Python Bindings via Nanobind
# ============================================================================
if(SAP_BUILD_PYTHON)
    message(STATUS "Building Python bindings...")

    # Find Python
    find_package(Python 3.10 COMPONENTS Interpreter Development.Module REQUIRED)

    # Find or fetch Nanobind
    find_package(nanobind QUIET)
    if(NOT nanobind_FOUND)
        message(STATUS "Nanobind not found, fetching from GitHub...")
        FetchContent_Declare(nanobind
            GIT_REPOSITORY https://github.com/wjakob/nanobind.git
            GIT_TAG v2.2.0
        )
        FetchContent_MakeAvailable(nanobind)
    endif()

    # Build the Python extension module
    nanobind_add_module(_core
        STABLE_ABI
        NB_STATIC
        src/bindings/module.cpp
        src/slam/tracker.cpp
    )

    target_include_directories(_core PRIVATE
        ${PROJECT_SOURCE_DIR}/src
        ${OpenCV_INCLUDE_DIRS}
    )

    target_link_libraries(_core PRIVATE
        Eigen3::Eigen
        ${OpenCV_LIBS}
    )

    # Install the extension to the Python package
    install(TARGETS _core LIBRARY DESTINATION python/spatial_ai_playground)

    message(STATUS "Python bindings configured for ${Python_EXECUTABLE}")
endif()

# ============================================================================
# New Spatial AI Playground Core Library
# ============================================================================
# This is the new modular architecture (separate from legacy)
add_library(sap_core INTERFACE)
target_include_directories(sap_core INTERFACE
    ${PROJECT_SOURCE_DIR}/core/include
    ${PROJECT_SOURCE_DIR}/src
)
target_link_libraries(sap_core INTERFACE
    Eigen3::Eigen
    ${OpenCV_LIBS}
)