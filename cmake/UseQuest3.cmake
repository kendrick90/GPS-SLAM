# UseQuest3.cmake
# Configuration for Meta Quest 3 support
#
# This module handles:
# - Quest 3 SDK detection (for on-device builds)
# - Streaming client configuration (for PC-side builds)
# - Android NDK configuration for cross-compilation
#
# Options:
#   WITH_QUEST3 - Enable Quest 3 support (default: OFF)
#   QUEST3_ON_DEVICE - Build for on-device execution (requires Android NDK)
#   QUEST3_STREAMING - Build streaming client for PC-side processing
#
# This module defines:
#   QUEST3_FOUND - Quest 3 support is available
#   QUEST3_LIBRARIES - Libraries to link
#   QUEST3_INCLUDE_DIRS - Include directories
#   QUEST3_DEFINITIONS - Preprocessor definitions

option(WITH_QUEST3 "Enable Meta Quest 3 support" OFF)
option(QUEST3_ON_DEVICE "Build for on-device Quest 3 execution" OFF)
option(QUEST3_STREAMING "Build Quest 3 streaming client" ON)

if(NOT WITH_QUEST3)
    set(QUEST3_FOUND FALSE)
    return()
endif()

message(STATUS "Configuring Quest 3 support...")

set(QUEST3_DEFINITIONS "")
set(QUEST3_LIBRARIES "")
set(QUEST3_INCLUDE_DIRS "")

# ============================================================================
# On-Device Build (Android NDK)
# ============================================================================
if(QUEST3_ON_DEVICE)
    if(NOT ANDROID)
        message(FATAL_ERROR "QUEST3_ON_DEVICE requires Android NDK cross-compilation. "
                           "Use -DCMAKE_TOOLCHAIN_FILE=path/to/android.toolchain.cmake")
    endif()

    # Check for Meta Quest SDK / Oculus Mobile SDK
    if(DEFINED ENV{OCULUS_SDK_PATH})
        set(OCULUS_SDK_PATH $ENV{OCULUS_SDK_PATH})
    elseif(DEFINED OCULUS_SDK_PATH)
        # Use CMake variable
    else()
        message(WARNING "OCULUS_SDK_PATH not set. Quest 3 features may be limited.")
    endif()

    if(OCULUS_SDK_PATH AND EXISTS "${OCULUS_SDK_PATH}")
        list(APPEND QUEST3_INCLUDE_DIRS "${OCULUS_SDK_PATH}/VrApi/Include")
        list(APPEND QUEST3_INCLUDE_DIRS "${OCULUS_SDK_PATH}/1stParty/OVR/Include")

        # Find OVR libraries
        find_library(OVR_LIBRARY vrapi
            PATHS "${OCULUS_SDK_PATH}/VrApi/Libs/Android/${ANDROID_ABI}"
        )
        if(OVR_LIBRARY)
            list(APPEND QUEST3_LIBRARIES ${OVR_LIBRARY})
        endif()
    endif()

    # Android Camera2 NDK (for Passthrough Camera API)
    find_library(CAMERA2_NDK_LIBRARY camera2ndk)
    if(CAMERA2_NDK_LIBRARY)
        list(APPEND QUEST3_LIBRARIES ${CAMERA2_NDK_LIBRARY})
    endif()

    # Android Media NDK (for image processing)
    find_library(MEDIANDK_LIBRARY mediandk)
    if(MEDIANDK_LIBRARY)
        list(APPEND QUEST3_LIBRARIES ${MEDIANDK_LIBRARY})
    endif()

    list(APPEND QUEST3_DEFINITIONS "QUEST3_ON_DEVICE")
    message(STATUS "Quest 3 on-device build configured")

endif()

# ============================================================================
# Streaming Client (PC-side)
# ============================================================================
if(QUEST3_STREAMING)
    # For streaming, we need networking libraries
    # The actual Quest 3 streaming protocol can be implemented using:
    # - WebRTC (for low-latency video)
    # - gRPC (for control/metadata)
    # - Raw UDP (for custom protocol)

    # Check for Boost.Asio (for async networking)
    find_package(Boost COMPONENTS system QUIET)
    if(Boost_FOUND)
        list(APPEND QUEST3_LIBRARIES Boost::system)
        list(APPEND QUEST3_DEFINITIONS "QUEST3_STREAMING_BOOST")
    endif()

    # Optional: WebRTC for video streaming
    find_package(WebRTC QUIET)
    if(WebRTC_FOUND)
        list(APPEND QUEST3_LIBRARIES WebRTC::WebRTC)
        list(APPEND QUEST3_DEFINITIONS "QUEST3_STREAMING_WEBRTC")
        message(STATUS "Quest 3 streaming with WebRTC enabled")
    else()
        message(STATUS "WebRTC not found - using custom streaming protocol")
    endif()

    list(APPEND QUEST3_DEFINITIONS "QUEST3_STREAMING")
    message(STATUS "Quest 3 streaming client configured")
endif()

# ============================================================================
# Common configuration
# ============================================================================
list(APPEND QUEST3_DEFINITIONS "WITH_QUEST3")

# Quest 3 hand tracking support
list(APPEND QUEST3_DEFINITIONS "QUEST3_HAND_TRACKING")

# Quest 3 eye tracking (requires user permission)
option(QUEST3_EYE_TRACKING "Enable Quest 3 eye tracking" OFF)
if(QUEST3_EYE_TRACKING)
    list(APPEND QUEST3_DEFINITIONS "QUEST3_EYE_TRACKING")
endif()

# Quest 3 spatial anchors
list(APPEND QUEST3_DEFINITIONS "QUEST3_SPATIAL_ANCHORS")

set(QUEST3_FOUND TRUE)

# Summary
message(STATUS "Quest 3 support enabled:")
message(STATUS "  On-device: ${QUEST3_ON_DEVICE}")
message(STATUS "  Streaming: ${QUEST3_STREAMING}")
message(STATUS "  Eye tracking: ${QUEST3_EYE_TRACKING}")
message(STATUS "  Definitions: ${QUEST3_DEFINITIONS}")
