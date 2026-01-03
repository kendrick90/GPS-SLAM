# FindRerun.cmake
# Find or fetch the Rerun SDK for C++/Python visualization
#
# This module defines:
#   RERUN_FOUND - System has Rerun SDK
#   RERUN_INCLUDE_DIRS - Rerun include directories
#   RERUN_LIBRARIES - Rerun libraries to link
#   rerun_sdk::rerun_sdk - Imported target
#
# Optional configuration:
#   RERUN_VERSION - Version to fetch (default: 0.21.0)
#   RERUN_USE_FETCHCONTENT - Force FetchContent instead of find_package

if(NOT DEFINED RERUN_VERSION)
    set(RERUN_VERSION "0.21.0")
endif()

# First try to find an installed version
if(NOT RERUN_USE_FETCHCONTENT)
    find_package(rerun_sdk ${RERUN_VERSION} QUIET CONFIG)
    if(rerun_sdk_FOUND)
        message(STATUS "Found installed Rerun SDK: ${rerun_sdk_VERSION}")
        set(RERUN_FOUND TRUE)
        return()
    endif()
endif()

# Fetch from GitHub if not found
message(STATUS "Rerun SDK not found locally, fetching version ${RERUN_VERSION} from GitHub...")

include(FetchContent)

# Rerun provides pre-built binaries for the C++ SDK
# For Windows x64, we can download the pre-built release
if(WIN32 AND CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(RERUN_PLATFORM "x86_64-pc-windows-msvc")
    set(RERUN_EXT "zip")
elseif(APPLE)
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm64")
        set(RERUN_PLATFORM "aarch64-apple-darwin")
    else()
        set(RERUN_PLATFORM "x86_64-apple-darwin")
    endif()
    set(RERUN_EXT "tar.gz")
else()
    set(RERUN_PLATFORM "x86_64-unknown-linux-gnu")
    set(RERUN_EXT "tar.gz")
endif()

set(RERUN_SDK_URL "https://github.com/rerun-io/rerun/releases/download/${RERUN_VERSION}/rerun_cpp_sdk-${RERUN_PLATFORM}.${RERUN_EXT}")

FetchContent_Declare(
    rerun_sdk
    URL ${RERUN_SDK_URL}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)

FetchContent_MakeAvailable(rerun_sdk)

# Set up the imported target
if(NOT TARGET rerun_sdk::rerun_sdk)
    add_library(rerun_sdk::rerun_sdk INTERFACE IMPORTED GLOBAL)
    set_target_properties(rerun_sdk::rerun_sdk PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${rerun_sdk_SOURCE_DIR}/include"
    )

    # Find the library file
    if(WIN32)
        find_library(RERUN_LIBRARY
            NAMES rerun_c rerun_sdk
            PATHS "${rerun_sdk_SOURCE_DIR}/lib"
            NO_DEFAULT_PATH
        )
    else()
        find_library(RERUN_LIBRARY
            NAMES rerun_c rerun_sdk librerun_c librerun_sdk
            PATHS "${rerun_sdk_SOURCE_DIR}/lib"
            NO_DEFAULT_PATH
        )
    endif()

    if(RERUN_LIBRARY)
        set_target_properties(rerun_sdk::rerun_sdk PROPERTIES
            INTERFACE_LINK_LIBRARIES "${RERUN_LIBRARY}"
        )
    endif()
endif()

set(RERUN_FOUND TRUE)
set(RERUN_INCLUDE_DIRS "${rerun_sdk_SOURCE_DIR}/include")
set(RERUN_LIBRARIES rerun_sdk::rerun_sdk)

message(STATUS "Rerun SDK configured from: ${rerun_sdk_SOURCE_DIR}")
