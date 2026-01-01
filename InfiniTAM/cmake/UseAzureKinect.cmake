#############################
# UseAzureKinect.cmake      #
#############################

OPTION(WITH_AZUREKINECT "Build with Azure Kinect support" ON)

IF(WITH_AZUREKINECT)
    # Try to find k4a library - cross-platform paths
    IF(WIN32)
        # Windows: Azure Kinect SDK default install location
        SET(K4A_DEFAULT_ROOT "C:/Program Files/Azure Kinect SDK v1.4.1")
        FIND_PATH(K4A_INCLUDE_DIR k4a/k4a.h
            PATHS
                "${K4A_DEFAULT_ROOT}/sdk/include"
                "$ENV{K4A_ROOT}/sdk/include"
                "${K4A_ROOT}/sdk/include"
        )
        FIND_LIBRARY(K4A_LIBRARY
            NAMES k4a
            PATHS
                "${K4A_DEFAULT_ROOT}/sdk/windows-desktop/amd64/release/lib"
                "$ENV{K4A_ROOT}/sdk/windows-desktop/amd64/release/lib"
                "${K4A_ROOT}/sdk/windows-desktop/amd64/release/lib"
        )
        # Store DLL path for runtime
        SET(K4A_DLL_DIR "${K4A_DEFAULT_ROOT}/sdk/windows-desktop/amd64/release/bin" CACHE PATH "Azure Kinect DLL directory")
    ELSE()
        # Linux paths
        FIND_PATH(K4A_INCLUDE_DIR k4a/k4a.h
            PATHS
                /usr/include
                /usr/local/include
                ${K4A_ROOT}/include
                $ENV{K4A_ROOT}/include
        )
        FIND_LIBRARY(K4A_LIBRARY
            NAMES k4a
            PATHS
                /usr/lib
                /usr/lib/x86_64-linux-gnu
                /usr/local/lib
                ${K4A_ROOT}/lib
                $ENV{K4A_ROOT}/lib
        )
    ENDIF()

    IF(K4A_INCLUDE_DIR AND K4A_LIBRARY)
        MESSAGE(STATUS "Found Azure Kinect SDK:")
        MESSAGE(STATUS "  Include: ${K4A_INCLUDE_DIR}")
        MESSAGE(STATUS "  Library: ${K4A_LIBRARY}")

        INCLUDE_DIRECTORIES(${K4A_INCLUDE_DIR})
        ADD_DEFINITIONS(-DCOMPILE_WITH_AzureKinect)
        SET(K4A_FOUND TRUE CACHE BOOL "Azure Kinect SDK found" FORCE)
        SET(K4A_LIBRARIES ${K4A_LIBRARY} CACHE STRING "Azure Kinect libraries" FORCE)
    ELSE()
        MESSAGE(STATUS "Azure Kinect SDK not found - building without Azure Kinect support")
        MESSAGE(STATUS "  To enable, install libk4a1.4-dev or set K4A_ROOT")
        SET(K4A_FOUND FALSE CACHE BOOL "Azure Kinect SDK not found" FORCE)
        SET(K4A_LIBRARIES "" CACHE STRING "Azure Kinect libraries" FORCE)
    ENDIF()
ELSE()
    SET(K4A_FOUND FALSE CACHE BOOL "Azure Kinect disabled" FORCE)
    SET(K4A_LIBRARIES "" CACHE STRING "Azure Kinect libraries" FORCE)
ENDIF()
