###################
# UseGLUT.cmake #
###################

IF(WIN32)
  # On Windows, use freeglut from vcpkg or system paths
  FIND_LIBRARY(GLUT_LIBRARY freeglut HINTS "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/lib")
  FIND_PATH(GLUT_INCLUDE_DIR GL/glut.h HINTS "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/include")
  IF(NOT GLUT_LIBRARY)
    # Fallback to find_package
    FIND_PACKAGE(FreeGLUT CONFIG)
    IF(FreeGLUT_FOUND)
      SET(GLUT_LIBRARY FreeGLUT::freeglut)
    ENDIF()
  ENDIF()
ELSEIF(APPLE)
  FIND_PACKAGE(GLUT REQUIRED)
ELSEIF("${CMAKE_SYSTEM}" MATCHES "Linux")
  FIND_LIBRARY(GLUT_LIBRARY glut HINTS "/usr/lib/x86_64-linux-gnu")
  FIND_PATH(GLUT_INCLUDE_DIR glut.h HINTS "/usr/include/GL")
ENDIF()
MESSAGE(STATUS "glut library: ${GLUT_LIBRARY}")
MESSAGE(STATUS "glut include: ${GLUT_INCLUDE_DIR}")
INCLUDE_DIRECTORIES(${GLUT_INCLUDE_DIR})
